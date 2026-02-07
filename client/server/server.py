import json
import http.client
import urllib.parse
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn
from pathlib import Path
import math


# ============================================================
# HTTP SERVER
# ============================================================

UI_LISTEN_ADDR = "0.0.0.0:8080"
STATIC_DIR = Path(__file__).parent / "static"

MEDIAMTX_ORIGIN = "http://127.0.0.1:8889"
ORIGIN = urllib.parse.urlsplit(MEDIAMTX_ORIGIN)
if ORIGIN.hostname is None:
    raise RuntimeError("MEDIAMTX_ORIGIN must include a hostname")

ORIGIN_HOST = ORIGIN.hostname
ORIGIN_SCHEME = ORIGIN.scheme
ORIGIN_PORT = ORIGIN.port or (443 if ORIGIN_SCHEME == "https" else 80)

# Hop-by-hop headers we should drop when proxying - not used heavily, but good to know.
HOP_BY_HOP = {
    "connection", "keep-alive", "proxy-authenticate", "proxy-authorization",
    "te", "trailers", "transfer-encoding", "upgrade"
}


class ThreadingHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True


def filter_headers(headers):
    out = {}
    for k, v in headers.items():
        lk = k.lower()
        if lk in HOP_BY_HOP or lk == "host":
            continue
        out[k] = v
    return out


class Handler(BaseHTTPRequestHandler):
    rover_ip = None

    def do_OPTIONS(self):
        if self.is_whep_path():
            self.send_response(204)
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Access-Control-Allow-Methods", "GET,POST,PATCH,OPTIONS")
            self.send_header("Access-Control-Allow-Headers", "Content-Type,If-None-Match,If-Match")
            self.end_headers()
            return
        self.send_response(204)
        self.end_headers()

    def do_POST(self):
        # Map POST endpoints to handlers. Kept super explicit for clarity.
        if self.path == "/api/calib/start": return self.calib_A()
        if self.path == "/api/calib/right": return self.calib_B()
        if self.path == "/api/calib/forward": return self.calib_C()
        if self.path == "/api/calib/finish": return self.calib_finish()
        if self.path == "/api/rover-command": return self.send_rover_cmd()
        if self.is_whep_path(): return self.proxy_whep()
        self.send_error(404)

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            return self.serve_file("index.html")
        if self.path.startswith("/static/"):
            return self.serve_file(self.path[8:])
        if self.is_whep_path():
            return self.proxy_whep()
        self.send_error(404)

    def do_PATCH(self):
        if self.is_whep_path():
            return self.proxy_whep()
        self.send_error(404)

    def is_whep_path(self):
        p = self.path
        return (
                p.startswith("/whep/") or
                p == "/drone/whep" or
                p.startswith("/drone/whep/")
        )

    def calib_A(self):
        # Save the current drone local pose as calibration point A
        global A
        A = latest_drone_local
        ws_broadcast(f"CALIB_A,{A[0]},{A[1]}")
        return self._ok("Saved A")

    def calib_B(self):
        # Save the current drone local pose as calibration point B
        global B
        B = latest_drone_local
        ws_broadcast(f"CALIB_B,{B[0]},{B[1]}")
        return self._ok("Saved B")

    def calib_C(self):
        # Save the current drone local pose as calibration point C
        global C
        C = latest_drone_local
        ws_broadcast(f"CALIB_C,{C[0]},{C[1]}")
        return self._ok("Saved C")

    def calib_finish(self):
        # Compute a simple 2D transform (rotation+translation) with A as origin, ux from A->B,
        # uy from A->C. This is a bit naive (no orthonormalization), but works OK for UI mapping.
        global A, B, C, R, T, CALIBRATED

        if A is None or B is None or C is None:
            # We don't throw 400; just tell the UI we need all points
            return self._ok("ERR Missing A/B/C")

        Ax, Ay, _ = A
        Bx, By, _ = B
        Cx, Cy, _ = C

        # x-axis: from A to B
        vx = (Bx - Ax, By - Ay)
        ln = math.hypot(*vx)
        ux = (vx[0] / ln, vx[1] / ln)

        # y-axis: from A to C
        vy = (Cx - Ax, Cy - Ay)
        ln = math.hypot(*vy)
        uy = (vy[0] / ln, vy[1] / ln)

        # R is column major-style here: columns are ux and uy
        R = [
            [ux[0], uy[0]],
            [ux[1], uy[1]]
        ]

        # T: translate so that world-origin sits at A
        T = (
            -Ax * R[0][0] - Ay * R[0][1],
            -Ax * R[1][0] - Ay * R[1][1]
        )

        CALIBRATED = True
        # Notify the UI with the matrix and vector (so they can compute properly)
        ws_broadcast(f"CALIB_DONE,{R[0][0]},{R[0][1]},{R[1][0]},{R[1][1]},{T[0]},{T[1]}")
        return self._ok("Calibration complete")

    def send_rover_cmd(self):
        # Receive JSON payloads (like {"cmd": "STOP"}), forward command via UDP to rover
        length = int(self.headers.get("Content-Length", 0))
        raw = self.rfile.read(length)
        payload = json.loads(raw.decode())
        cmd = payload.get("cmd", "")

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.sendto(cmd.encode(), (self.rover_ip, 5005))
        sock.close()
        return self._ok("Sent")

    def _ok(self, msg):
        # Helper: send a simple text response with status 200
        msg_b = msg.encode()
        self.send_response(200)
        self.send_header("Content-Length", str(len(msg_b)))
        self.end_headers()
        self.wfile.write(msg_b)

    def serve_file(self, rel):
        # Serve a file from the static dir. No security checks — fine for our local dev use.
        p = STATIC_DIR / rel
        if not p.exists(): return self.send_error(404)

        data = p.read_bytes()
        ctype = {
            ".html": "text/html; charset=utf-8",
            ".css": "text/css; charset=utf-8",
            ".js": "application/javascript; charset=utf-8"
        }.get(p.suffix, "application/octet-stream")

        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def proxy_whep(self):
        incoming = urllib.parse.urlsplit(self.path)
        target_url = ORIGIN._replace(
            path=incoming.path,
            query=incoming.query or ""
        ).geturl()

        length = int(self.headers.get("Content-Length", "0") or "0")
        body = self.rfile.read(length) if length > 0 else None

        conn = None
        try:
            if ORIGIN_SCHEME == "https":
                conn = http.client.HTTPSConnection(ORIGIN_HOST, ORIGIN_PORT, timeout=20)
            else:
                conn = http.client.HTTPConnection(ORIGIN_HOST, ORIGIN_PORT, timeout=20)

            conn.request(self.command, target_url, body=body, headers=filter_headers(self.headers))
            resp = conn.getresponse()

            self.send_response(resp.status, resp.reason)
            for k, v in resp.getheaders():
                if k.lower() in HOP_BY_HOP:
                    continue
                self.send_header(k, v)
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()

            while True:
                chunk = resp.read(65536)
                if not chunk:
                    break
                self.wfile.write(chunk)
        except Exception as e:
            self.send_error(502, "MediaMTX upstream error: %s" % e)
        finally:
            if conn is not None:
                try:
                    conn.close()
                except Exception:
                    pass
