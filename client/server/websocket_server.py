import base64
import hashlib
import threading
import socket

# ============================================================
# WEBSOCKET SERVER
# ============================================================
# A really minimal websocket implementation: handshake + unmasked frames
WS_CLIENTS = []

def build_ws_frame(msg):
    # Create a small unmasked text frame (client -> server masking not needed for server->client)
    payload = msg.encode()
    L = len(payload)
    if L < 126:
        header = bytes([0x81, L])
    elif L < 65536:
        header = bytes([0x81, 126]) + L.to_bytes(2, 'big')
    else:
        header = bytes([0x81, 127]) + L.to_bytes(8, 'big')
    return header + payload

def ws_accept_client(conn):
    # Basic (insecure) websocket handshake - parses Sec-WebSocket-Key, returns Sec-WebSocket-Accept
    try:
        hdr = conn.recv(2048).decode()
        key = ""
        for line in hdr.split("\r\n"):
            if "Sec-WebSocket-Key" in line:
                key = line.split(":")[1].strip()
        magic = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
        accept = base64.b64encode(
            hashlib.sha1((key + magic).encode()).digest()
        ).decode()

        resp = (
            "HTTP/1.1 101 Switching Protocols\r\n"
            "Upgrade: websocket\r\n"
            "Connection: Upgrade\r\n"
            "Sec-WebSocket-Accept: " + accept + "\r\n\r\n"
        )
        conn.send(resp.encode())
        WS_CLIENTS.append(conn)
    except:
        # If handshake fails, close the socket and move on.
        conn.close()

def ws_broadcast(msg):
    # Broadcast a string to all connected web socket clients.
    frame = build_ws_frame(msg)
    dead = []
    for c in WS_CLIENTS:
        try:
            c.send(frame)
        except:
            dead.append(c)
    for d in dead:
        WS_CLIENTS.remove(d)

def start_ws_server():
    # Simple single-threaded accept loop; each new connection's handshake runs in a separate thread
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(("0.0.0.0", 8765))
    s.listen(5)
    print("[WS] Listening on ws://0.0.0.0:8765")
    while True:
        conn, _ = s.accept()
        threading.Thread(target=ws_accept_client, args=(conn,), daemon=True).start()