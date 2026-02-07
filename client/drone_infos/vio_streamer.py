# ============================================================
# VIO STREAMER (where the FIX happens)
# ============================================================
# This spawns "voxl-inspect-qvio" and parses its stdout. The code
# extracts x,y and yaw, converts to a local relative frame using an initial
# offset, and broadcasts both to the UI and the rover over UDP.
pose_regex = re.compile(r"\|\s*([-+]?\d*\.\d+|\d+)\s+([-+]?\d*\.\d+|\d+)\s+([-+]?\d*\.\d+|\d+)\|")
rpy_regex = re.compile(
    r"\|\s*[-0-9.]+\s+[-0-9.]+\s+[-0-9.]+\|\s*([-0-9.]+)\s+([-0-9.]+)\s+([-0-9.]+)\|"
)
quality_regex = re.compile(r"\|\s*\d+\s*\|\s*(\d+)%")

INITIAL_X=None  # Starting offset so we don't use absolute world coords
INITIAL_Y=None
INITIAL_YAW=None

def normalize_angle(a):
    # Keep angles between -180 and 180 for nicer UI
    while a>180: a-=360
    while a<-180: a+=360
    return a

def vio_streamer(roverIp):
    # This is the meat: parse output, track initial offsets, compute local coords,
    # optionally map to world coords if calibration is done, then broadcast.
    global latest_drone_local, INITIAL_X, INITIAL_Y, INITIAL_YAW, CALIBRATED

    vio = subprocess.Popen(
        ["sudo", "voxl-inspect-qvio"],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        universal_newlines=True, bufsize=1
    )

    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    while True:
        line = vio.stdout.readline()
        if not line: break
        line=line.strip()

        # The parser is pretty lax; it looks for the patterns and skips any non-match lines
        pose=pose_regex.search(line)
        rpy=rpy_regex.search(line)
        q=quality_regex.search(line)

        # If we don't have a full pose+orientation, ignore this line.
        if not pose or not rpy:
            continue

        raw_x = float(pose.group(1))
        raw_y = float(pose.group(2))
        yaw_raw = float(rpy.group(3))

        # Initialize offsets from the first good reading
        if INITIAL_Y is None:
            INITIAL_X = raw_x
            INITIAL_Y = raw_y

        if INITIAL_YAW is None:
            # yaw might be noisy at startup; don't set until it seems plausible
            if abs(yaw_raw) > 0.1:
                INITIAL_YAW = yaw_raw
            else:
                continue

        # Compute local displacement and normalized yaw relative to the initial pose
        dx = raw_x - INITIAL_X
        dy = raw_y - INITIAL_Y
        yaw = normalize_angle(yaw_raw - INITIAL_YAW)

        latest_drone_local = (dx, dy, yaw)

        # Compute world coords for the UI only if the operator did a calibration
        if CALIBRATED:
            world_x = R[0][0]*dx + R[0][1]*dy + T[0]
            world_y = R[1][0]*dx + R[1][1]*dy + T[1]
        else:
            world_x, world_y = dx, dy

        ts=time.time()
        quality = q.group(1) if q else "-"

        # Send WORLD coords to UI via websocket
        ws_broadcast(f"{ts:.3f},{world_x:.3f},{world_y:.3f},{yaw:.3f},{quality}")

        # Send LOCAL coords to rover (we use world_x here so rover and UI are consistent)
        rover_packet = f"{ts:.3f},{world_x:.3f},{world_y:.3f},{quality}"
        udp.sendto(rover_packet.encode(), (roverIp, 5005))
