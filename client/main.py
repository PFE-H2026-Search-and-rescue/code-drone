#!/usr/bin/env python3

import threading
import argparse
import subprocess

import time
import socket

from client.drone_infos.vio_streamer import vio_streamer
from client.server.server import UI_LISTEN_ADDR, ThreadingHTTPServer, Handler
from client.server.websocket_server import ws_broadcast, start_ws_server

# ============================================================
# GLOBAL STATE
# ============================================================
# Quick note: I'm a student so I kept things global for simplicity.
# It's not the cleanest design, but it's easier to reason about for this project.

latest_rover_x = 0.0  # rover-reported x
latest_rover_y = 0.0  # rover-reported y
latest_rover_o = 0.0  # rover orientation (heading)




def restart_voxl_services():
    # VOXL services need restarting sometimes; convenience function.
    # TODO: Could add a check to only restart if necessary.
    subprocess.call(["sudo", "systemctl", "restart",
                     "voxl-qvio-server", "voxl-vision-px4", "voxl-px4"])

    # Give the services a moment to reboot
    time.sleep(5)

    # Wait until QVIO is actually running again
    for i in range(20):
        status = subprocess.getoutput("systemctl is-active voxl-qvio-server")
        if "active" in status:
            print("[INIT] QVIO is active.")
            return
        time.sleep(0.5)

    # If not active by now we've got bigger problems (but the function will just continue).



# ============================================================
# ROVER UDP LISTENER
# ============================================================
# Listens for packets from the rover and updates the latest_rover_* globals,
# also rebroadcasts the message to any websocket clients.
def rover_udp_listener():
    global latest_rover_x, latest_rover_y, latest_rover_o

    sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0",5006))
    print("[ROVER] Listening on :5006")

    while True:
        msg,_=sock.recvfrom(1024)
        msg=msg.decode().strip()

        if msg.startswith("ROVER"):
            p = msg.split(",")
            latest_rover_x = float(p[1])
            latest_rover_y = float(p[2])
            latest_rover_o = float(p[3])

        # Rebroadcast all rover messages to UI
        ws_broadcast(msg)


# ============================================================
# LOG THREAD
# ============================================================
# Simple periodic status dump originally used for debugging.
#def log_thread():
#    while True:
#        time.sleep(10)  # Increase sleep time to reduce log frequency
#        dx, dy, yaw = latest_drone_local
#        # Reduced logging to avoid performance issues
#        # print("\n===== STATUS =====")
#        # print(f"Drone Local:   x={dx:.2f}, y={dy:.2f}, yaw={yaw:.2f}")
#        # print(f"Rover:         x={latest_rover_x:.2f}, y={latest_rover_y:.2f}, o={latest_rover_o:.2f}")
#        # print(f"R matrix:      {R}")
#        # print(f"T vector:      {T}")
#        # print("==================\n")


# ============================================================
# MAIN
# ============================================================
def main():
    host, port = UI_LISTEN_ADDR.split(":")
    httpd = ThreadingHTTPServer((host, int(port)), Handler)
    print("[HTTP] Listening on", UI_LISTEN_ADDR)
   
    httpd.serve_forever()

if __name__=="__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--roverIp",
        "-ri",
        type=str,
        default="192.168.8.2",
        help="IP address of the device"
    )
    args = parser.parse_args()
    Handler.rover_ip = args.roverIp

    # On startup, restart QVIO services (needed for VOXL). Not ideal for dev workflow,
    # but helps keep things consistent when running on the actual hardware.
    restart_voxl_services()

    # Spawn the various service threads. I use daemon threads so it will exit cleanly
    # when the main thread (HTTP server) stops.
    threading.Thread(target=start_ws_server, daemon=True).start()
    threading.Thread(target=vio_streamer, args=(args.roverIp,), daemon=True).start()
    threading.Thread(target=rover_udp_listener, daemon=True).start()
    #threading.Thread(target=log_thread, daemon=True).start()
    main()
