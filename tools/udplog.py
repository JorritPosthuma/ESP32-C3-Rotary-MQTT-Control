"""Print the knobs' UDP lines (VOLKNOB/1 ...) with timestamps and append them to a log file.

Run with Apple's /usr/bin/python3: other Pythons get EHOSTUNREACH from macOS Local Network privacy.
Usage: /usr/bin/python3 tools/udplog.py [logfile] [bind-address]

While BuchardtBar holds the port, bind to the subnet broadcast address instead, for example
192.168.1.255; the knobs send to that address.
"""
import socket
import sys
from datetime import datetime

PORT = 47777
LOG = sys.argv[1] if len(sys.argv) > 1 else "udplog.txt"
BIND = sys.argv[2] if len(sys.argv) > 2 else "0.0.0.0"

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
if hasattr(socket, "SO_REUSEPORT"):
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
sock.bind((BIND, PORT))

print(f"listening on UDP {BIND}:{PORT}, logging to {LOG}", flush=True)
while True:
    data, (sender, _) = sock.recvfrom(1024)
    line = f"{datetime.now():%H:%M:%S.%f}"[:-3] + f"  {sender}  {data.decode(errors='replace').rstrip()}"
    print(line, flush=True)
    with open(LOG, "a") as f:
        f.write(line + "\n")
