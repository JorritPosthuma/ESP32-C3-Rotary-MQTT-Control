#!/bin/sh
# Creates a Hue app key for the lamp knob and lists the bridge's rooms.
# Press the link button on the bridge first, then run within 30 seconds:
#   tools/hue_pair.sh 192.168.1.220
set -e
IP=${1:?usage: tools/hue_pair.sh <bridge-ip>}

REPLY=$(curl -s -X POST "http://$IP/api" -d '{"devicetype":"knob#lamp"}')
KEY=$(echo "$REPLY" | sed -n 's/.*"username":"\([^"]*\)".*/\1/p')
if [ -z "$KEY" ]; then
    echo "No key: $REPLY"
    echo "Press the link button on the bridge and run this again within 30 seconds."
    exit 1
fi

echo "Put these in include/config.h:"
echo "#define HUE_APP_KEY \"$KEY\""
curl -s "http://$IP/api/$KEY/groups" | /usr/bin/python3 -c '
import json, sys
for gid, group in json.load(sys.stdin).items():
    print(f"#define HUE_GROUP_ID {gid}  // {group[\"name\"]} ({group[\"type\"]})")
'
