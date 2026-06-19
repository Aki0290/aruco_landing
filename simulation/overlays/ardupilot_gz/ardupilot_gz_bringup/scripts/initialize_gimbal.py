#!/usr/bin/env python3
"""Apply the legacy MAVProxy RC values once SITL is available."""

import time

from pymavlink import mavutil


def main():
    connection = mavutil.mavlink_connection(
        "udpin:127.0.0.1:14551", source_system=255
    )

    while True:
        heartbeat = connection.wait_heartbeat(timeout=5)
        if heartbeat is not None:
            break
        print("Waiting for MAVProxy/ArduPilot heartbeat on 14551...")
        time.sleep(1)

    ignored = 65535
    connection.mav.rc_channels_override_send(
        connection.target_system,
        connection.target_component,
        ignored,
        ignored,
        ignored,
        ignored,
        ignored,
        ignored,
        ignored,
        1500,  # RC8: yaw centered
        1500,  # RC9: roll centered
        1300,  # RC10: camera down
        ignored,
        ignored,
        ignored,
        ignored,
        ignored,
        ignored,
        ignored,
        ignored,
    )
    print("Gimbal initialized: RC8=1500 RC9=1500 RC10=1300")


if __name__ == "__main__":
    main()
