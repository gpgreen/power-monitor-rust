#!/usr/bin/env python3
"""Shutdown_monitor.py

Copyright 2020 Greg Green <ggreen@bit-builder.com>

Script to detect a shutdown signal sent by the Chart Plotter Hat.  If
the signal is detected, then first kill opencpn with the 'pkill'
command, then call 'poweroff' to shutdown the Pi
cleanly.

"""

import sys
import time
import os
# raspberry pi imports
import gpiozero

# environment variables
PKILL_DELAY = "OPENCPN_PKILL_DELAY"
PKILL_USER = "OPENCPN_USER"

# raspberry pi gpio pin numbers and delay constants
SHUTDOWN_PIN = 22
MCU_RUNNING_PIN = 23
SHUTDOWN_PULSE_MINIMUM = 600              # milliseconds
SHUTDOWN_WAIT_DELAY = 20                  # milliseconds

def main():
    """ main function """
    # get environment variable(s)
    try:
        opencpn_pkill_delay = int(os.environ[PKILL_DELAY])
    except KeyError:
        print("{} environment variable not defined, quitting".format(PKILL_DELAY))
        sys.exit(1)
    try:
        opencpn_user = os.environ[PKILL_USER]
    except KeyError:
        print("{} environment variable not defined, quitting".format(PKILL_USER))
        sys.exit(1)

    # the shutdown pin, active state is high
    shutdown_button = gpiozero.Button(SHUTDOWN_PIN,
                                      pull_up=None,
                                      active_state=True,
                                      hold_time=SHUTDOWN_PULSE_MINIMUM/1000.0)

    # the "i am running" pin
    running_device = gpiozero.DigitalOutputDevice(MCU_RUNNING_PIN,
                                                  active_high=True,
                                                  initial_value=True)

    # set initial state
    running_device.on()

    sleep_interval = 1                             # start out with 1 second sleep
    while True:
        if shutdown_button.is_pressed:
            sleep_interval = SHUTDOWN_WAIT_DELAY / 1000.0
            if shutdown_button.is_held:
                print("Detected shutdown signal, powering off..")
                os.system("/usr/bin/pkill -u {} opencpn".format(opencpn_user))
                time.sleep(opencpn_pkill_delay)
                # execute the shutdown command
                os.system("/usr/bin/sudo /sbin/poweroff")
                sys.exit(0)
        else:
            sleep_interval = 1
        time.sleep(sleep_interval)

if __name__ == '__main__':
    main()
