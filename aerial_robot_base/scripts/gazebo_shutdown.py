#!/usr/bin/env python

# Code courtsy of Sihoj and cosmicog on Github
# https://github.com/Sihoj
# https://github.com/cosmicog


import sys, signal, subprocess, time


timeout_before_kill = 0.1  # [s]
timeout_after_kill = 0.1  # [s]


def signal_handler(sig, frame):
    time.sleep(timeout_before_kill)
    subprocess.call("killall -q gzclient & killall -q gzserver", shell=True)
    time.sleep(timeout_after_kill)
    subprocess.call("killall -9 -q gzclient & killall -9 -q gzserver", shell=True)
    sys.exit(0)


if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    input("aaaaaaa")