import os
import time
import argparse

def parse_args():
    """Parse input arguments."""
    desc = ('Use TPLink protocol to communicate with Autopilot')
    parser = argparse.ArgumentParser(description=desc)
    parser.add_argument('-p', '--port_num', type=int, default=80, help='communicate serial port number')
    args = parser.parse_args()
    return args

def main():
    args = parse_args()
    if args.category_num <= 0:
        raise SystemExit('ERROR: bad category_num (%d)!' % args.category_num)

if __name__ == '__main__':
    main()
