#!/usr/bin/env python3
"""
extract_poses_ros1.py
Sample PoseStamped messages from a ROS1 bag at a fixed rate and save to TXT.

Output columns (space-separated):
stamp_sec stamp_nsec x y z qx qy qz qw frame_id
"""
import argparse
from pathlib import Path
import rosbag
from geometry_msgs.msg import PoseStamped

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", "-b", required=True, help="Path to .bag")
    ap.add_argument("--topic", "-t", required=True, help="PoseStamped topic, e.g. /pose")
    ap.add_argument("--rate", '-r', type=float, required=True, help="Sampling rate in Hz (e.g. 10)")
    ap.add_argument("--output", "-o", required=True, help="Output .txt path")
    ap.add_argument("--start", type=float, default=None, help="Start time offset [s] from bag begin")
    ap.add_argument("--end", type=float, default=None, help="End time offset [s] from bag begin")
    args = ap.parse_args()

    bag_path = Path(args.bag)
    out_path = Path(args.output)
    step = 1.0 / args.rate

    with rosbag.Bag(str(bag_path), "r") as bag, out_path.open("w") as f:
        # Resolve absolute time window if provided
        bag_start = bag.get_start_time()
        bag_end   = bag.get_end_time()
        t0 = bag_start if args.start is None else bag_start + args.start
        t1 = bag_end   if args.end   is None else min(bag_end, bag_start + args.end)

        print(f"start time: {t0}, end_time: {t1}, duration {t1 - t0}")
        next_t = t0
        for topic, msg, t in bag.read_messages(topics=[args.topic]):
            # Skip outside window
            ts = t.to_sec()
            if ts < t0:
                continue
            if ts > t1:
                break
            # if not isinstance(msg, PoseStamped):
            #     continue

            # Fixed-rate gating
            # if ts + 1e-12 >= next_t:
            if True:
                p = msg.pose.position
                q = msg.pose.orientation
                print(p)
                frame_id = msg.header.frame_id or ""
                f.write(f"{msg.header.stamp.secs} {msg.header.stamp.nsecs} "
                        f"{p.x:.9f} {p.y:.9f} {p.z:.9f} "
                        f"{q.x:.9f} {q.y:.9f} {q.z:.9f} {q.w:.9f} "
                        f"{frame_id}\n")
                # Advance gate to the smallest multiple >= ts
                k = int((ts - t0) / step) + 1
                next_t = t0 + k * step

if __name__ == "__main__":
    main()
