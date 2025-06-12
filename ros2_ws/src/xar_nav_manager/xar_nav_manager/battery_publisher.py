#!/usr/bin/env python3
# XAR/battery_publisher.py  – ROSbot 3 / 3 Pro (3-cell Li-ion)

import rclpy
from   rclpy.node import Node
from   sensor_msgs.msg import BatteryState
from   std_msgs.msg   import Float32
from   bisect import bisect_right

# ── 3-S Li-ion rest-voltage curve  (ASCENDING order) ───────────────
CURVE = [                  # (voltage, percent)
    ( 9.90,  0), (10.50,  5), (10.95, 10), (11.08, 20),
    (11.20, 25), (11.28, 30), (11.44, 40), (11.52, 45),
    (11.68, 55), (11.86, 60), (12.02, 70), (12.18, 80),
    (12.30, 85), (12.40, 90), (12.50, 95), (12.60,100)
]

VOLTS   = [p[0] for p in CURVE]          # separate list for bisect
PERCENTS= [p[1] for p in CURVE]

def soc_from_voltage(v):
    if v <= VOLTS[0]:              # at or below cutoff
        return 0.0
    if v >= VOLTS[-1]:             # fully charged
        return 100.0

    idx_hi = bisect_right(VOLTS, v)       # first element > v
    v_lo, soc_lo = VOLTS[idx_hi-1], PERCENTS[idx_hi-1]
    v_hi, soc_hi = VOLTS[idx_hi],   PERCENTS[idx_hi]

    # linear interpolation within the segment
    return soc_lo + (soc_hi - soc_lo) * (v - v_lo) / (v_hi - v_lo)

class BatteryPublisher(Node):
    def __init__(self):
        super().__init__("battery_publisher")

        self.pub_v  = self.create_publisher(Float32, "/battery_voltage", 10)
        self.pub_pc = self.create_publisher(Float32, "/battery_percent", 10)

        # listen only to BatteryState to avoid the topic-type clash
        self.create_subscription(BatteryState, "/battery",
                                 self._cb_state, 10)

    def _cb_state(self, msg: BatteryState):
        v   = msg.voltage
        pct = soc_from_voltage(v)

        self.pub_v .publish(Float32(data=v))
        self.pub_pc.publish(Float32(data=pct))

        if pct < 20:
            self.get_logger().warn(f"LOW BATTERY {pct:.0f}% ({v:.2f} V)")

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(BatteryPublisher())
    rclpy.shutdown()

