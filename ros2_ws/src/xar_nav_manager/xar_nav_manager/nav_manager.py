#!/usr/bin/env python3
# nav_manager.py  –  18 Jun 2025  “odom-tf-ready”
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import String
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


class NavManager(Node):
    def __init__(self):
        super().__init__('nav_manager')

        self.navigator = BasicNavigator()
        self.waypoints = []          # list[PoseStamped]
        self.state     = 'idle'      # idle → toA → waiting → toB → done

        self.status_pub = self.create_publisher(String, '/robot_status', 10)
        self.create_subscription(PoseStamped, '/goal_pose',
                                 self.on_waypoint, 10)
        self.create_subscription(String, '/mission_cmd',
                                 self.on_cmd, 10)

        # Wait until Nav2 stack is active
        self.navigator.waitUntilNav2Active()
        self.say('NavManager ready – Nav2 active')

        # Optional: set an initial pose if you use AMCL / SLAM Toolbox
        self.set_initial_pose()

    # ───────────────────────────────────────────────────────────
    def set_initial_pose(self):
        init = PoseWithCovarianceStamped()
        init.header.frame_id = 'map'
        init.pose.pose.orientation.w = 1.0
        self.navigator.setInitialPose(init)

    # ── callbacks ──────────────────────────────────────────────
    def on_waypoint(self, msg: PoseStamped):
        self.waypoints.append(msg)
        letter = chr(64 + len(self.waypoints))     # A, B, C …
        self.say(f'Pole {letter} received')

    def on_cmd(self, msg: String):
        cmd = msg.data.upper()
        if cmd == 'GO' and self.state == 'idle' and self.waypoints:
            self.drive_next('A')
        elif cmd == 'START' and self.state == 'waiting' and self.waypoints:
            self.drive_next('B')
        elif cmd == 'CANCEL':
            self.navigator.cancelTask()
            self.state = 'idle'
            self.say('Mission cancelled')

    # ── helpers ───────────────────────────────────────────────
    def drive_next(self, tag: str):
        goal = self.waypoints.pop(0)
        self.say(f'Navigating to pole {tag}')
        self.navigator.goToPose(goal)
        self.state = 'toA' if tag == 'A' else 'toB'

    def say(self, txt: str):
        self.status_pub.publish(String(data=txt))
        self.get_logger().info(txt)

    # ── spin loop ─────────────────────────────────────────────
    def spin(self):
        rclpy.spin_once(self, timeout_sec=0.1)
        if self.navigator.isTaskComplete():
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                if self.state == 'toA':
                    self.say('Arrived at A – waiting for START')
                    self.state = 'waiting'
                else:
                    self.say('Arrived at B – mission done')
                    self.state = 'done'
            elif result == TaskResult.CANCELED:
                self.say('Navigation cancelled')
                self.state = 'idle'


def main():
    rclpy.init()
    node = NavManager()
    try:
        while rclpy.ok():
            node.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

