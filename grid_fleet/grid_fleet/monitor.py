#!/usr/bin/env python3
# ============================================================
# monitor.py  —  Student 5 (Part B)
# Monitor Node  |  ROS2 / rclpy
# ============================================================

import time
import rclpy
from rclpy.node import Node
from grid_fleet_interfaces.msg import VehicleState


STUCK_THRESHOLD = 10.0


class MonitorNode(Node):

    def __init__(self):
        super().__init__("monitor")

        # vehicle_name -> state string
        self.vehicle_states = {}

        # vehicle_name -> timestamp of last state change
        self.state_change_time = {}

        self.create_subscription(
            VehicleState,
            "/vehicle_state",
            self.handle_vehicle_state,
            10
        )

        # Print dashboard every 3 seconds
        self.create_timer(3.0, self.print_dashboard)

        self.get_logger().info("Monitor node started.")

    def handle_vehicle_state(self, msg):
        name      = msg.vehicle_name
        new_state = msg.state
        old_state = self.vehicle_states.get(name)

        if old_state != new_state:
            self.vehicle_states[name] = new_state
            self.state_change_time[name] = time.time()
        else:
            if name not in self.state_change_time:
                self.state_change_time[name] = time.time()

    def print_dashboard(self):
        self.get_logger().info("=" * 52)
        self.get_logger().info("   FLEET MONITOR DASHBOARD")
        self.get_logger().info("=" * 52)

        now          = time.time()
        waiting_list = []
        stuck_list   = []

        for name in sorted(self.vehicle_states):
            state   = self.vehicle_states[name]
            elapsed = now - self.state_change_time.get(name, now)

            self.get_logger().info(
                f"  {name:<12} | {state:<22} | {elapsed:.1f}s in state"
            )

            if state == "WAITING":
                waiting_list.append(name)

            if elapsed > STUCK_THRESHOLD and state not in ("IDLE", "FINISHED"):
                stuck_list.append(name)

        self.get_logger().info("-" * 52)
        self.get_logger().info(f"  Waiting : {waiting_list if waiting_list else 'None'}")

        for name in stuck_list:
            self.get_logger().warn(
                f"ALERT: {name} is STUCK in '{self.vehicle_states[name]}' for >10s!"
            )

        self.get_logger().info("=" * 52)


def main(args=None):
    rclpy.init(args=args)
    node = MonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()