#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from grid_fleet_interfaces.msg import VehiclePosition, VehicleState
from grid_fleet_interfaces.srv import RequestTask, RequestMove

# الـ states
IDLE              = "IDLE"
REQUEST_TASK      = "REQUEST_TASK"
MOVING_TO_PICKUP  = "MOVING_TO_PICKUP"
MOVING_TO_DROPOFF = "MOVING_TO_DROPOFF"
WAITING           = "WAITING"
FINISHED          = "FINISHED"

VEHICLE_STARTS = {
    "vehicle1": (0, 0),
    "vehicle2": (7, 0),
    "vehicle3": (0, 7),
}

class Vehicle(Node):
    def __init__(self, name):
        super().__init__(name)
        self.v_id = name

        sx, sy = VEHICLE_STARTS[name]
        self.x, self.y = sx, sy

        self.state           = IDLE
        self.current_task    = None
        self.current_task_id = -1
        self.prev_state      = MOVING_TO_PICKUP

        # publishers
        self.pos_pub   = self.create_publisher(VehiclePosition, '/vehicle_position', 10)
        self.state_pub = self.create_publisher(VehicleState,    '/vehicle_state',    10)

        # clients
        self.t_cli = self.create_client(RequestTask, '/request_task')
        self.m_cli = self.create_client(RequestMove, '/request_move')

        self.get_logger().info(f"[{self.v_id}] waiting for services...")
        self.t_cli.wait_for_service()
        self.m_cli.wait_for_service()
        self.get_logger().info(f"[{self.v_id}] ready at ({self.x},{self.y})")

    # ── publish ──────────────────────────────────────
    def pub_pos(self):
        m = VehiclePosition()
        m.vehicle_name = self.v_id
        m.x = self.x
        m.y = self.y
        self.pos_pub.publish(m)

    def pub_state(self):
        m = VehicleState()
        m.vehicle_name = self.v_id
        m.state = self.state
        self.state_pub.publish(m)

    def set_state(self, s):
        self.get_logger().info(f"[{self.v_id}] {self.state} -> {s}")
        self.state = s
        self.pub_state()

    # ── طلب task ─────────────────────────────────────
    def get_new_task(self):
        req = RequestTask.Request()
        req.completed_task_id = self.current_task_id
        future = self.t_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()

        if resp.has_task:
            self.current_task = {
                "id":        resp.task_id,
                "pickup_x":  resp.pickup_x,
                "pickup_y":  resp.pickup_y,
                "dropoff_x": resp.dropoff_x,
                "dropoff_y": resp.dropoff_y,
            }
            self.current_task_id = resp.task_id
            self.get_logger().info(
                f"[{self.v_id}] task {resp.task_id}: "
                f"pickup({resp.pickup_x},{resp.pickup_y}) "
                f"dropoff({resp.dropoff_x},{resp.dropoff_y})"
            )
            self.set_state(MOVING_TO_PICKUP)
        else:
            self.set_state(FINISHED)

    # ── خطوة واحدة ناحية target ──────────────────────
    def step_towards_target(self, tx, ty):
        # وصلنا؟
        if self.x == tx and self.y == ty:
            return True

        # حساب الخطوة — x الأول وبعدين y
        next_x, next_y = self.x, self.y
        if self.x != tx:
            next_x = self.x + (1 if tx > self.x else -1)
        else:
            next_y = self.y + (1 if ty > self.y else -1)

        # طلب إذن التحرك
        req = RequestMove.Request()
        req.vehicle_name = self.v_id
        req.target_x     = next_x
        req.target_y     = next_y
        future = self.m_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result().approved:
            # وافق — اتحرك
            self.x, self.y = next_x, next_y
            self.pub_pos()
            self.get_logger().info(
                f"[{self.v_id}] moved to ({self.x},{self.y})"
            )
            time.sleep(1.5)
        else:
            # رفض — استنى
            self.get_logger().info(
                f"[{self.v_id}] blocked at ({next_x},{next_y}) — waiting"
            )
            self.set_state(WAITING)
            time.sleep(1.0)
            self.set_state(self.prev_state)

        return False

    # ── logic loop ───────────────────────────────────
    def logic_loop(self):
        if self.state == IDLE:
            self.set_state(REQUEST_TASK)

        elif self.state == REQUEST_TASK:
            self.get_new_task()

        elif self.state == MOVING_TO_PICKUP:
            self.prev_state = MOVING_TO_PICKUP
            reached = self.step_towards_target(
                self.current_task["pickup_x"],
                self.current_task["pickup_y"]
            )
            if reached:
                self.get_logger().info(f"[{self.v_id}] arrived at pickup!")
                self.prev_state = MOVING_TO_DROPOFF
                self.set_state(MOVING_TO_DROPOFF)

        elif self.state == MOVING_TO_DROPOFF:
            self.prev_state = MOVING_TO_DROPOFF
            reached = self.step_towards_target(
                self.current_task["dropoff_x"],
                self.current_task["dropoff_y"]
            )
            if reached:
                self.get_logger().info(
                    f"[{self.v_id}] delivered task {self.current_task['id']}!"
                )
                self.set_state(REQUEST_TASK)

        elif self.state == WAITING:
            self.pub_state()

        elif self.state == FINISHED:
            self.get_logger().info(f"[{self.v_id}] all done!")

    # ── run ──────────────────────────────────────────
    def run(self):
        self.pub_pos()
        self.set_state(REQUEST_TASK)

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            if self.state == FINISHED:
                break

            self.logic_loop()

def main(args=None):
    rclpy.init(args=args)
    import sys
    name = sys.argv[1] if len(sys.argv) > 1 else "vehicle1"
    node = Vehicle(name)
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()