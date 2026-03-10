#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from grid_fleet_interfaces.msg import VehiclePosition
from grid_fleet_interfaces.srv import RequestMove

DEADLOCK_TIMEOUT = 4.0

class TrafficControllerNode(Node):
    def __init__(self):
        super().__init__('traffic_controller')

        # (x,y) -> vehicle_name
        self.occupied_cells    = {}
        # vehicle_name -> (x,y)
        self.vehicle_positions = {}
        # vehicle_name
        self.wait_start        = {}

        self.create_subscription(
            VehiclePosition,
            '/vehicle_position',
            self.handle_position,
            10
        )

        self.create_service(
            RequestMove,
            '/request_move',
            self.handle_move
        )

        self.get_logger().info("Traffic Controller is ready!")

    def handle_position(self, msg):
        name    = msg.vehicle_name
        new_pos = (msg.x, msg.y)
        old_pos = self.vehicle_positions.get(name)


        if old_pos and self.occupied_cells.get(old_pos) == name:
            del self.occupied_cells[old_pos]


        self.vehicle_positions[name] = new_pos
        self.occupied_cells[new_pos] = name


        if old_pos != new_pos:
            self.wait_start.pop(name, None)

    def handle_move(self, request, response):
        target   = (request.target_x, request.target_y)
        occupant = self.occupied_cells.get(target)


        if occupant is None or occupant == request.vehicle_name:
            self.wait_start.pop(request.vehicle_name, None)
            response.approved = True
            self.get_logger().info(
                f"{request.vehicle_name} -> {target} APPROVED"
            )
            return response


        now = time.time()
        if request.vehicle_name not in self.wait_start:
            self.wait_start[request.vehicle_name] = now

        waited = now - self.wait_start[request.vehicle_name]

        if waited >= DEADLOCK_TIMEOUT:
            self.get_logger().warn(
                f"DEADLOCK: {request.vehicle_name} waited {waited:.1f}s — approved!"
            )
            del self.wait_start[request.vehicle_name]
            response.approved = True
        else:
            response.approved = False
            self.get_logger().info(
                f"{request.vehicle_name} -> {target} REJECTED (waited {waited:.1f}s)"
            )

        return response

def main(args=None):
    rclpy.init(args=args)
    node = TrafficControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
