#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from grid_fleet_interfaces.msg import VehiclePosition
from turtlesim.srv import Spawn
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
#mesh httslmmmmmm
# scale من grid (0-7)  turtlesim (1-10)
def grid_to_turtle(val):
    return 1.0 + (val / 7.0) * 9.0

TURTLE_NAMES = {
    "vehicle1": "turtle1",
    "vehicle2": "turtle2",
    "vehicle3": "turtle3",
}

class TurtlesimVisualizer(Node):
    def __init__(self):
        super().__init__('turtlesim_visualizer')

      
        self.turtle_poses = {}
        self.targets = {}

        # spawn turtle2 و turtle3
        self.spawn_cli = self.create_client(Spawn, '/spawn')
        self.spawn_cli.wait_for_service()
        self.spawn_turtles()

        # subscribe لمواقع العربيات
        self.create_subscription(
            VehiclePosition,
            '/vehicle_position',
            self.handle_vehicle_pos,
            10
        )


        self.create_subscription(
            Pose, '/turtle1/pose',
            lambda msg: self.save_pose('turtle1', msg), 10
        )
        self.create_subscription(
            Pose, '/turtle2/pose',
            lambda msg: self.save_pose('turtle2', msg), 10
        )
        self.create_subscription(
            Pose, '/turtle3/pose',
            lambda msg: self.save_pose('turtle3', msg), 10
        )

        # publisher للسرعة ة
        self.vel_pubs = {
            'turtle1': self.create_publisher(Twist, '/turtle1/cmd_vel', 10),
            'turtle2': self.create_publisher(Twist, '/turtle2/cmd_vel', 10),
            'turtle3': self.create_publisher(Twist, '/turtle3/cmd_vel', 10),
        }

        # timer 
        self.create_timer(0.1, self.move_turtles)

        self.get_logger().info("Turtlesim Visualizer ready!")
        self.get_logger().info("turtle1=vehicle1  turtle2=vehicle2  turtle3=vehicle3")

    def spawn_turtles(self):

        for name, tx, ty in [('turtle2', 10.0, 1.0), ('turtle3', 1.0, 10.0)]:
            req       = Spawn.Request()
            req.x     = tx
            req.y     = ty
            req.theta = 0.0
            req.name  = name
            future = self.spawn_cli.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            self.get_logger().info(f"Spawned {name} at ({tx},{ty})")

    def save_pose(self, turtle_name, msg):
        self.turtle_poses[turtle_name] = (msg.x, msg.y, msg.theta)

    def handle_vehicle_pos(self, msg):
        self.targets[msg.vehicle_name] = (msg.x, msg.y)

    def move_turtles(self):
        for vehicle_name, (gx, gy) in self.targets.items():
            turtle_name = TURTLE_NAMES.get(vehicle_name)
            if turtle_name is None:
                continue

            pose = self.turtle_poses.get(turtle_name)
            if pose is None:
                continue

            # target بالـ turtlesim scale
            tx = grid_to_turtle(gx)
            ty = grid_to_turtle(gy)

            cx, cy, ctheta = pose
            dx   = tx - cx
            dy   = ty - cy
            dist = math.sqrt(dx**2 + dy**2)

            twist = Twist()

            if dist > 0.1:
               
                angle_to_target = math.atan2(dy, dx)
                angle_diff      = angle_to_target - ctheta

                # normalize بين -pi و pi
                while angle_diff >  math.pi: angle_diff -= 2 * math.pi
                while angle_diff < -math.pi: angle_diff += 2 * math.pi

                twist.linear.x  = min(2.0, dist * 1.5)
                twist.angular.z = angle_diff * 4.0
            else:
               
                twist.linear.x  = 0.0
                twist.angular.z = 0.0

            self.vel_pubs[turtle_name].publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = TurtlesimVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
