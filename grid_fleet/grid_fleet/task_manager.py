#!/usr/bin/env python3
"""
Task Manager Node for Distributed Grid Fleet Coordination (Simulation Only)

Responsibilities:
- Generate at least 10 tasks
- Store tasks in a list
- Provide /request_task service
- Assign one task at a time, never assign same task twice
- Keep count of completed tasks (simulation-only)
"""

import rclpy
from rclpy.node import Node
import random
from grid_fleet_interfaces.srv import RequestTask  # Custom service

class TaskManagerNode(Node):
    def __init__(self):
        super().__init__('task_manager')
        self.get_logger().info("Task Manager Node Started")

        # Parameter: number of tasks (default 10)
        self.declare_parameter("num_tasks", 10)
        num_tasks = self.get_parameter("num_tasks").value

        # Generate tasks
        self.tasks = self.generate_tasks(num_tasks)

        # Count of tasks completed (simulation-only)
        self.completed_count = 0

        # Service to handle task requests
        self.task_service = self.create_service(
            RequestTask,
            '/request_task',
            self.handle_request_task
        )

        self.get_logger().info(f"{num_tasks} tasks generated")
        self.get_logger().info("Task Manager Ready! Waiting for vehicle requests...")

    def generate_tasks(self, num_tasks):
        """Generate random pickup and dropoff tasks"""
        task_list = []
        for i in range(num_tasks):
            pickup = (random.randint(0, 7), random.randint(0, 7))
            dropoff = (random.randint(0, 7), random.randint(0, 7))
            while dropoff == pickup:
                dropoff = (random.randint(0, 7), random.randint(0, 7))
            task_list.append({
                'id': i,
                'pickup': pickup,
                'dropoff': dropoff,
                'assigned': False
            })
        return task_list

    def handle_request_task(self, request, response):
        """
        Handles vehicle requests for a new task.
        Assigns the first available unassigned task.
        """
        vehicle_id = request.completed_task_id

        for task in self.tasks:
            if not task['assigned']:
                task['assigned'] = True
                self.completed_count += 1  # count as completed in simulation

                self.get_logger().info(
                    f"Vehicle {vehicle_id} assigned Task {task['id']} "
                    f"Pickup {task['pickup']} -> Dropoff {task['dropoff']} "
                    f"(Completed count: {self.completed_count}/{len(self.tasks)})"
                )

                response.has_task = True
                response.task_id = task['id']
                response.pickup_x = task['pickup'][0]
                response.pickup_y = task['pickup'][1]
                response.dropoff_x = task['dropoff'][0]
                response.dropoff_y = task['dropoff'][1]
                return response

        # No tasks left
        self.get_logger().info(f"Vehicle {vehicle_id} requested task but none available")
        response.has_task = False
        response.task_id = -1
        response.pickup_x = -1
        response.pickup_y = -1
        response.dropoff_x = -1
        response.dropoff_y = -1
        return response

def main(args=None):
    rclpy.init(args=args)
    node = TaskManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()