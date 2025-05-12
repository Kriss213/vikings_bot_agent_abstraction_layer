#!/usr/bin/env python3

"""
Loader nodes are responsible for publishing tasks to the task_broadcast topic.
"""

import rclpy
import rclpy.logging
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
import json
import time
import random
from agent_layer.classes import Task, Status
from visualization_msgs.msg import Marker
from vikings_bot_agent_abstraction_layer.srv import LoadUnloadRequest
from agent_layer.classes import STATUS_IDLE, STATUS_BUSY
random.seed(42)


class TaskLoader(Node):
    def __init__(self):
        super().__init__('loader_node')
        
        # ===== Parameters =====
        self.declare_parameter('loader_id', value='loader_1')
        self.declare_parameter('loc_x', value=0.0)
        self.declare_parameter('loc_y', value=0.0)
        self.declare_parameter('loc_yaw', value=0.0)
        self.declare_parameter('task_spawn_chance', value=0.3)
        self.declare_parameter('task_queue_size', value=10)
        self.declare_parameter('broadcast_interval', value=5.0)
        self.declare_parameter('loading_time', value=5.0)
        
        self.task_spawn_chance = self.get_parameter('task_spawn_chance').value
        self.task_queue_size = self.get_parameter('task_queue_size').value
        self.broadcast_interval = self.get_parameter('broadcast_interval').value
        self.loading_time = self.get_parameter('loading_time').value
        x, y, yaw = float(self.get_parameter('loc_x').value), float(self.get_parameter('loc_y').value), float(self.get_parameter('loc_yaw').value)
        
        # ===== Internal properties =====
        self.loader_id = self.get_parameter('loader_id').value
        self.location = (x, y, yaw)
        self.tasks:list[Task] = []
        self.status = Status(self.loader_id, STATUS_IDLE, self.location)
        self.available_unloaders = []
        
        # ===== Publishers =====
        self.pub_task = self.create_publisher(String, '/task_broadcast', 10)
        self.pub_status = self.create_publisher(String, '/loader_status', 10)
        self.pub_marker = self.create_publisher(Marker, '/loader_marker', 10)

        # ===== Subscribers =====
        self.sub_unloaders = self.create_subscription(String, '/unloader_status', self.unloader_status_callback, 10)

        # ===== Services =====
        self.srv_cb_group = ReentrantCallbackGroup() # callback group for services
        self.srv_loading = self.create_service(srv_type=LoadUnloadRequest, srv_name=f'{self.loader_id}/load_request', callback=self.clb_load_request, callback_group=self.srv_cb_group)    

        # ===== Timers =====
        self.timer = self.create_timer(self.broadcast_interval, self.broadcast)
        
        # ==================

        self.get_logger().info(f'Loader ID: {self.loader_id} at location {x}, {y}, {yaw}')

    def broadcast(self):
        if random.random() < self.task_spawn_chance and len(self.tasks) < self.task_queue_size and self.available_unloaders:
            task = Task(
            task_id=random.randint(1, 1000),
                loader_id=self.loader_id,
                unloader_id=random.choice(self.available_unloaders), 
                priority=random.randint(1, 10),
            )
            self.tasks.append(task)
            task_json = task.to_json()
            self.pub_task.publish(String(data=task_json))
            self.get_logger().info(f'{self.loader_id} published task: {task_json}')

        # publish status
        self.pub_status.publish(String(data=self.status.to_json()))
        self.publish_marker()

    def unloader_status_callback(self, msg:String):
        """Callback for unloader status updates"""
        status = Status.from_json(msg.data)
        #self.get_logger().info(f'{self.loader_id} received status update from unloader {status.id}: {status.work_status}')
        if status.work_status == STATUS_IDLE:
            self.available_unloaders.append(status.id)
        elif status.work_status == STATUS_BUSY and status.id in self.available_unloaders:
            self.available_unloaders.remove(status.id)

    def clb_load_request(self, request, response):
        """Load package on UGV
        
        A non-blocking service that loads a package on the UGV.

        request:
            * str task_id
            * str ugv_status
        response:
            * bool success
            * str task_id
        """
        response.task_id = request.task_id
        response.reason = ''
        ugv_status = Status.from_json(request.ugv_status)
        self.get_logger().info(f'{self.loader_id} received load request for task {request.task_id} from UGV {ugv_status.id}...')

        #=============
        # check if loader is available
        if self.status.work_status == STATUS_BUSY:
            response.success = False
            response.reason = 'Loader is busy'
            return response

        # check if UGV is at loader (within 1m)
        dist_ugv_loader = ((self.location[0] - ugv_status.location[0])**2 + (self.location[1] - ugv_status.location[1])**2)**0.5
        self.get_logger().info(f"Distance between UGV and loader: {dist_ugv_loader}")
        self.get_logger().info(f"UGV location: {ugv_status.location}")
        self.get_logger().info(f"Loader location: {self.location}")
        if dist_ugv_loader > 1.0:
            response.success = False
            response.reason = 'UGV is too far from loader'
            return response

        # Check if the task is in queue
        task_id = request.task_id
        task = next((t for t in self.tasks if t.task_id == task_id), None)
        if task is None:
            response.success = False
            response.reason = "Task not found in loader queue"
            return response
        
        # Set status to busy
        self.status.work_status = STATUS_BUSY

        # Load task on UGV
        self.get_logger().info(f'{self.loader_id} is loading task {task_id} on UGV {ugv_status.id}...')

        # Simulate loading time
        time.sleep(self.loading_time)

        response.success = True

        # remove task from queue
        self.tasks.remove(task)

        # set status to idle
        self.status.work_status = STATUS_IDLE
        self.get_logger().info(f'{self.loader_id} finished loading task {task_id} on UGV {ugv_status.id}. Task amount in queue: {len(self.tasks)}')

        return response
        
    def publish_marker(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = self.status.id
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = float(self.location[0])
        marker.pose.position.y = float(self.location[1])
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.color.r = 1.0 # red
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.text = self.loader_id
        marker.lifetime.sec = int(self.broadcast_interval * 2)
        self.pub_marker.publish(marker)


def main(args=None):
    # rclpy.logging.set_logger_level('vikings_bot_agent_abstraction_layer', rclpy.logging.LoggingSeverity.DEBUG)
    rclpy.init(args=args)
    node = TaskLoader()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()


if __name__ == "__main__":
    main()