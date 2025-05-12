#!/usr/bin/env python3

"""
Unloader nodes are responsible for unloading couriers via service.
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from visualization_msgs.msg import Marker
import json
import random
import time
from agent_layer.classes import Status
from vikings_bot_agent_abstraction_layer.srv import LoadUnloadRequest
from agent_layer.classes import STATUS_IDLE, STATUS_BUSY

class TaskUnloader(Node):
    def __init__(self):
        super().__init__('unloader_node')

        # ===== Parameters =====
        self.declare_parameter('unloader_id', value='unloader_1')
        self.declare_parameter('loc_x', value=0.0)
        self.declare_parameter('loc_y', value=0.0)
        self.declare_parameter('loc_yaw', value=0.0)
        self.declare_parameter('broadcast_interval', value=5.0)
        self.declare_parameter('unloading_time', value=5.0)
        
        x, y, yaw = float(self.get_parameter('loc_x').value), float(self.get_parameter('loc_y').value), float(self.get_parameter('loc_yaw').value)
        self.broadcast_interval = self.get_parameter('broadcast_interval').value
        self.unloading_time = self.get_parameter('unloading_time').value

        # ===== Internal properties =====
        self.unloader_id = self.get_parameter('unloader_id').value
        self.location = (x, y, yaw)
        self.status = Status(self.unloader_id, STATUS_IDLE, self.location)

        # ===== Publishers =====
        self.pub_status = self.create_publisher(String, '/unloader_status', 10)
        self.pub_marker = self.create_publisher(Marker, '/loader_marker', 10)

        # ===== Subscribers =====
        
        # ===== Services =====
        self.srv_cb_group = ReentrantCallbackGroup() # callback group for services
        self.srv_loading = self.create_service(srv_type=LoadUnloadRequest, srv_name=f'{self.unloader_id}/unload_request', callback=self.clb_unload_request, callback_group=self.srv_cb_group)

        # ===== Timers =====
        self.timer = self.create_timer(self.broadcast_interval, self.broadcast)

        # ==================

        self.get_logger().info(f'Unloader ID: {self.unloader_id} at location {x}, {y}, {yaw}')

    def broadcast(self):
        # publish status
        self.pub_status.publish(String(data=self.status.to_json()))
        self.publish_marker()
    
    def clb_unload_request(self, request, response):
        """Unload package from UGV
        
        A non-blocking service that unloads a package from the UGV.

        request:
            * str task_id
            * str ugv_status
        response:
            * bool success
            * str task_id
        """
        response.task_id = request.task_id
        ugv_status = Status.from_json(request.ugv_status)
        self.get_logger().info(f'{self.unloader_id} received unload request for task {request.task_id} from UGV {ugv_status.id}...')

        # check if unloader is available
        if self.status.work_status == STATUS_BUSY:
            response.success = False
            response.reason = "Unloader is busy"
            return response
        
        # check if UGV is at unloader (within 1m)
        dist_ugv_unloader = ((self.location[0] - ugv_status.location[0])**2 + (self.location[1] - ugv_status.location[1])**2)**0.5
        if dist_ugv_unloader > 1.0:
            response.success = False
            response.reason = "UGV is not at unloader"
            return response
        

        # Set status to busy
        self.status.work_status = STATUS_BUSY

        # Unload task from UGV
        task_id = request.task_id
        self.get_logger().info(f'{self.unloader_id} is unloading task {task_id} on UGV {ugv_status.id}...')

        # Simulate loading time
        time.sleep(self.unloading_time)

        response.success = True

        # set status to idle
        self.status.work_status = STATUS_IDLE
        self.get_logger().info(f'{self.unloader_id} finished unloading task {task_id} on UGV {ugv_status.id}.')

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
        marker.color.r = 0.0
        marker.color.g = 1.0  # green
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.text = self.unloader_id
        marker.lifetime.sec = int(self.broadcast_interval * 2)
        self.pub_marker.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = TaskUnloader()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

if __name__== '__main__':
    main()