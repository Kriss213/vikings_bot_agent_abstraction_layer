#!/usr/bin/env python3

"""
Courier nodes are responsible for collecting task broadcasts, running CBBA,
and completing delivery via Nav2 SimpleCommander API calls.
"""

import threading
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import json
import time
import math
from agent_layer.classes import Task, Status
from vikings_bot_agent_abstraction_layer.srv import LoadUnloadRequest
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from agent_layer.classes import STATUS_IDLE, STATUS_BUSY

class CBBA:
    def __init__(self, ugv_id):
        self.ugv_id = ugv_id
        self.task_memory = {}  # task_id: Task
        self.bids = {}         # task_id: bid_value
        self.winners = {}      # task_id: ugv_id
        self.bundle = []       # list of task_ids assigned to this courier

class Courier(Node):
    cbba_cb_group = ReentrantCallbackGroup()
    pose_cb_group = ReentrantCallbackGroup()  # for AMCL pose updates
    timer_cb_group = ReentrantCallbackGroup()  # for timers
    srv_cb_group = ReentrantCallbackGroup()  # for service calls

    def __init__(self):
        super().__init__('courier_node')

        # ===== Parameters =====
        self.declare_parameter('ugv_id', value='ugv_1')
        self.declare_parameter('start_x', value=0.0)
        self.declare_parameter('start_y', value=0.0)

        self.ugv_id = self.get_parameter('ugv_id').value

        self.status = Status(self.ugv_id, STATUS_IDLE, [
            float(self.get_parameter('start_x').value),
            float(self.get_parameter('start_y').value)
        ])
        self.navigator = BasicNavigator()

        # ===== Internal State =====
        self.cbba = CBBA(self.ugv_id)
        self.loader_status = {}    # loader_id -> Status
        self.unloader_status = {}  # unloader_id -> Status
        self.current_task = None
        self.nav_state = None
        self.waiting_for_service_response = False

        # ===== Publishers =====
        self.pub_bid = self.create_publisher(String, '/bid_updates', 10)

        # ===== Subscribers =====
        self.sub_task = self.create_subscription(String, '/task_broadcast', self.task_callback, 10)
        self.sub_loader_status = self.create_subscription(String, '/loader_status', self.loader_status_callback, 10)
        self.sub_unloader_status = self.create_subscription(String, '/unloader_status', self.unloader_status_callback, 10)
        self.sub_bid = self.create_subscription(String, '/bid_updates', self.bid_callback, 10)
        self.sub_amcl_pose = self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.amcl_callback, 10, callback_group=self.pose_cb_group)

        # ===== Services Clients =====
        self.cli_loaders = {}
        self.cli_unloaders = {}

        # ===== Timers =====
        self.cbba_timer = self.create_timer(3.0, self.run_cbba, callback_group=self.cbba_cb_group)
        self.exec_timer = self.create_timer(1.0, self.task_trigger_loop, callback_group=self.cbba_cb_group)
        #self.log_timer = self.create_timer(1.0, self.log_status, callback_group=self.timer_cb_group)

        self.get_logger().info(f'Courier {self.ugv_id} initialized at {self.status.location}')

    def amcl_callback(self, msg:PoseWithCovarianceStamped):
        self.status.location = [msg.pose.pose.position.x, msg.pose.pose.position.y]

    def task_callback(self, msg):
        task = Task.from_json(msg.data)
        if task.task_id not in self.cbba.task_memory:
            self.cbba.task_memory[task.task_id] = task
            self.get_logger().info(f'Received new task {task.task_id}')

    def loader_status_callback(self, msg):
        status = Status.from_json(msg.data)
        self.loader_status[status.id] = status

    def unloader_status_callback(self, msg):
        status = Status.from_json(msg.data)
        self.unloader_status[status.id] = status

    def bid_callback(self, msg):
        data = json.loads(msg.data)
        if data['ugv_id'] == self.ugv_id:
            return
        for task_id, winner in zip(data['task_ids'], data['winners']):
            if task_id not in self.cbba.winners or data['bids'][task_id] > self.cbba.bids.get(task_id, -1):
                self.cbba.winners[task_id] = winner
                self.cbba.bids[task_id] = data['bids'][task_id]

    def run_cbba(self):
        # TODO add check if agent can perform task
        # like storage capacity, etc.
        if self.nav_state:
            return

        available_tasks = [t for t_id, t in self.cbba.task_memory.items() if self.cbba.winners.get(t_id) is None or self.cbba.winners[t_id] == self.ugv_id]
        self.cbba.bundle.clear()
        for task in sorted(available_tasks, key=lambda t: self.compute_bid(t), reverse=True):
            task_id = task.task_id
            self.cbba.bundle.append(task_id)
            self.cbba.bids[task_id] = self.compute_bid(task)
            self.cbba.winners[task_id] = self.ugv_id

        bid_msg = {
            'ugv_id': self.ugv_id,
            'task_ids': self.cbba.bundle,
            'bids': {tid:self.cbba.bids[tid] for tid in self.cbba.bundle},
            'winners': [self.ugv_id for _ in self.cbba.bundle]
        }
        self.pub_bid.publish(String(data=json.dumps(bid_msg)))
        self.get_logger().info(f'Published CBBA bids: {bid_msg}')

    def compute_bid(self, task):
        # TODO add other factors to the bid calculation
        # like battery capacity, etc.
        if task.loader_id not in self.loader_status:
            return 0.0
        loader_loc = self.loader_status[task.loader_id].location
        dx = loader_loc[0] - self.status.location[0]
        dy = loader_loc[1] - self.status.location[1]
        dist = math.hypot(dx, dy)
        return 100.0 - dist

    def task_trigger_loop(self):
        self.get_logger().info(f'Task trigger loop: {self.cbba.bundle, self.nav_state}')
        if self.waiting_for_service_response:
            return  # Don't do anything until the last service finishes
    
        if self.nav_state is None and self.cbba.bundle:
            self.current_task = self.cbba.bundle.pop(0)
            task = self.cbba.task_memory[self.current_task]
            if task.loader_id in self.loader_status:
                self.send_nav_goal(self.loader_status[task.loader_id].location, 'pickup')

        if self.nav_state:
            # getResult is only supposed to be called afer isTaskComplete returns True
            result = self.navigator.getResult() if self.navigator.isTaskComplete() else None
            if result == TaskResult.SUCCEEDED: # or self.navigator.isTaskComplete():
                task = self.cbba.task_memory[self.current_task]
            
                if self.nav_state == 'pickup':
                    response = self.call_service(task.loader_id, task.task_id, is_loading=True)
                    if response.success:
                        self.get_logger().info(f'Successfully loaded task {task.task_id}')
                        self.send_nav_goal(self.unloader_status[task.unloader_id].location, 'dropoff')
                    else:
                        self.get_logger().warn(f'Loading failed: retrying. Reason: {response.reason}')
                        self.send_nav_goal(self.loader_status[task.loader_id].location, 'pickup')
                        
                elif self.nav_state == 'dropoff':
                    response = self.call_service(task.unloader_id, task.task_id, is_loading=False)
                    if response.success:
                        self.get_logger().info(f'Delivered task {task.task_id} successfully.')
                        del self.cbba.task_memory[task.task_id]
                        del self.cbba.winners[task.task_id]
                        self.nav_state = None
                        self.current_task = None
                        
                    else:
                        self.get_logger().warn(f'Delivery failed: retrying. Reason: {response.reason}')
                        self.send_nav_goal(self.unloader_status[task.unloader_id].location, 'dropoff')
            elif result == TaskResult.FAILED:
                self.get_logger().warn(f'Navigation failed: retrying ')
                if self.nav_state == 'pickup':
                    self.send_nav_goal(self.loader_status[task.loader_id].location, 'pickup')
                elif self.nav_state == 'dropoff':
                    self.send_nav_goal(self.unloader_status[task.unloader_id].location, 'dropoff')
            else:
                self.get_logger().info(f'Navigation in progress: {self.nav_state} result={result}')

    def send_nav_goal(self, target, goal_type):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = float(target[0])
        goal.pose.position.y = float(target[1])
        goal.pose.orientation.w = 1.0
        self.navigator.goToPose(goal)
        self.nav_state = goal_type
        self.get_logger().info(f'Navigating to {goal_type} goal: {target}')

    def call_service(self, station_id, task_id, is_loading):
        self.waiting_for_service_response = True

        client_map = self.cli_loaders if is_loading else self.cli_unloaders
        service_name = f'/{station_id}/load_request' if is_loading else f'/{station_id}/unload_request'

        if station_id not in client_map:
            client_map[station_id] = self.create_client(LoadUnloadRequest, service_name, callback_group=self.srv_cb_group)
        client = client_map[station_id]

        if not client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn(f'Service {service_name} not available.')
            self.waiting_for_service_response = False
            return self._fake_response(task_id, False, 'No response')

        req = LoadUnloadRequest.Request()
        req.task_id = task_id
        req.ugv_status = self.status.to_json()
        self.service_future = client.call_async(req)
        while rclpy.ok() and not self.service_future.done():
            self.service_called = True
            time.sleep(0.1)
        self.waiting_for_service_response = False
            #rclpy.spin_once(self, timeout_sec=0.5)
        #rclpy.spin_until_future_complete(self, future)
        return self.service_future.result()

    def log_status(self):
        self.get_logger().info(f'Courier {self.ugv_id} status: {self.status.to_json()}')

    def _fake_response(self, task_id, success: bool, reason: str):
        response = LoadUnloadRequest.Response()
        response.success = success
        response.task_id = task_id
        response.reason = reason
        return response
def main(args=None):
    rclpy.init(args=args)
    node = Courier()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

if __name__ == '__main__':
    main()
