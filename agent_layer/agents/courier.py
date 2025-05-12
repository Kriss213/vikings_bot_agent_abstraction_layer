#!/usr/bin/env python3

"""
Courier nodes are responsible for collecting task broadcasts, running CBBA,
and completing delivery via Nav2 SimpleCommander API calls.
"""

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
from visualization_msgs.msg import Marker
from scipy.spatial.transform import Rotation as R

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

        self.declare_parameter('ugv_id', value='ugv_1')
        self.declare_parameter('start_x', value=0.0)
        self.declare_parameter('start_y', value=0.0)
        self.declare_parameter('start_yaw', value=0.0)

        self.ugv_id = self.get_parameter('ugv_id').value
        self.status = Status(self.ugv_id, STATUS_IDLE, [
            float(self.get_parameter('start_x').value),
            float(self.get_parameter('start_y').value),
            float(self.get_parameter('start_yaw').value)
        ])
        self.navigator = BasicNavigator()

        self.cbba = CBBA(self.ugv_id)
        self.loader_status = {}
        self.unloader_status = {}
        self.current_task = None
        self.nav_state = None
        self.waiting_for_service_response = False

        self.pub_bid = self.create_publisher(String, '/bid_updates', 10)
        self.pub_marker = self.create_publisher(Marker, '/courier_marker', 10)
        self.pub_task_status_update = self.create_publisher(String, '/task_status_update', 10)

        self.sub_task_update = self.create_subscription(String, '/task_status_update', self.task_status_update_callback, 10)
        self.sub_task = self.create_subscription(String, '/task_broadcast', self.task_callback, 10)
        self.sub_loader_status = self.create_subscription(String, '/loader_status', self.loader_status_callback, 10)
        self.sub_unloader_status = self.create_subscription(String, '/unloader_status', self.unloader_status_callback, 10)
        self.sub_bid = self.create_subscription(String, '/bid_updates', self.bid_callback, 10)
        self.sub_amcl_pose = self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.amcl_callback, 10, callback_group=self.pose_cb_group)

        self.cli_loaders = {}
        self.cli_unloaders = {}

        self.consensus_iterations = 0
        self.max_consensus_iterations = 5
        self.cbba_converged = False

        self.cbba_timer = self.create_timer(1.0, self.run_cbba, callback_group=self.cbba_cb_group)
        self.exec_timer = self.create_timer(1.0, self.task_trigger_loop, callback_group=self.cbba_cb_group)
        # self.status_times = self.create_timer(5.0, self.log_status, callback_group=self.timer_cb_group)

        self.get_logger().info(f'Courier {self.ugv_id} initialized at {self.status.location}')

    def log_status(self):
        # publish status
        self.get_logger().info(f'=====Courier {self.ugv_id} status:======')
        self.get_logger().info(f'\tLoc: {self.status.location}')
        self.get_logger().info(f'\tBundle: {self.cbba.bundle}')
        self.get_logger().info(f'\tTask memory: {self.cbba.task_memory.keys()}')
        self.get_logger().info(f'\tBids: {self.cbba.bids}')
        self.get_logger().info(f'\tWinners: {self.cbba.winners}')
        self.get_logger().info(f'\tLoader status: {[s.id for s in self.loader_status.values()]}')
        self.get_logger().info(f'\tUnloader status: {[s.id for s in self.unloader_status.values()]}')
        self.get_logger().info(f'\tCurrent task: {self.current_task}')
        self.get_logger().info(f'==========================')

    def task_status_update_callback(self, msg:String):
        """
        Recieves Task JSON to updates local task memory.
        """
        data = json.loads(msg.data)
        if data['ugv_id'] == self.ugv_id:
            return
        task = Task.from_json(data['task'])
        task_id = task.task_id


        if task_id in self.cbba.task_memory \
            and (task.status == Task.STATUS_COMPLETED or task.status == Task.STATUS_STARTED):
            del self.cbba.task_memory[task_id]
    
    def publish_task_update(self, task:Task):
        """
        Publish task update to other agents.
        """
        task_json = task.to_json()
        msg = {
            'ugv_id': self.ugv_id,
            'task': task_json
        }
        self.pub_task_status_update.publish(String(data=json.dumps(msg)))
        
    def amcl_callback(self, msg:PoseWithCovarianceStamped):
        q = msg.pose.pose.orientation
        q = [q.x, q.y, q.z, q.w]
        r = R.from_quat(q)
        roll, pitch, yaw = r.as_euler('xyz')
        self.status.location = [msg.pose.pose.position.x, msg.pose.pose.position.y, yaw]

    def task_callback(self, msg):
        task = Task.from_json(msg.data)
        if task.task_id not in self.cbba.task_memory:
            self.cbba.task_memory[task.task_id] = task
            self.get_logger().info(f'Received new task {task.task_id}')
            self.cbba_converged = False
            self.consensus_iterations = 0

    def loader_status_callback(self, msg):
        status = Status.from_json(msg.data)
        self.loader_status[status.id] = status

    def unloader_status_callback(self, msg):
        status = Status.from_json(msg.data)
        self.unloader_status[status.id] = status

    def bid_callback(self, msg):
        data = json.loads(msg.data)
        if data['ugv_id'] == self.ugv_id:# or self.current_task:
            return
        changed = False
        for task_id, winner in zip(data['task_ids'], data['winners']):
            their_bid = data['bids'][str(task_id)] # str cause some random bs
            
            cond1 = task_id not in list(self.cbba.winners.keys())
            cond2 = their_bid > self.cbba.bids.get(task_id, float('-inf')) 
            if cond1 or cond2:
                #self.get_logger().info(f'Winners: {self.cbba.winners}')
                self.get_logger().info(f"Condition 1: {cond1} Condition 2: {cond2}")
                self.get_logger().info(f"Their bid: {their_bid} My bid: {self.cbba.bids.get(task_id, float('-inf'))}")
                self.cbba.winners[task_id] = winner
                self.cbba.bids[task_id] = their_bid
                changed = True
        if changed:
            self.cbba_converged = False
            self.consensus_iterations = 0

    def run_cbba(self):
        if self.current_task or self.cbba_converged:
            #self.get_logger().info(f'CBBA is not running: task is in progress {self.current_task != None} CBBA has converged {self.cbba_converged}.')
            return

        #available_tasks:list[Task] = [t for t_id, t in self.cbba.task_memory.items() if self.cbba.winners.get(t_id) is None or self.cbba.winners[t_id] == self.ugv_id]
        available_tasks = []

        for task_id, task in self.cbba.task_memory.items():
            if task.status != Task.STATUS_PENDING:
                continue

            current_winner = self.cbba.winners.get(task_id)
            current_bid = self.cbba.bids.get(task_id, -1)

            if current_winner is None or current_winner == self.ugv_id:
                # no winner yet or courier is the winner
                available_tasks.append(task)
            elif current_winner != self.ugv_id and self.cbba.bids.get(task_id, -1) > current_bid:
                # my bid is higher
                available_tasks.append(task)
        
        if len(available_tasks) == 0:
            #self.get_logger().info('No available tasks to bid on.')
            return
        
        self.cbba.bundle.clear()
        for task in sorted(available_tasks, key=lambda t: self.compute_bid(t), reverse=True):
            task_id = task.task_id
            self.cbba.bundle.append(task_id)
            self.cbba.bids[task_id] = self.compute_bid(task)
            self.cbba.winners[task_id] = self.ugv_id

        bid_msg = {
            'ugv_id': self.ugv_id,
            'task_ids': self.cbba.bundle,
            'bids': {tid : self.cbba.bids[tid] for tid in self.cbba.bundle},
            'winners': [self.ugv_id for _ in self.cbba.bundle]
        }
        self.pub_bid.publish(String(data=json.dumps(bid_msg)))
        #self.get_logger().info(f'Published CBBA bids: {bid_msg}')

        self.consensus_iterations += 1
        self.get_logger().info(f"{self.consensus_iterations}/{self.max_consensus_iterations} iterations. Bundle: {self.cbba.bundle}")
        if self.consensus_iterations >= self.max_consensus_iterations:
            self.cbba_converged = True
            self.get_logger().info(f'CBBA performed {self.consensus_iterations} iterations. Executing tasks.')

    def compute_bid(self, task:Task):
        if task.loader_id not in self.loader_status:
            return -float('inf')

        # start bid
        bid = 100.0

        # distance to loader
        loader = self.loader_status[task.loader_id]
        loader_loc = loader.location
        dx = loader_loc[0] - self.status.location[0]
        dy = loader_loc[1] - self.status.location[1]
        dist = round(math.hypot(dx, dy),1)

        # distance to unloader
        if task.unloader_id not in self.unloader_status:
            return -float('inf')
        unloader = self.unloader_status[task.unloader_id]
        unloader_loc = unloader.location
        dx = unloader_loc[0] - loader_loc[0]
        dy = unloader_loc[1] - loader_loc[1]
        dist += round(math.hypot(dx, dy),1) # round to reduce amcl noise impact

        # remove distance
        bid -= dist

        # battery charge TODO

        # check if the loader and unloader are available
        if loader.work_status == STATUS_BUSY or unloader.work_status == STATUS_BUSY:
            bid -= 10.0

        # bid higher for every second task is waiting
        task_age = task.get_age() #seconds

        # to avoid infinite bidding loops, increase bid only if >10 seconds have passed in task age
        bid += task_age//10 * task.priority


        return round(bid,4)

    def task_trigger_loop(self):
        self.publish_marker()

        if not self.cbba_converged and self.current_task is None:
            #self.get_logger().info(f'Convergence: {self.cbba_converged} Task: {self.current_task}')
            return

        if self.waiting_for_service_response:
            #self.get_logger().info(f'Waiting for service response...')
            return

        if self.nav_state is None and self.cbba.bundle:
            self.current_task = self.cbba.bundle.pop(0)
            
            # let others know task is claimed
            self.cbba.task_memory[self.current_task].status = Task.STATUS_STARTED
            self.publish_task_update(self.cbba.task_memory[self.current_task])

            self.get_logger().info(f'Executing task {self.current_task}')

            task = self.cbba.task_memory[self.current_task]
            if task.loader_id in self.loader_status:
                self.send_nav_goal(self.loader_status[task.loader_id].location, 'pickup')

        if self.nav_state:
            result = self.navigator.getResult() if self.navigator.isTaskComplete() else None
            if result == TaskResult.SUCCEEDED:
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
                task = self.cbba.task_memory[self.current_task]
                self.get_logger().warn(f'Navigation failed: retrying')
                if self.nav_state == 'pickup':
                    self.send_nav_goal(self.loader_status[task.loader_id].location, 'pickup')
                elif self.nav_state == 'dropoff':
                    self.send_nav_goal(self.unloader_status[task.unloader_id].location, 'dropoff')

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
            time.sleep(0.1)
        self.waiting_for_service_response = False
        return self.service_future.result()

    def _fake_response(self, task_id, success: bool, reason: str):
        response = LoadUnloadRequest.Response()
        response.success = success
        response.task_id = task_id
        response.reason = reason
        return response

    def publish_marker(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = self.status.id
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = float(self.status.location[0])
        marker.pose.position.y = float(self.status.location[1])
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0 #blue 
        marker.color.a = 1.0
        marker.text = self.ugv_id
        marker.lifetime.sec = int(2.0)
        self.pub_marker.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = Courier()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

if __name__ == '__main__':
    main()
