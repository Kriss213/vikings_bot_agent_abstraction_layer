from typing import Dict, List, Tuple
import json
from std_msgs.msg import String
import math
from rclpy.clock import Clock
from builtin_interfaces.msg import Time as TimeMsg
from rclpy.time import Time


STATUS_IDLE = 'idle'
STATUS_BUSY = 'busy'

class Task:
    STATUS_COMPLETED = 2
    STATUS_STARTED = 1
    STATUS_PENDING = 0
    @classmethod
    def from_json(cls, json_data):
        try:
            if isinstance(json_data, str):
                json_data = json.loads(json_data)
            task_id = json_data.get("task_id")
            loader_id = json_data.get("loader_id")
            unloader_id = json_data.get("unloader_id")
            priority = json_data.get("priority")
            s, ns = json_data.get("spawn_time").get("sec"), json_data.get("spawn_time").get("nanosec")
            spawn_time = Time(nanoseconds=int(str(s)+str(ns)))
            #spawn_time.nanoseconds = int(str(json_data.get("spawn_time").get("sec"))+str(json_data.get("spawn_time").get("nanosec")))
        except Exception as e:
            raise ValueError(f"Invalid JSON data {json_data} -> {e}")

        return cls(task_id, loader_id, unloader_id, priority, spawn_time=spawn_time)

    def __init__(self, task_id, loader_id, unloader_id, priority:int, spawn_time=None):
        self.task_id = task_id
        self.loader_id = loader_id
        self.unloader_id = unloader_id
        self.status = self.STATUS_PENDING
        self.__spawn_time:Time = Clock().now() if spawn_time is None else spawn_time

        # self.task_type = task_type
        self.priority:int = priority
    
    def get_age(self) -> float:
        """
        Get the age of the task in seconds.
        """
        now = Clock().now()

        return (now - self.__spawn_time).nanoseconds / 1e9

    def to_json(self):
        spawn_time:TimeMsg = self.__spawn_time.to_msg()
        return json.dumps({
            "task_id": self.task_id,
            "loader_id": self.loader_id,
            "unloader_id": self.unloader_id,
            "priority": self.priority,
            "spawn_time": {"sec":spawn_time.sec, "nanosec":spawn_time.nanosec},
            "status": self.status,
        })

class Status:
    @classmethod
    def from_json(cls, json_data):
        try:
            if isinstance(json_data, str):
                json_data = json.loads(json_data)
            id = json_data.get("id")
            work_status = json_data.get("work_status")
            location = json_data.get("location")
        except Exception as e:
            raise ValueError(f"Invalid JSON data {json_data} -> {e}")
        return cls(id, work_status, location)
    def __init__(self, id, work_status, location:Tuple[float,float,float]):
        self.id = id
        self.work_status = work_status
        self.location:Tuple[float,float,float] = location
    
    def to_json(self):
        return json.dumps({
            "id": self.id,
            "work_status": self.work_status,
            "location": self.location
        })

# class Task:
#     STATUS_PENDING = 'pending'
#     STATUS_STARTED = 'started'
#     STATUS_COMPLETED = 'completed'

#     def __init__(self, task_id: str, loader_id: str, unloader_id: str, priority: float = 1.0):
#         self.task_id = task_id
#         self.loader_id = loader_id
#         self.unloader_id = unloader_id
#         self.status = Task.STATUS_PENDING
#         self.priority = priority
#         self.spawn_time = Time(seconds=0)

#     def get_age(self, current_time: Time):
#         return (current_time.nanoseconds - self.spawn_time.nanoseconds) / 1e9

#     def to_dict(self):
#         return self.__dict__

#     @staticmethod
#     def from_dict(data):
#         task = Task(data['task_id'], data['loader_id'], data['unloader_id'], data['priority'])
#         task.status = data['status']
#         task.spawn_time = Time(nanoseconds=int(data['spawn_time']))
#         return task

# class Status:
#     def __init__(self, agent_id: str, work_status: str, location: Tuple[float, float, float]):
#         self.id = agent_id
#         self.work_status = work_status
#         self.location:Tuple[float, float, float] = location

class CBBA:
    """
    Consensus-Based Bundle Algorithm for multi-agent task allocation.
    """
    def __init__(self, agent_id: str):
        self.agent_id = agent_id
        self.task_memory: Dict[str, Task] = {}
        self.bids: Dict[str, float] = {}
        self.winners: Dict[str, str] = {}
        self.bundle: List[str] = []

    def update_tasks(self, task: Task, now: Time):
        if task.task_id not in self.task_memory:
            task.spawn_time = now
            self.task_memory[task.task_id] = task

    def update_position(self, position: List[float]):
        self.position = position

    def run_cbba(self, clock: Time) -> Dict:
        self.bundle.clear()
        available_tasks = [t for t in self.task_memory.values() if t.status == Task.STATUS_PENDING]

        for task in sorted(available_tasks, key=lambda t: self.compute_bid(t, clock), reverse=True):
            task_id = task.task_id
            bid = self.compute_bid(task, clock)
            if bid > self.bids.get(task_id, -math.inf):
                self.bundle.append(task_id)
                self.bids[task_id] = bid
                self.winners[task_id] = self.agent_id

        return {
            'agent_id': self.agent_id,
            'bids': self.bids,
            'winners': self.winners,
            'bundle': self.bundle
        }

    def compute_bid(self, task: Task, clock: Time):
        if task.status != Task.STATUS_PENDING:
            return -math.inf
        dx = self.position[0] - 0.0
        dy = self.position[1] - 0.0
        distance = math.hypot(dx, dy)
        age_sec = task.get_age(clock)
        boost = age_sec * task.priority
        return 100.0 - distance + boost

        

        



# if __name__ == "__main__":
#     # Example usage
#     task = Task(1, "loader_1", "unloader_1")
#     print(task.to_json())
#     task_json = task.to_json()
#     new_task = Task.from_json(task_json)
#     print(new_task.to_json())