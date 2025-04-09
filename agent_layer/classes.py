import json
from std_msgs.msg import String
import math

STATUS_IDLE = 'idle'
STATUS_BUSY = 'busy'

class Task:
    @classmethod
    def from_json(cls, json_data):
        try:
            if isinstance(json_data, str):
                json_data = json.loads(json_data)
            task_id = json_data.get("task_id")
            loader_id = json_data.get("loader_id")
            unloader_id = json_data.get("unloader_id")
            # task_type = json_data.get("task_type")
            # priority = json_data.get("priority")
        except Exception as e:
            raise ValueError(f"Invalid JSON data {json_data} -> {e}")

        return cls(task_id, loader_id, unloader_id)

    def __init__(self, task_id, loader_id, unloader_id):# task_type, priority):
        self.task_id = task_id
        self.loader_id = loader_id
        self.unloader_id = unloader_id
        self.status = "pending"
        # self.task_type = task_type
        # self.priority = priority

    def to_json(self):
        return json.dumps({
            "task_id": self.task_id,
            "loader_id": self.loader_id,
            "unloader_id": self.unloader_id,
            "status": self.status,
            # "task_type": self.task_type,
            # "priority": self.priority
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
    def __init__(self, id, work_status, location):
        self.id = id
        self.work_status = work_status
        self.location = location
    
    def to_json(self):
        return json.dumps({
            "id": self.id,
            "work_status": self.work_status,
            "location": self.location
        })

class CBBA:
    """
    Consensus-Based Bundle Algorithm for multi-agent task allocation.
    """
    def __init__(self, courier_status:Status, pub_bids, loaders:list, unloaders:list):
        self.courier_status:Status = courier_status
        self.bids:dict = {} # dictionary of bids. Populated by listening to other agent bids and by own bids.
        self.available_tasks:list = [] # populated by listening to loaders that broadcast their tasks
        self.bundle:list = [] # list of tasks that the agent is "claiming"
        self.winnings_bids:dict = {} # dictionary of current winning bids for each task

        # Implementation specific:
        self.pub_bids = pub_bids # publisher for the winning bids
        self.loader_statuses = loaders # list of active loaders
        self.unloader_statuses = unloaders # list of active unloaders

    def task_callback(self, msg: String):
        """
        Callback for task updates from loaders.
        """
        task = Task.from_json(msg.data)
        # Check if the task is already in the available tasks list
        if task.task_id in [t.task_id for t in self.available_tasks]:
            return
        
        self.available_tasks.append(task)

    def compute_bid(self, task:Task) -> float:
        """
        Compute the bid for a given task. Get pickup and dropoff locations from the task.
        """
        # TODO add other factors to the bid calculation
        # like battery capacity, etc.

        # For now calculate the bid based on the distance to the loader
        loader_status:Status = next((l for l in self.loader_statuses if l.loader_id == task.loader_id), None)
        if loader_status is None:
            return 0.0 # return 0 if courier has no info about the loader
        loader_x, loader_y = loader_status.location
        c_x, c_y = self.courier_status.location
        dx = loader_x - c_x
        dy = loader_y - c_y
        dist = math.hypot(dx, dy)

        return 1/(dist+1e6) # bigger -> better

    def run_cbba_iteration(self):
        """
        Run iteration of the CBBA algorithm.
        Parse available tasks and create a bundle.
        """
        # Calculate the bid for each available task
        # sort available tasks greedily by bid
        #self.bundle.clear()
        for task in sorted(self.available_tasks, key=lambda t: self.compute_bid(t), reverse=True):
            bid = self.compute_bid(task)
            self.bids[task.task_id] = bid
            # Check if the bid is better than the current winning bid
            # or if the task has no bid yet. Add it to the bundle if so.
            if task.task_id not in self.winnings_bids or bid > self.winnings_bids[task.task_id]:
                self.winnings_bids[task.task_id] = bid
                if task not in self.bundle:
                    self.bundle.append(task)

    def publish_winning_bids(self):
        """
        Publish the winning bids to the other agents.
        """
        bid_msg = {
            "courier_id": self.courier_status.id,
            "winning_bids": self.winnings_bids
        }

        msg = String(data=json.dumps(bid_msg))
        self.pub_bids.publish(msg)
        
    def bid_callback(self, msg:String):
        """
        Listen to bids from other agents and update the winning bids.
        """
        msg_data = json.loads(msg.data)
        # Skip if bid is from the same agent
        if msg_data['courier_id'] == self.courier_status.id:
            return
        
        heard_winning_bids:dict = msg_data['winning_bids']


        

        



# if __name__ == "__main__":
#     # Example usage
#     task = Task(1, "loader_1", "unloader_1")
#     print(task.to_json())
#     task_json = task.to_json()
#     new_task = Task.from_json(task_json)
#     print(new_task.to_json())