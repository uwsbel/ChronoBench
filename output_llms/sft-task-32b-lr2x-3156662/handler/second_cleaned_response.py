import pychrono as ch
import pychrono.ros as chros

import rclpy.publisher
from std_msgs.msg import String


class MyCustomHandler(chros.ChROSHandler):
    

    def __init__(self, topic):
        super().__init__(1)  

        self.topic = topic
        self.publisher: rclpy.publisher.Publisher = None
        self.ticker = 0  
        self.message = "Hello, world! At time: "  

    def Initialize(self, interface: chros.ChROSPythonInterface) -> bool:
        
        print(f"Creating publisher for topic {self.topic} ...")
        
        self.publisher = interface.Get
print("error happened with only start ```python")