import pychrono as ch
import pychrono.ros as chros

from rclpy.publisher import Publisher
from std_msgs.msg import Int64


class MyCustomHandler(chros.ChROSHandler):
    

    def __init__(self, topic):
        
        super().__init__(25)  

        self.topic = topic
        self.publisher: Publisher = None
        self.ticker = 0  

    def Initialize(self, interface: chros.ChROSPythonInterface) -> bool:
        
        print(f"Creating publisher for topic '{self.topic}' ...")

        
        self.publisher = interface.GetNode().create_publisher(Int64, self.topic, 1)

        return True

    def Tick(self, time: float):
        
        print(f"Publishing message at time {time:.3f} ...")

        
        msg = Int64()
        msg.data = self.ticker

        
        self.publisher.publish(msg)

        
        self.ticker += 1


def main():
    
    
    
    system = ch.ChSystemNSC()

    
    system.SetGravitationalAcceleration(ch.ChVector3d(0, 0, -9.81))

    
    
    
    material = ch.ChContactMaterialNSC()
    material.SetFriction(0.5)
    material.SetRestitution(0.1)

    
    
    
    floor = ch.ChBodyEasyBox(
        10, 10, 1,      
        1000,           
        True,           
        True,           
        material        
    )
    floor.SetPos(ch.ChVector3d(0, 0, -1))  
    floor.SetFixed(True)                   
    floor.SetName("base_link")             
    system.Add(floor)

    
    
    
    box = ch.ChBodyEasyBox(
        1, 1, 1,        
        1000,           
        True,           
        True,           
        material        
    )
    box.SetPos(ch.ChVector3d(0, 0, 5))     
    box.SetName("box")                     
    system.Add(box)

    
    
    
    ros_manager = chros.ChROSPythonManager()

    
    ros_manager.RegisterHandler(chros.ChROSClockHandler())

    
    ros_manager.RegisterHandler(
        chros.ChROSBodyHandler(25, box, "~/box_state")
    )

    
    ros_manager.RegisterHandler(chros.ChROSTFHandler(25))

    
    custom_handler = MyCustomHandler("~/my_topic")
    ros_manager.RegisterPythonHandler(custom_handler)

    
    ros_manager.Initialize()

    
    
    
    time = 0.0
    time_step = 1e-3   
    time_end = 30.0    

    
    realtime_timer = ch.ChRealtimeStepTimer()

    while time < time_end:
        
        system.DoStepDynamics(time_step)
        time = system.GetChTime()

        
        if not ros_manager.Update(time, time_step):
            break

        
        realtime_timer.Spin(time_step)


if __name__ == "__main__":
    main()