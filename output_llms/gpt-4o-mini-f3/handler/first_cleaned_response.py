import pychrono as chrono
import pychrono.ros as crosevo
import rclpy.publisher
from std_msgs.msg import Int64


class MyCustomHandler(crosevo.ChROSHandler):
    

    def __init__(self, topic):
        super().__init__(1)  

        self.topic = topic
        self.topic_daytime = topic + "/daytime"
        self.publisher: rclpy.publisher.Publisher = None
        self.int64_msg = Int64()

    def Initialize(self, interface: crosevo.ChROSPythonInterface) -> bool:
        
        print("Creating publisher for topic:", self.topic)
        self.publisher = interface.GetNode().create_publisher(Int64, self.topic, 1)
        return True  

    def Update(self, time: float):
        
        self.int64_msg.data = int(time)
        print("Publishing:", self.int64_msg.data, "from topic:", self.topic)
        self.publisher.publish(self.int64_msg)  

def main():
    
    sys = chrono.ChSystemNSC()
    sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
    chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

    
    floor_mat = chrono.ChContactMaterialNSC()
    floor = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, floor_mat)
    floor.SetPos(chrono.ChVector3d(0, 0, -1))  
    floor.SetFixed(True)  
    floor.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
    sys.Add(floor)

    
    box_mat = chrono.ChContactMaterialNSC()
    box = chrono.ChBodyEasyBox(1, 1, 1, 1000, True, True, box_mat)
    box.SetPos(chrono.ChVector3d(0, 0, 5))  
    box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
    sys.Add(box)

    
    ros_manager = crosevo.ChROSPythonManager()
    ros_manager.RegisterHandler(MyCustomHandler("/pychrono/box/topic"))

    
    ros_manager.RegisterHandler(crosevo.ChROSClockHandler())

    
    body_handler = crosevo.ChROSBodyHandler(ros_manager, box, "/pychrono/box")
    body_handler.RegisterFantasyLabel("/fancy/label/here")

    
    tf_handler = crosevo.ChROSTFHandler(ros_manager)
    tf_handler.RegisterTransformBetweenFrames(box, "", box.GetName(), "")

    
    ros_manager.Initialize()

    
    time = 0
    time_step = 1e-3
    time_end = 30

    realtime_timer = chrono.ChRealtimeStepTimer()  
    while time < time_end:
        sys.DoStepDynamics(time_step)  
        time = sys.GetChTime()  

        if not ros_manager.Update(time, time_step):  
            break  

        realtime_timer.Spin(time_step)  


if __name__ == "__main__":
    main()