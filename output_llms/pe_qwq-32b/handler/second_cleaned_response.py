import pychrono as ch
import pychrono.ros as chros

from std_msgs.msg import String  

class MyCustomHandler(chros.ChROSHandler):
    
    def __init__(self, topic):
        super().__init__(1)  
        self.topic = topic
        self.publisher = None
        self.ticker = 0
        self.message = "Hello, world! At time: "  

    def Initialize(self, interface: chros.ChROSPythonInterface) -> bool:
        
        print(f"Creating publisher for topic {self.topic} ...")
        self.publisher = interface.GetNode().create_publisher(String, self.topic, 1)
        return True

    def Tick(self, time: float):
        
        print(f"Publishing {self.ticker} ...")
        msg = String()  
        msg.data = self.message + str(self.ticker)  
        self.publisher.publish(msg)
        self.ticker += 1

def main():
    sys = ch.ChSystemNSC()
    sys.SetGravitationalAcceleration(ch.ChVectorD(0, 0, -9.81))  
    
    
    phys_mat = ch.ChContactMaterialNSC()
    phys_mat.SetFriction(0.5)

    
    floor = ch.ChBodyEasyBox(10, 10, 1, 1000, True, True, phys_mat)
    floor.SetPos(ch.ChVectorD(0, 0, -1))
    floor.SetFixed(True)
    floor.SetName("base_link")
    sys.Add(floor)

    
    box = ch.ChBodyEasyBox(1, 1, 1, 1000, True, True, phys_mat)
    box.SetPos(ch.ChVectorD(0, 0, 5))
    box.SetRot(ch.Q_from_AngAxis(0.2, ch.ChVectorD(1, 0, 0)))  
    box.SetName("box")
    sys.Add(box)

    
    ros_manager = chros.ChROSPythonManager()
    
    
    ros_manager.RegisterHandler(chros.ChROSClockHandler())  
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(25, box, "~/box"))  
    tf_handler = chros.ChROSTFHandler(30)
    tf_handler.AddTransform(floor, floor.GetName(), box, box.GetName())
    ros_manager.RegisterHandler(tf_handler)  
    
    
    custom_handler = MyCustomHandler("~/my_topic")
    ros_manager.RegisterHandler(custom_handler)  
    
    
    ros_manager.Initialize(sys)

    
    time_step = 1e-3
    time_end = 30
    realtime_timer = ch.ChRealtimeStepTimer()

    while sys.GetChTime() < time_end:
        sys.DoStepDynamics(time_step)
        current_time = sys.GetChTime()
        
        
        if not ros_manager.Update(current_time, time_step):
            break
        
        realtime_timer.Spin(time_step)

if __name__ == "__main__":
    main()