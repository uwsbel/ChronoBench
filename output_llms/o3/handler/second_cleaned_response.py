import pychrono as ch
import pychrono.ros as chros

import rclpy                      
import rclpy.publisher
from std_msgs.msg import String   






class MyCustomHandler(chros.ChROSHandler):
    

    def __init__(self, topic: str):
        super().__init__(1)                 

        self.topic = topic
        self.publisher: rclpy.publisher.Publisher = None
        self.ticker = 0

        
        self.message = "Hello, world! At time: "

    
    
    
    def Initialize(self, interface: chros.ChROSPythonInterface) -> bool:
        print(f"[ROS] Creating publisher on topic  {self.topic}")
        self.publisher = interface.GetNode().create_publisher(String,
                                                              self.topic,
                                                              1)
        return True

    def Tick(self, time: float):
        
        payload = self.message + str(self.ticker)
        print(f"[ROS] Publishing  \"{payload}\"")
        msg = String()
        msg.data = payload
        self.publisher.publish(msg)
        self.ticker += 1






def main():

    
    
    
    sys = ch.ChSystemNSC()
    sys.Set_G_acc(ch.ChVectorD(0, 0, -9.81))              

    
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

    
    ros_manager.RegisterPythonHandler(MyCustomHandler("~/my_topic"))

    
    if not ros_manager.Initialize():
        print("ERROR: could not initialise ROS manager.")
        return

    
    
    
    time_step = 1e-3
    time_end  = 30.0

    realtime_timer = ch.ChRealtimeStepTimer()

    while sys.GetChTime() < time_end:

        sys.DoStepDynamics(time_step)                         
        if not ros_manager.Update(sys.GetChTime(), time_step):
            print("ROS manager requested shutdown.")
            break

        realtime_timer.Spin(time_step)                        




if __name__ == "__main__":
    main()