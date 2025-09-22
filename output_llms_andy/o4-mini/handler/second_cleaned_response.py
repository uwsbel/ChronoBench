import pychrono as ch
import pychrono.ros as chros

import rclpy
from rclpy.publisher import Publisher
from std_msgs.msg import String


class MyCustomHandler(chros.ChROSHandler):
    
    def __init__(self, topic: str):
        super().__init__(1)            
        self.topic = topic
        self.publisher: Publisher = None
        self.ticker = 0
        self.message = "Hello, world! At time: "

    def Initialize(self, interface: chros.ChROSPythonInterface) -> bool:
        
        print(f"[MyCustomHandler] Creating publisher on topic '{self.topic}' …")
        self.publisher = interface.GetNode().create_publisher(
            String, self.topic, 1
        )
        return True

    def Tick(self, time: float):
        
        payload = self.message + str(self.ticker)
        print(f"[MyCustomHandler] Publishing: '{payload}'")
        msg = String()
        msg.data = payload
        self.publisher.publish(msg)
        self.ticker += 1


def main():
    
    
    
    rclpy.init()

    
    
    
    sys = ch.ChSystemNSC()
    sys.Set_G_acc(ch.ChVector3d(0, 0, -9.81))

    
    phys_mat = ch.ChContactMaterialNSC()
    phys_mat.SetFriction(0.5)

    
    
    
    floor = ch.ChBodyEasyBox(
        10, 10, 1,     
        1000,          
        True, True, phys_mat
    )
    floor.SetPos(ch.ChVector3d(0, 0, -1))
    floor.SetFixed(True)
    floor.SetName("base_link")
    sys.Add(floor)

    
    
    
    box = ch.ChBodyEasyBox(
        1, 1, 1,
        1000,
        True, True, phys_mat
    )
    box.SetPos(ch.ChVector3d(0, 0, 5))
    
    box.SetRot(ch.Q_from_AngAxis(0.2, ch.ChVector3d(1, 0, 0)))
    box.SetName("box")
    sys.Add(box)

    
    
    
    ros_manager = chros.ChROSPythonManager()

    
    ros_manager.RegisterHandler(chros.ChROSClockHandler())

    
    ros_manager.RegisterHandler(
        chros.ChROSBodyHandler(25, box, "~/box")
    )

    
    tfh = chros.ChROSTFHandler(30)
    tfh.AddTransform(floor, floor.GetName(), box, box.GetName())
    ros_manager.RegisterHandler(tfh)

    
    my_handler = MyCustomHandler("~/my_topic")
    ros_manager.RegisterPythonHandler(my_handler)

    
    ros_manager.Initialize()

    
    
    
    t_end = 30.0
    dt = 1e-3
    realtime_timer = ch.ChRealtimeStepTimer()

    while sys.GetChTime() < t_end:
        sys.DoStepDynamics(dt)
        current_time = sys.GetChTime()

        if not ros_manager.Update(current_time, dt):
            print("[main] ROS manager signaled a shutdown.")
            break

        realtime_timer.Spin(dt)

    
    
    
    ros_manager.Shutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main()