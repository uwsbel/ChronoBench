import pychrono as chrono
import pychrono.ros as chronoros
import rospy
from std_msgs.msg import Int32

class CustomHandler(chronoros.ChROSHandler):
    def __init__(self, topic):
        super().__init__()
        self.counter = 0
        self.publisher = rospy.Publisher(topic, Int32, queue_size=10)
        
    def Synchronize(self, time):
        msg = Int32()
        msg.data = self.counter
        self.publisher.publish(msg)
        self.counter += 1

def main():
    
    rospy.init_node('pychrono_ros_demo', anonymous=True)
    
    
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  
    
    
    material = chrono.ChMaterialSurfaceNSC()
    material.SetFriction(0.5)
    
    
    floor = chrono.ChBodyEasyBox(20, 1, 20, 1000, True, True, material)
    floor.SetPos(chrono.ChVectorD(0, -5, 0))
    floor.SetBodyFixed(True)
    system.Add(floor)
    
    
    box = chrono.ChBodyEasyBox(1, 1, 1, 1000, True, True, material)
    box.SetPos(chrono.ChVectorD(0, 0, 0))
    system.Add(box)
    
    
    manager = chronoros.ChROSManager()
    manager.SetChSystem(system)
    
    
    manager.RegisterHandler(chronoros.ChROSClockHandler())  
    manager.RegisterHandler(chronoros.ChROSBodyHandler(manager))  
    manager.RegisterHandler(chronoros.ChROSTFHandler(manager))  
    
    
    custom_handler = CustomHandler("/pychrono/counter")
    manager.RegisterHandler(custom_handler)
    
    
    manager.Initialize()
    
    
    time_step = 0.01
    realtime_step = True
    
    while rospy.is_shutdown() is False:
        
        system.DoStepDynamics(time_step)
        
        
        manager.Update(system.GetChTime(), time_step)
        
        
        if realtime_step:
            chrono.ChRealtimeStepTimer.Spin(time_step)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass