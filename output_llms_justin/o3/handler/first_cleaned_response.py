import time
import math
import rospy                                   
from std_msgs.msg import Int32                 

import pychrono.core as chrono                 


try:
    import pychrono.ros as chros
except ImportError:
    chros = None
    rospy.logwarn("Chrono was not compiled with the ROS bridge. "
                  "The simulation will still run, but no built-in "
                  "handlers (/clock, /tf, /body) will be produced.")





class IntPublisher:
    
    def __init__(self, topic_name="/int_counter", queue_size=10):
        self._pub     = rospy.Publisher(topic_name, Int32, queue_size=queue_size)
        self._counter = 0

    def update(self):
        
        msg       = Int32()
        msg.data  = self._counter
        self._pub.publish(msg)
        self._counter += 1





def create_system():

    
    sys = chrono.ChSystemSMC()                         
    sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))      

    
    mat = chrono.ChMaterialSurfaceSMC()
    mat.SetYoungModulus(2.0e5)     
    mat.SetFriction(0.5)
    mat.SetRestitution(0.1)

    
    floor_size      = chrono.ChVectorD(4, 0.20, 4)     
    floor_density   = 1000                             

    floor = chrono.ChBodyEasyBox(floor_size.x,
                                 floor_size.y,
                                 floor_size.z,
                                 floor_density,        
                                 True,                 
                                 True,                 
                                 mat)                  
    floor.SetPos(chrono.ChVectorD(0, -floor_size.y / 2.0, 0))
    floor.SetBodyFixed(True)
    sys.Add(floor)

    
    box_size      = chrono.ChVectorD(0.5, 0.5, 0.5)
    box_density   = 700                                

    box = chrono.ChBodyEasyBox(box_size.x,
                               box_size.y,
                               box_size.z,
                               box_density,
                               True,
                               True,
                               mat)
    box.SetPos(chrono.ChVectorD(0, 1.0, 0))            
    sys.Add(box)

    return sys, floor, box





def create_ros_bridge(system, floor, box):
    
    if chros is None:
        return None

    bridge = chros.ChROSBridge(system)                 

    
    bridge.AddROSPublisher(chros.ChROSClockHandler())

    
    
    bridge.AddROSPublisher(chros.ChROSBodyHandler(box, "box"))

    
    bridge.AddROSPublisher(chros.ChROSTFHandler(floor, "floor"))

    
    bridge.AddROSPublisher(chros.ChROSTFHandler(box,  "box"))

    return bridge





def run_simulation():
    
    
    
    rospy.init_node("chrono_ros_sim", anonymous=True)

    
    
    
    system, floor, box = create_system()

    
    
    
    bridge      = create_ros_bridge(system, floor, box)
    int_handler = IntPublisher("/my_int_topic")

    
    
    
    step_size  = 1.0 / 1000.0                     
    realtime   = chrono.ChRealtimeStepTimer()     

    sim_time   = 0.0
    end_time   = 10.0                             

    rospy.loginfo("Starting Chrono – ROS simulation …")
    while (not rospy.is_shutdown()) and (sim_time < end_time):

        
        system.DoStepDynamics(step_size)
        sim_time += step_size

        
        if bridge is not None:
            bridge.Update()                       

        
        int_handler.update()

        
        realtime.Spin(step_size)

    rospy.loginfo("Simulation finished.")





if __name__ == "__main__":
    try:
        run_simulation()
    except rospy.ROSInterruptException:
        pass