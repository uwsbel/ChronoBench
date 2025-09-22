import pychrono.core as chrono
import pychrono.irrlicht as irr
import rospy
from std_msgs.msg import Int32
import threading
import time


class CustomROSHandler:
    def __init__(self, topic_name="/chrono/updates"):
        self.publisher = rospy.Publisher(topic_name, Int32, queue_size=10)
        self.message_counter = 0

    def publish_message(self):
        msg = Int32()
        msg.data = self.message_counter
        self.publisher.publish(msg)
        self.message_counter += 1


def init_ros():
    rospy.init_node('pychrono_ros_node', anonymous=True)


def main():
    
    init_ros()

    
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
    system.SetSolverType(chrono.ChSolver.Type.PRECONDITIONED)
    system.SetMaxPenetrationRecoverySpeed(1.5)
    system.SetMaxBounceSpeed(2.0)
    system.SetSolverMaxIterations(50)
    system.SetTimestep(1e-3)

    
    material = chrono.ChMaterialSurfaceNSC()
    material.SetFriction(0.4)
    material.SetRestitution(0.1)

    
    floor = chrono.ChBodyEasyBox(10, 0.5, 10, 1000, True, True)
    floor.SetPosition(chrono.ChVectorD(0, -0.25, 0))
    floor.SetBodyFixed(True)
    floor.GetMaterialSurface().SetFriction(0.4)
    system.Add(floor)

    
    box = chrono.ChBodyEasyBox(1, 1, 1, 500, True, True)
    box.SetPosition(chrono.ChVectorD(0, 1, 0))
    box.GetMaterialSurface().SetFriction(0.4)
    system.Add(box)

    
    ros_handler = CustomROSHandler()

    
    def ros_spin():
        rospy.spin()

    ros_thread = threading.Thread(target=ros_spin)
    ros_thread.daemon = True
    ros_thread.start()

    
    timestep = system.GetTimestep()
    total_time = 10  
    steps = int(total_time / timestep)

    
    for step in range(steps):
        
        system.DoStepDynamics(timestep)

        
        ros_handler.publish_message()

        
        time.sleep(timestep)

if __name__ == "__main__":
    main()