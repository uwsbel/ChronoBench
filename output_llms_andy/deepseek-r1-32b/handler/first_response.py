import pychrono aschrono
import rospy
from std_msgs.msg import Int32
fromchrono import ChSystemDFS, ChBody, ChBodyFixed, ChBoxShape, ChMaterialSurface, ChRosManager, ChRosSubscriber

def main():
    # Initialize ROS node
    rospy.init_node('chrono_ros_simulation', anonymous=True)
    
    # Create Chrono system
    system = ChSystemDFS()
    system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
    system.EnableCollisionDetection(True)
    
    # Create fixed floor
    floor = ChBodyFixed(system)
    floor.SetPos(chrono.ChVectorD(0, 0, 0))
    floor_shape = ChBoxShape()
    floor_shape.SetDimensions(chrono.ChVectorD(10, 1, 10))
    floor.AddShape(floor_shape)
    floor.SetMaterialSurface(ChMaterialSurface())
    
    # Create movable box
    box = ChBody(system)
    box.SetPos(chrono.ChVectorD(0, 1, 0))
    box_shape = ChBoxShape()
    box_shape.SetDimensions(chrono.ChVectorD(0.5, 0.5, 0.5))
    box.AddShape(box_shape)
    box.SetMass(1.0)
    box.SetInertiaFromMassAndDimensions(1.0, box_shape.GetDimensions())
    box.SetMaterialSurface(ChMaterialSurface())
    
    # Create ROS manager
    ros_manager = ChRosManager()
    ros_manager.Initialize(system, "chrono_simulation", 0, 0.001)
    ros_manager.RegisterHandler(ChRosClockHandler())
    ros_manager.RegisterHandler(ChRosBodyHandler())
    ros_manager.RegisterHandler(ChRosTransformHandler())
    
    # Custom ROS handler
    class CustomRosHandler(ChRosSubscriber):
        def __init__(self):
            super().__init__()
            self.counter = 0
            self.publisher = rospy.Publisher('chrono_topic', Int32, queue_size=10)
            
        def OnStart(self):
            super().OnStart()
            self.publisher = rospy.Publisher('chrono_topic', Int32, queue_size=10)
            
        def OnStep(self):
            super().OnStep()
            self.counter += 1
            msg = Int32()
            msg.data = self.counter
            self.publisher.publish(msg)
    
    custom_handler = CustomRosHandler()
    ros_manager.RegisterHandler(custom_handler)
    
    # Run simulation
    try:
        while not rospy.is_shutdown():
            system.DoStepFrame()
            ros_manager.Update()
            if ros_manager.GetRealTimeFactor() > 0:
                rospy.sleep(ros_manager.GetRealTimeFactor() * system.GetStep())
    except rospy.ROSInterruptException:
        pass
    
    # Cleanup
    ros_manager.UnregisterAllHandlers()
    del ros_manager
    del system

if __name__ == "__main__":
    main()