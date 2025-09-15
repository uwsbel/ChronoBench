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
    box.SetMass(1.0)
    box.SetInertiaXX(chrono.ChVectorD(0.0833333, 0.1666667, 0.0833333))
    box.SetPos(chrono.ChVectorD(0, 1, 0))
    box_shape = ChBoxShape()
    box_shape.SetDimensions(chrono.ChVectorD(0.5, 0.5, 0.5))
    box.AddShape(box_shape)
    box.SetMaterialSurface(ChMaterialSurface())
    
    # Create material properties
    material = ChMaterialSurface()
    material.SetFriction(0.5)
    material.SetRestitution(0.3)
    box.GetMaterialSurface().SetMaterial(material)
    
    # Create ROS manager
    ros_manager = ChRosManager()
    ros_manager.Initialize(system, "chrono_node", 0, 0.001)
    ros_manager.SetClockHandler()
    ros_manager.SetBodyHandler()
    ros_manager.SetTransformHandler()
    
    # Create custom ROS handler
    class CustomRosHandler(ChRosSubscriber):
        def __init__(self):
            super(CustomRosHandler, self).__init__()
            self.counter = 0
            self.publisher = rospy.Publisher('chrono_counter', Int32, queue_size=10)
            
        def OnStart(self):
            super(CustomRosHandler, self).OnStart()
            
        def callback(self, data):
            self.counter += 1
            msg = Int32()
            msg.data = self.counter
            self.publisher.publish(msg)
            
    custom_handler = CustomRosHandler()
    ros_manager.RegisterSubscriber(custom_handler)
    
    # Run simulation
    try:
        while not rospy.is_shutdown():
            system.DoStepFrame()
            ros_manager.Update()
            if ros_manager.GetRealTimeFactor() > 0:
                rospy.sleep(ros_manager.GetRealTimeFactor() * system.GetChTimeStep())
    except KeyboardInterrupt:
        pass
    
if __name__ == "__main__":
    main()