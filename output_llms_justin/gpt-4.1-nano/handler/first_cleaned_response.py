import chrono
import rospy
from std_msgs.msg import Int32
from pychrono.core import ChSystemNSC, ChMaterialSurface, ChVectorD
from pychrono.ros import ChIntegrationROSOgre, ChBodyPositionUpdate, ChBodyOrientationUpdate, ChTransformPublisher


rospy.init_node('pychrono_ros_integration', anonymous=True)


system = ChSystemNSC()
system.Set_G_acc(ChVectorD(0, -9.81, 0))


material = ChMaterialSurface()
material.SetFriction(0.5)
material.SetDampingF(0.01)
material.SetYoungModulus(2e7)
material.SetPoissonRatio(0.3)


floor_body = chrono.ChBody()
floor_body.SetBodyFixed(True)
floor_body.SetPos(ChVectorD(0, -1, 0))

floor_collision = floor_body.GetCollisionModel()
floor_collision.ClearModel()
floor_collision.AddBox(material, 5, 0.1, 5)
floor_collision.BuildModel()
system.Add(floor_body)


box_body = chrono.ChBody()
box_body.SetPos(ChVectorD(0, 0.5, 0))
box_body.SetMass(1.0)
box_body.SetInertiaXX(ChVectorD(0.1, 0.1, 0.1))

collision_model = box_body.GetCollisionModel()
collision_model.ClearModel()
collision_model.AddBox(material, 0.2, 0.2, 0.2)
collision_model.BuildModel()
box_body.GetCollisionModel().BuildModel()
system.Add(box_body)



class CustomROSHandler:
    def __init__(self, topic_name='/simulation_data'):
        self.publisher = rospy.Publisher(topic_name, Int32, queue_size=10)

    def publish_data(self, data):
        msg = Int32()
        msg.data = data
        self.publisher.publish(msg)


custom_handler = CustomROSHandler('/simulation_int')


ros_manager = ChIntegrationROSOgre()


ros_manager.AddBodyPositionUpdate(ChBodyPositionUpdate())
ros_manager.AddBodyOrientationUpdate(ChBodyOrientationUpdate())
ros_manager.AddTransformPublisher(ChTransformPublisher())






dt = 1e-3  
total_time = 10.0
steps = int(total_time / dt)
clock = rospy.Rate(1.0 / dt)

for step in range(steps):
    system.StepDynamics(dt)
    
    ros_manager.DoStepDynamics()
    
    custom_handler.publish_data(step)
    
    clock.sleep()