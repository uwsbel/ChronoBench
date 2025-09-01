import pychrono as chrono
import pychrono.ros as chros
import rospy
from std_msgs.msg import Int32

class CustomROSHandler(chros.ChROSHandler):
    def __init__(self, topic, rate):
        super().__init__(rate)
        self.publisher = rospy.Publisher(topic, Int32, queue_size=10)
        self.counter = 0

    def OnUpdate(self, time):
        msg = Int32()
        msg.data = self.counter
        self.publisher.publish(msg)
        self.counter += 1

def main():
    
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))

    
    floor_material = chrono.ChMaterialSurfaceNSC()
    floor_material.SetFriction(0.2)
    floor_material.SetRestitution(0.1)

    floor = chrono.ChBody()
    floor.SetMaterialSurface(floor_material)
    floor.SetPos(chrono.ChVectorD(0, 0, -0.05))  
    floor.SetBodyFixed(True)
    floor.GetCollisionModel().ClearModel()
    floor.GetCollisionModel().AddBox(floor_material, 5, 5, 0.05, chrono.ChVectorD(0,0,0))
    floor.GetCollisionModel().BuildModel()
    system.Add(floor)

    
    box_material = chrono.ChMaterialSurfaceNSC()
    box_material.SetFriction(0.3)
    box_material.SetRestitution(0.2)

    box = chrono.ChBody()
    box.SetMaterialSurface(box_material)
    box.SetPos(chrono.ChVectorD(0, 0, 1))
    box.SetMass(10)
    box.SetInertiaXX(chrono.ChVectorD(1,1,1))
    box.GetCollisionModel().ClearModel()
    box.GetCollisionModel().AddBox(box_material, 0.5, 0.5, 0.5, chrono.ChVectorD(0,0,0))
    box.GetCollisionModel().BuildModel()
    system.Add(box)

    
    ros_manager = chros.ChROSManager()
    ros_manager.RegisterHandler(chros.ChROSClockHandler())
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(system))
    ros_manager.RegisterHandler(chros.ChROSTransformHandler(system))
    custom_handler = CustomROSHandler("/custom_topic", 10)  
    ros_manager.RegisterHandler(custom_handler)

    
    step_timer = chrono.ChRealtimeStepTimer()
    step = 1.0 / 50  

    while not rospy.is_shutdown():
        step_timer.Spin(step)
        system.DoStepDynamics(step)
        ros_manager.Update()

if __name__ == "__main__":
    main()