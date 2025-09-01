import pychrono as chrono
import pychrono.ros as chros
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class CustomROSHandler(chros.ChROSHandler):
    def __init__(self, node: Node, topic_name: str):
        super().__init__()
        self.publisher = node.create_publisher(Int32, topic_name, 10)
        self.msg = Int32()

    def OnBeginStep(self, time: float):
        self.msg.data = int(time)
        self.publisher.publish(self.msg)

def main():
    # Initialize the Chrono system
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))
    mat = chrono.ChContactMaterialNSC()
    mat.SetFriction(0.5)
    mat.SetRestitution(0.01)
    system.GetContactManager().SetDefaultMaterial(mat)

    # Create a fixed floor
    floor = chrono.ChBodyEasyBox(10, 10, 1, 1000)
    floor.SetPos(chrono.ChVector3d(0, 0, -1))
    floor.SetFixed(True)
    floor.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
    system.Add(floor)

    # Create a movable box
    box = chrono.ChBodyEasyBox(1, 1, 1, 1000)
    box.SetPos(chrono.ChVector3d(0, 0, 2))
    box.SetRot(chrono.QUNIT)
    box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
    box.SetMass(1)
    box.SetInertiaXX(chrono.ChVector3d(1, 1, 1))
    system.Add(box)

    # Initialize ROS
    rclpy.init()
    node = Node("chrono_ros")
    handler = CustomROSHandler(node, "chrono_time")
    chros.ChROSManager.RegisterHandler(handler)
    chros.ChROSManager.RegisterClockHandler()
    chros.ChROSManager.RegisterBodyHandler(box)
    chros.ChROSManager.RegisterTransformHandler(box)
    chros.ChROSManager.Initialize(system)

    # Run the simulation
    time = 0
    step_size = 1e-3
    time_end = 10
    while rclpy.ok() and time < time_end:
        time = system.GetChTime()
        system.DoStepDynamics(step_size)
        chros.ChROSManager.Update(time)
        handler.OnBeginStep(time)
        chrono.ChCollisionSystemBullet.SyncBulletTime(system.GetCollisionSystem(), time)
        if not chros.ChROSManager.SpinOnce(1.0 / 60, node):
            break

if __name__ == "__main__":
    main()