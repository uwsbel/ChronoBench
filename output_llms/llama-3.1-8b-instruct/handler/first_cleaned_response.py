import chrono as ch
from chrono_ros import ROSHandler, ROSManager


ch.InitChroNode()


sys = ch.ChSystem()


ch.SetMaterial(1, ch.MAT_DEFAULT, 1e6, 0.3, 0.01, 0.01)
ch.SetMaterial(2, ch.MAT_DEFAULT, 1e6, 0.3, 0.01, 0.01)


floor = ch.ChBody()
floor.SetPos(ch.Vec3(0, 0, 0))
floor.SetRot(ch.Vec3(0, 0, 0))
floor.SetMass(0)
floor.SetBodyFixed(True)
floor.SetMaterial(1)
sys.Add(floor)


box = ch.ChBody()
box.SetPos(ch.Vec3(0, 1, 0))
box.SetRot(ch.Vec3(0, 0, 0))
box.SetMass(1)
box.SetBodyFixed(False)
box.SetMaterial(2)
sys.Add(box)


class MyROSHandler(ROSHandler):
    def __init__(self):
        super().__init__()
        self.count = 0

    def on_timer(self):
        self.publish_message(self.count)
        self.count += 1


ros_manager = ROSManager()
ros_manager.set_clock_handler(ch.ChClockHandler())
ros_manager.set_body_handler(ch.ChBodyHandler())
ros_manager.set_transform_handler(ch.ChTransformHandler())
ros_manager.register_handler(MyROSHandler())


t = 0
dt = 0.001
while t < 10:
    sys.DoStep(dt)
    ros_manager.update()
    t += dt


sys.Destroy()
ros_manager.shutdown()