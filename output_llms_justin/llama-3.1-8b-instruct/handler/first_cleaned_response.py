import chrono as ch
from chrono_ros import ROSHandler, ROSManager


ch.InitChroNode()


sys = ch.ChSystem()


sys.SetGravity(0, 0, -9.81)


mat = ch.ChMaterialTool.SetFromNameDefault("StainlessSteel")


floor = ch.ChBodyEasyBox(10, 1, 0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0, mat)
floor.SetBodyFixed(True)
sys.Add(floor)


box = ch.ChBodyEasyBox(1, 1, 0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0, mat)
box.SetPos(0, 1, 0)
sys.Add(box)


class MyHandler(ROSHandler):
    def __init__(self):
        super().__init__()
        self.count = 0

    def publish(self):
        self.publish_int("my_topic", self.count)

    def update(self):
        self.count += 1


ros_manager = ROSManager(sys, 10)
ros_manager.AddHandler(MyHandler())


while True:
    sys.DoStepDynamics(0.01)
    ros_manager.Update()
    ros_manager.Poll()