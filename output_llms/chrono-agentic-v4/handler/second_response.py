"""
Handler demo — plain MBS with a custom ChROSHandler publishing String messages.

Plan type: core/mbs
Scene: floor + dynamic box with a custom Python ROS handler publishing
       "Hello, world! At time: <ticker>" as a std_msgs/String topic.
"""

import os
import sys
import math

import pychrono.core as chrono
import pychrono.ros as chros
import pychrono.irrlicht as chronoirr


# === Named constants ===
time_step = 1e-3
sim_end = 10.0
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))

box_mass = 1.0
box_size = 0.5
floor_z = 0.0


# === System & gravity ===
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Contact material ===
mat = chrono.ChContactMaterialNSC()
mat.SetFriction(0.8)
mat.SetRestitution(0.0)

# === Bodies ===
# Floor — fixed rigid ground
floor = chrono.ChBodyEasyBox(4.0, 4.0, 1.0, 1000.0, True, True, mat)
floor.SetFixed(True)
floor.SetPos(chrono.ChVector3d(0.0, -0.5, 0.0))
floor.SetName("base_link")
sys.Add(floor)

# Dynamic box
box = chrono.ChBodyEasyBox(box_size, box_size, box_size, box_mass, True, True, mat)
box.SetPos(chrono.ChVector3d(0.0, 0.0, floor_z + box_size))
box.SetName("box")
sys.Add(box)

# === ROS ===
ros_manager = chros.ChROSPythonManager()
ros_manager.RegisterHandler(chros.ChROSClockHandler())

# Custom handler — publishes String message: "Hello, world! At time: <ticker>"
class MyCustomHandler(chros.ChROSHandler):
    """Publishes a string message on a ROS topic at 1 Hz."""

    def __init__(self, topic):
        super().__init__(1)
        self.topic = topic
        self.publisher = None
        self.ticker = 0
        self.message = "Hello, world! At time: "

    def Initialize(self, interface: chros.ChROSPythonInterface) -> bool:
        from std_msgs.msg import String
        self.publisher = interface.GetNode().create_publisher(String, self.topic, 1)
        return True

    def Tick(self, time: float):
        from std_msgs.msg import String
        msg = String()
        msg.data = self.message + str(self.ticker)
        self.publisher.publish(msg)
        self.ticker += 1

ros_manager.RegisterPythonHandler(MyCustomHandler("~/my_topic"))

# Body handler on the box
box_body_handler = chros.ChROSBodyHandler(30, box, "~/box_pose")
ros_manager.RegisterHandler(box_body_handler)

# TF handler: floor -> box
tf_handler = chros.ChROSTFHandler(30)
tf_handler.AddTransform(floor, floor.GetName(), box, box.GetName())
ros_manager.RegisterHandler(tf_handler)

ros_manager.Initialize()

# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Handler Demo — ROS String publisher")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(3, -3, 2), chrono.ChVector3d(0, 0, 0.5))
vis.AddTypicalLights()
vis.AddGrid(0.5, 0.5, 20, 20,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))

# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
frame = 0

while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    for _ in range(render_every):
        sim_time = sys.GetChTime()
        sys.DoStepDynamics(time_step)

        # ROS update: break if ROS layer shuts down
        if not ros_manager.Update(sim_time, time_step):
            break

        if sys.GetChTime() >= sim_end:
            break

        realtime_timer.Spin(time_step)

print("Simulation complete.")
