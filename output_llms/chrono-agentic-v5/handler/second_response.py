"""Custom Chrono::ROS handler demo (PyChrono 9.0.0, NSC, Irrlicht).

Models a minimal rigid multibody scene — a fixed floor ("base_link") and a
free-falling box ("box") under gravity — whose purpose is to drive a custom
Python ROS2 handler. The custom handler publishes a std_msgs/String message
of the form "Hello, world! At time: <tick>" on a ROS2 topic at 1 Hz, alongside
a clock handler, a body-pose handler for the box, and a TF transform between
the floor and the box. Expected behavior: the box falls and settles on the
floor while the handlers stream state onto the ROS2 graph.
"""

import os
import math

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.ros as chros

import rclpy.publisher
from std_msgs.msg import String


# === Custom ROS handler === publishes a String greeting with the tick count
class MyCustomHandler(chros.ChROSHandler):
    """Publishes a 'Hello, world! At time: <tick>' String message at 1 Hz."""

    def __init__(self, topic):
        super().__init__(1)  # publish rate in Hz
        self.topic = topic
        self.publisher: rclpy.publisher.Publisher = None
        self.message = "Hello, world! At time: "
        self.ticker = 0

    def Initialize(self, interface: chros.ChROSPythonInterface) -> bool:
        self.publisher = interface.GetNode().create_publisher(String, self.topic, 1)
        return True  # MUST return True or the handler is dropped

    def Tick(self, time: float):
        msg = String()
        msg.data = self.message + str(self.ticker)
        self.publisher.publish(msg)
        self.ticker += 1


# === Named constants === geometry / physics / timing
time_step = 1e-3
sim_end = 30.0
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))   # precomputed once
box_start_z = 5.0

# === System & gravity === NSC system, contact present so Bullet collision is required
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Bodies === fixed floor (base_link) + falling box, sharing one contact material
phys_mat = chrono.ChContactMaterialNSC()
phys_mat.SetFriction(0.5)

floor = chrono.ChBodyEasyBox(10, 10, 1, 1000, True, True, phys_mat)
floor.SetPos(chrono.ChVector3d(0, 0, -1))
floor.SetFixed(True)
floor.SetName("base_link")          # conventional TF root frame
sys.Add(floor)

box = chrono.ChBodyEasyBox(1, 1, 1, 1000, True, True, phys_mat)
box.SetPos(chrono.ChVector3d(0, 0, box_start_z))
box.SetRot(chrono.QuatFromAngleAxis(0.2, chrono.ChVector3d(1, 0, 0)))
box.SetName("box")
sys.Add(box)

# === ROS manager === clock + body + TF + the custom String handler
ros_manager = chros.ChROSPythonManager()
ros_manager.RegisterHandler(chros.ChROSClockHandler())
ros_manager.RegisterHandler(chros.ChROSBodyHandler(25, box, "~/box"))

tf_handler = chros.ChROSTFHandler(30)
tf_handler.AddTransform(floor, floor.GetName(), box, box.GetName())
ros_manager.RegisterHandler(tf_handler)

custom_handler = MyCustomHandler("~/my_topic")
ros_manager.RegisterPythonHandler(custom_handler)

ros_manager.Initialize()   # exactly once, after all handlers are registered

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Custom ROS Handler - String publisher")
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(8, 8, 4), chrono.ChVector3d(0, 0, 1))
vis.AddTypicalLights()
vis.AddGrid(0.5, 0.5, 40, 40,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, -0.5), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))

# === Main loop === step physics, publish ROS state, render in real time

os.makedirs("cam", exist_ok=True)   # guard against missing output dir

try:

    realtime_timer = chrono.ChRealtimeStepTimer()
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            sys.DoStepDynamics(time_step)
            t = sys.GetChTime()
            if not ros_manager.Update(t, time_step):
                break
            if t >= sim_end:
                break
        realtime_timer.Spin(time_step)
except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
