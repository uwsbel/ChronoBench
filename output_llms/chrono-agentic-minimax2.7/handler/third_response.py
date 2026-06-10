"""
Handler demo — plain MBS with a custom ChROSHandler publishing String messages.
Features ROS string publisher, textures on bodies, and conditional rendering.

Plan type: core/mbs
Scene: floor + dynamic box with a custom Python ROS handler publishing
       "Hello, world! At time: <ticker>" as a std_msgs/String topic.
       Visual textures applied to floor and box; rendering controlled via
       render_steps (conditional scene updates every N physics steps).
"""

import os
import math

import pychrono.core as chrono
import pychrono.ros as chros
import pychrono.irrlicht as chronoirr

import os
import math

import pychrono.core as chrono
import pychrono.ros as chros
import pychrono.irrlicht as chronoirr


# === Named constants ===
time_step = 1e-3
sim_end = 10.0
render_step_size = time_step  # step size for rendering cadence
render_steps = 50              # number of physics steps between scene renders
publish_rate = 10             # ROS handler publish rate in Hz

box_mass = 1.0
box_size = 0.5
floor_z = -0.5

REC = bool(os.environ.get("SIMBENCH_RECORD"))  # review-only — controls frame capture

# === System & gravity ===
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Bodies ===
# Floor — fixed rigid ground
floor = chrono.ChBodyEasyBox(10.0, 0.2, 10.0, 1000.0, True, True)
floor.SetFixed(True)
floor.SetPos(chrono.ChVector3d(0, floor_z, 0))
floor.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
sys.Add(floor)

# Dynamic box
box = chrono.ChBodyEasyBox(box_size, box_size, box_size, box_mass, True, True)
box.SetPos(chrono.ChVector3d(0, box_size / 2, 0))
box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/bluewhite.png"))
sys.Add(box)

# === ROS ===
ros_manager = chros.ChROSPythonManager()
ros_manager.RegisterHandler(chros.ChROSClockHandler())

# Custom handler — publishes String message at publish_rate Hz
class MyCustomHandler(chros.ChROSHandler):
    """Publishes a string message on a ROS topic at publish_rate Hz."""

    def __init__(self, topic, rate):
        super().__init__(rate)
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

ros_manager.RegisterPythonHandler(MyCustomHandler("~/my_topic", publish_rate))

# Body handler on the box
box_body_handler = chros.ChROSBodyHandler(30, box, "~/box_pose")
ros_manager.RegisterHandler(box_body_handler)

# TF handler: floor -> box
tf_handler = chros.ChROSTFHandler(30)
floor.SetName("base_link")
box.SetName("box")
tf_handler.AddTransform(floor, floor.GetName(), box, box.GetName())
ros_manager.RegisterHandler(tf_handler)

ros_manager.Initialize()

# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Handler Demo — ROS String publisher with textures")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(3, -3, 2), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()
vis.AddGrid(0.5, 0.5, 40, 40,
            chrono.ChCoordsysd(chrono.ChVector3d(0, floor_z, 0), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))

# === Main loop ===
step_number = 0
frame = 0
try:
    while vis.Run() and sys.GetChTime() < sim_end:
        # Conditional rendering: update scene every render_steps
        if step_number % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
        for _ in range(render_steps):
            sys.DoStepDynamics(render_step_size)
            if not ros_manager.Update(sys.GetChTime(), render_step_size):
                break
            step_number += 1
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:
    import traceback
    traceback.print_exc()
    raise
finally:
    pass  # review-only writers closed by sim_recording
