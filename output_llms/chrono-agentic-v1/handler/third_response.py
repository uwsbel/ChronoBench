"""
Handler ROS demo — PyChrono 9.0.0 (Irrlicht, ChSystemNSC)

Models a simple floor + box multi-body scene connected to a ROS2 graph via
ChROSPythonManager.  A custom ChROSHandler subclass (MyCustomHandler) publishes
an incrementing integer to a ROS topic at 1 Hz; ChROSBodyHandler publishes the
box body pose; ChROSTFHandler broadcasts the transform from floor to box.

Visualization uses ChVisualSystemIrrlicht with explicit camera, typical lights,
and a sky box.  The floor and box each carry a texture loaded with SetTexture.
A publish_rate variable (10 Hz) drives the body/TF handler rates.  Rendering is
throttled through step_number / render_step_size / render_steps variables exactly
as the prompt specifies — the scene is redrawn only every render_steps physics
steps.

System type : ChSystemNSC
Gravity     : (0, 0, -9.81)  — Z-up convention
Expected    : box slides / bounces slightly on the floor; ROS topics publish;
              Irrlicht window shows the textured scene.
"""

# === Imports ===
import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.ros as chros
import rclpy.publisher
from std_msgs.msg import Int64


# === Simulation parameters ===
time_step       = 1e-3          # physics step [s]
sim_end         = 10.0          # simulation end time [s]
publish_rate    = 10            # ROS handler publish rate [Hz] for body/TF handlers
render_fps      = 50.0          # target render frame rate [fps]
render_step_size = time_step    # duration of one physics step (matches time_step)
# precomputed once — number of physics steps between rendered frames
render_steps    = max(1, math.ceil(1.0 / (render_fps * render_step_size)))

# === Contact material (NSC) ===
mat = chrono.ChContactMaterialNSC()
mat.SetFriction(0.8)
mat.SetRestitution(0.01)

# === System & gravity ===
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Bodies ===
# Floor — fixed, textured
FLOOR_SX, FLOOR_SY, FLOOR_SZ = 10.0, 10.0, 0.2
floor = chrono.ChBodyEasyBox(FLOOR_SX, FLOOR_SY, FLOOR_SZ, 1000.0, True, True, mat)
floor.SetFixed(True)
floor.SetName("base_link")
floor.SetPos(chrono.ChVector3d(0.0, 0.0, -FLOOR_SZ / 2.0))   # top face at z=0
floor.GetVisualShape(0).SetTexture(
    chrono.GetChronoDataFile("textures/concrete.jpg")
)
sys.Add(floor)

# Box — dynamic, textured, spawned above the floor
BOX_SX, BOX_SY, BOX_SZ = 0.4, 0.4, 0.4
BOX_DENSITY = 500.0
box = chrono.ChBodyEasyBox(BOX_SX, BOX_SY, BOX_SZ, BOX_DENSITY, True, True, mat)
box.SetName("box")
box.SetPos(chrono.ChVector3d(0.0, 0.0, BOX_SZ / 2.0 + 1.0))  # 1 m above floor top
box.GetVisualShape(0).SetTexture(
    chrono.GetChronoDataFile("textures/blue.png")
)
sys.Add(box)

# === Custom ROS handler — publishes an incrementing Int64 at 1 Hz ===
class MyCustomHandler(chros.ChROSHandler):
    """Publishes an incrementing integer on ~/custom_data at 1 Hz."""

    def __init__(self, topic: str):
        super().__init__(1)                  # 1 Hz publish rate
        self.topic = topic
        self.publisher: rclpy.publisher.Publisher = None
        self.ticker: int = 0

    def Initialize(self, interface: chros.ChROSPythonInterface) -> bool:
        """Create the publisher from the rclpy node inside the ROS interface."""
        self.publisher = interface.GetNode().create_publisher(
            Int64, self.topic, 1
        )
        return True                          # must return True on success

    def Tick(self, time: float) -> None:
        """Publish an incrementing counter every time the handler fires."""
        msg = Int64()
        msg.data = self.ticker
        self.publisher.publish(msg)
        self.ticker += 1

# === ROS manager setup ===
ros_manager = chros.ChROSPythonManager()

# 1. Clock handler first — publishes /clock for ROS time synchronisation
ros_manager.RegisterHandler(chros.ChROSClockHandler())

# 2. Body handler — publishes box pose/twist at publish_rate Hz
ros_manager.RegisterHandler(
    chros.ChROSBodyHandler(publish_rate, box, "~/output/box/state")
)

# 3. TF handler — broadcasts floor→box transform at publish_rate Hz
tf_handler = chros.ChROSTFHandler(publish_rate)
tf_handler.AddTransform(floor, floor.GetName(), box, box.GetName())
ros_manager.RegisterHandler(tf_handler)

# 4. Custom Python handler — incrementing counter at 1 Hz
ros_manager.RegisterPythonHandler(MyCustomHandler("~/output/custom_data"))

# Initialize ONCE, after all handlers are registered
ros_manager.Initialize()

# === Visualization — Irrlicht window ===
# Full Irrlicht block: Initialize() FIRST, then scene elements AFTER
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Handler ROS Demo — PyChrono 9.0.0")
vis.Initialize()                                                          # FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(
    chrono.ChVector3d(6, -6, 4),     # eye
    chrono.ChVector3d(0, 0, 0),      # target
)
vis.AddTypicalLights()
vis.AddGrid(
    1.0, 1.0, 20, 20,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    chrono.ChColor(0.4, 0.4, 0.4),
)

# === Review-only setup ===


# === Main loop ===
# Rendering is conditional: the scene is redrawn every render_steps physics steps.
# step_number tracks the current physics step within the render cadence.
step_number = 0    # tracks which physics step we are on within the render cycle
frame = 0          # consecutive frame counter for review video

try:
    while vis.Run() and sys.GetChTime() < sim_end:
        # Conditional rendering — draw only every render_steps physics steps
        if step_number % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        # Physics step

        sys.DoStepDynamics(time_step)

        # ROS update — after physics step so handlers read current state
        time_now = sys.GetChTime()
        if not ros_manager.Update(time_now, time_step):
            break

        step_number += 1

except (RuntimeError, ValueError) as exc:     # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass

# === Post-processing (review-only) ===
