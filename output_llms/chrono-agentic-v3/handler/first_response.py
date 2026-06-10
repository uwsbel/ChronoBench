"""
PyChrono simulation: Floor + movable box with a custom ROS handler.

System type: ChSystemNSC (Y-up, gravity -9.81 along Y).
Bodies:
  - floor: fixed flat slab (collision via ChBodyEasyBox).
  - box: dynamic 1 kg box resting on the floor (collision via ChBodyEasyBox).
ROS:
  - ChROSPythonManager with ChROSClockHandler, ChROSBodyHandler (box),
    ChROSTFHandler (floor->box), and a custom MyIntPublisher that publishes
    an incrementing Int64 to ~/my_int_data at 1 Hz.
Expected behavior: the box falls from above, lands on the floor, and remains
at rest. ROS graph receives /clock, box pose/twist, /tf frames, and the custom
integer topic.
"""


import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.ros as chros
import rclpy.publisher
from std_msgs.msg import Int64

# === Named constants ===
TIME_STEP   = 1e-3           # physics step [s]
SIM_END     = 10.0           # simulation end time [s]
RENDER_FPS  = 50.0           # Irrlicht render cadence [fps]
GRAVITY_Y   = -9.81          # world gravity [m/s²]

# Floor dimensions (full extents) [m]
FLOOR_SX    = 10.0
FLOOR_SY    = 0.2
FLOOR_SZ    = 10.0
FLOOR_Y     = -0.1           # floor center Y so top surface is at Y=0

# Box dimensions and spawn position [m]
BOX_SX      = 0.5
BOX_SY      = 0.5
BOX_SZ      = 0.5
BOX_DENSITY = 1000.0         # density -> mass ≈ 125 kg
BOX_START_Y = 1.5            # drop from 1.5 m above floor top

# Precomputed render cadence (once, before the loop)
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once


# === Custom ROS handler — publishes an incrementing Int64 ===

class MyIntPublisher(chros.ChROSHandler):
    """Publishes an incrementing integer to ~/my_int_data at 1 Hz."""

    def __init__(self, topic: str):
        super().__init__(1)          # 1 Hz publish rate
        self._topic   = topic
        self._pub: rclpy.publisher.Publisher = None
        self._counter = 0

    def Initialize(self, interface: chros.ChROSPythonInterface) -> bool:
        self._pub = interface.GetNode().create_publisher(Int64, self._topic, 1)
        return True                  # MUST return True

    def Tick(self, time: float):
        msg       = Int64()
        msg.data  = self._counter
        self._pub.publish(msg)
        self._counter += 1


# === System & gravity ===

sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, GRAVITY_Y, 0))
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Contact material (NSC, shared by floor and box) ===

mat = chrono.ChContactMaterialNSC()
mat.SetFriction(0.8)
mat.SetRestitution(0.1)

# === Bodies ===

# Fixed floor
floor = chrono.ChBodyEasyBox(FLOOR_SX, FLOOR_SY, FLOOR_SZ,
                             2000.0, True, True, mat)
floor.SetPos(chrono.ChVector3d(0, FLOOR_Y, 0))
floor.SetFixed(True)
floor.SetName("base_link")
floor.GetVisualShape(0).SetTexture(
    chrono.GetChronoDataFile("textures/concrete.jpg"))
sys.Add(floor)

# Dynamic box (drops onto the floor)
box = chrono.ChBodyEasyBox(BOX_SX, BOX_SY, BOX_SZ,
                           BOX_DENSITY, True, True, mat)
box.SetPos(chrono.ChVector3d(0, BOX_START_Y, 0))
box.SetName("box")
sys.Add(box)

# === ROS manager ===

ros_manager = chros.ChROSPythonManager()

# 1. Clock handler first
ros_manager.RegisterHandler(chros.ChROSClockHandler())

# 2. Body handler — publishes box pose/twist
ros_manager.RegisterHandler(
    chros.ChROSBodyHandler(25, box, "~/box/state"))

# 3. TF handler — floor->box transform
tf_handler = chros.ChROSTFHandler(30)
tf_handler.AddTransform(floor, floor.GetName(), box, box.GetName())
ros_manager.RegisterHandler(tf_handler)

# 4. Custom Python handler — incrementing integer topic
ros_manager.RegisterPythonHandler(MyIntPublisher("~/my_int_data"))

# Initialize ONCE, after all registration, before the loop
ros_manager.Initialize()

# === Visualization ===

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Handler ROS Demo — Floor + Box")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(5, 3, 5), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()

# === Review-only setup ===


# === Main loop ===

try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(RENDER_EVERY):
            t = sys.GetChTime()
            sys.DoStepDynamics(TIME_STEP)
            if not ros_manager.Update(t, TIME_STEP):
                break
            if sys.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:                # solver divergence / bad input
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
