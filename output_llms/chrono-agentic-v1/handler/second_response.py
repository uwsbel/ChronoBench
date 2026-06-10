"""
PyChrono ROS Custom Handler Simulation — String Publisher Demo
==============================================================
System type : ChSystemNSC (Y-up, gravity = (0, -9.81, 0))
Bodies       : fixed floor, dynamic box sitting on the floor
Mechanism    : none — static contact scene
ROS          : ChROSPythonManager with:
                 - ChROSClockHandler (publishes /clock)
                 - MyCustomHandler   (custom Python subclass: publishes
                                      String "Hello, world! At time: N" at 1 Hz)
                 - ChROSBodyHandler  (publishes box pose on ~/output/box/state)
                 - ChROSTFHandler    (publishes TF frame: floor -> box)
Expected     : box rests on floor under gravity; ROS /clock, /tf, and the
               custom String topic publish throughout the 10-second run.
"""

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.ros as chros
import rclpy.publisher
from std_msgs.msg import String

# === Simulation parameters ===
TIME_STEP  = 1e-3          # physics step [s]
SIM_END    = 10.0          # total simulation time [s]
RENDER_FPS = 50.0          # Irrlicht render rate [frames/s]
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# Body geometry
FLOOR_W, FLOOR_D, FLOOR_H = 10.0, 10.0, 0.2   # floor slab full extents [m]
BOX_W,   BOX_D,   BOX_H   = 0.5, 0.5, 0.5     # box full extents [m]
BOX_DENSITY = 1000.0                            # kg/m³

# Floor center at Y=0; top surface at Y = FLOOR_H/2
FLOOR_CY = 0.0
BOX_CY   = FLOOR_H / 2 + BOX_H / 2            # box rests on floor surface


# === Custom ROS handler — publishes String ===

class MyCustomHandler(chros.ChROSHandler):
    """Publishes a String ROS message: 'Hello, world! At time: <tick>' at 1 Hz."""

    def __init__(self, topic: str):
        super().__init__(1)          # publish rate: 1 Hz
        self.topic = topic
        self.publisher: rclpy.publisher.Publisher = None
        self.ticker = 0
        self.message = "Hello, world! At time: "  # message prefix attribute

    def Initialize(self, interface: chros.ChROSPythonInterface) -> bool:  # noqa: N802
        """Create the ROS String publisher on the given topic. Must return True."""
        self.publisher = interface.GetNode().create_publisher(String, self.topic, 1)
        return True

    def Tick(self, time: float):  # noqa: N802
        """Publish the concatenated string message each tick."""
        msg = String()
        msg.data = self.message + str(self.ticker)
        self.publisher.publish(msg)
        self.ticker += 1


# === System & gravity ===
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))   # Y-up
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)    # contact present

# === Contact material ===
mat = chrono.ChContactMaterialNSC()
mat.SetFriction(0.8)
mat.SetRestitution(0.0)

# === Bodies ===

# Floor — fixed slab
floor = chrono.ChBodyEasyBox(FLOOR_W, FLOOR_H, FLOOR_D, 1000.0, True, True, mat)
floor.SetPos(chrono.ChVector3d(0, FLOOR_CY, 0))
floor.SetFixed(True)
floor.SetName("base_link")     # TF root frame name
floor.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
sys.Add(floor)

# Box — dynamic, rests on floor
box = chrono.ChBodyEasyBox(BOX_W, BOX_H, BOX_D, BOX_DENSITY, True, True, mat)
box.SetPos(chrono.ChVector3d(0, BOX_CY, 0))
box.SetName("box")             # TF child frame name
box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/bluewhite.png"))
sys.Add(box)

# === ROS manager ===
ros_manager = chros.ChROSPythonManager()

# 1. Clock handler FIRST
ros_manager.RegisterHandler(chros.ChROSClockHandler())

# 2. Custom String publisher handler
custom_handler = MyCustomHandler("~/output/custom/data")
ros_manager.RegisterPythonHandler(custom_handler)

# 3. Body handler — publishes box pose
ros_manager.RegisterHandler(chros.ChROSBodyHandler(25, box, "~/output/box/state"))

# 4. TF handler — floor -> box transforms
tf_handler = chros.ChROSTFHandler(30)
tf_handler.AddTransform(floor, floor.GetName(), box, box.GetName())
ros_manager.RegisterHandler(tf_handler)

# Initialize ONCE, after all handlers are registered
ros_manager.Initialize()

# === Visualization ===  (Irrlicht — Initialize FIRST, then scene elements AFTER)
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)    # Y-up world
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("ROS Custom Handler — String Publisher")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(3, 3, 3), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()
vis.AddGrid(
    0.5, 0.5, 20, 20,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    chrono.ChColor(0.4, 0.4, 0.4),
)

# === Main loop ===
frame = 0
try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(RENDER_EVERY):
            sys.DoStepDynamics(TIME_STEP)
            time = sys.GetChTime()
            if not ros_manager.Update(time, TIME_STEP):
                break
            if time >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:           # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
