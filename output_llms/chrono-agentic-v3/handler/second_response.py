"""
Simulation: Custom ROS Handler Demo — String message variant.

Models a simple MBS scene (fixed floor + dynamic box) with a ChROSPythonManager.
A custom Python ChROSHandler subclass publishes a concatenated String message
("Hello, world! At time: " + str(ticker)) over ROS at 1 Hz, alongside a
ChROSBodyHandler (box pose) and ChROSTFHandler (box frame relative to base_link).

System type: ChSystemNSC (Y-up, standard gravity).
Expected behavior: box falls onto the floor; ROS topics /clock, ~/output,
~/output/body, /tf publish throughout the simulation.
"""

import os
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.ros as chros
import rclpy.publisher
from std_msgs.msg import String                             # changed from Int64 to String


# === Constants ===
TIME_STEP = 1e-3        # physics time step [s]
SIM_END   = 10.0        # simulation end time [s]
RENDER_FPS = 50.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # precomputed once

FLOOR_HX  = 5.0         # floor half-size X [m]
FLOOR_HY  = 0.1         # floor half-height [m]
FLOOR_HZ  = 5.0         # floor half-size Z [m]
BOX_SIZE  = 0.4         # box full side [m]
BOX_DENSITY = 1000.0    # box density [kg/m³]
BOX_INIT_Y  = 2.0       # box initial height [m]

# === Custom ROS Handler ===

class MyCustomHandler(chros.ChROSHandler):
    """Publishes a String message ('Hello, world! At time: ' + str(ticker)) at 1 Hz."""

    def __init__(self, topic: str):
        super().__init__(1)         # publish rate: 1 Hz
        self.topic = topic
        self.publisher: rclpy.publisher.Publisher = None
        self.ticker: int = 0
        self.message: str = "Hello, world! At time: "    # prefix message attribute

    def Initialize(self, interface: chros.ChROSPythonInterface) -> bool:
        self.publisher = interface.GetNode().create_publisher(String, self.topic, 1)
        return True                 # MUST return True on success

    def Tick(self, time: float) -> None:
        msg = String()
        msg.data = self.message + str(self.ticker)       # concatenated string message
        self.publisher.publish(msg)
        self.ticker += 1


# === System & gravity ===
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Bodies ===
# Contact material (NSC)
mat = chrono.ChContactMaterialNSC()
mat.SetFriction(0.8)
mat.SetRestitution(0.0)

# Fixed floor
floor = chrono.ChBodyEasyBox(FLOOR_HX * 2, FLOOR_HY * 2, FLOOR_HZ * 2,
                              1000.0, True, True, mat)
floor.SetFixed(True)
floor.SetPos(chrono.ChVector3d(0.0, -FLOOR_HY, 0.0))
floor.SetName("base_link")              # TF root frame name
floor.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
sys.Add(floor)

# Dynamic box that falls onto the floor
box = chrono.ChBodyEasyBox(BOX_SIZE, BOX_SIZE, BOX_SIZE,
                            BOX_DENSITY, True, True, mat)
box.SetPos(chrono.ChVector3d(0.0, BOX_INIT_Y, 0.0))
box.SetName("box")
box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/bluewhite.png"))
sys.Add(box)

# === ROS Manager ===
ros_manager = chros.ChROSPythonManager()

# 1. Clock handler first
ros_manager.RegisterHandler(chros.ChROSClockHandler())

# 2. Custom String-message handler
ros_manager.RegisterPythonHandler(MyCustomHandler("~/output"))

# 3. Body handler — publishes box pose/twist
ros_manager.RegisterHandler(chros.ChROSBodyHandler(25, box, "~/output/body"))

# 4. TF handler — box frame relative to base_link
tf_handler = chros.ChROSTFHandler(30)
tf_handler.AddTransform(floor, floor.GetName(), box, box.GetName())
ros_manager.RegisterHandler(tf_handler)

# 5. Initialize once, after all registrations, before the loop
ros_manager.Initialize()

# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Custom ROS Handler Demo — String message")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.Initialize()                        # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(2, 3, -4), chrono.ChVector3d(0, 0.5, 0))
vis.AddTypicalLights()
vis.AddGrid(0.5, 0.5, 20, 20,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))

# === Review-only setup ===

# === Main loop ===
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
            if sys.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:      # solver divergence / bad physics state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass                                        # CSV closed in review-only block below
