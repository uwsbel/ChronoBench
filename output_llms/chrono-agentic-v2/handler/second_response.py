"""
Chrono ROS handler demo — custom ChROSHandler subclass with String message type.

System type: ChSystemNSC (Y-up, gravity (0, -9.81, 0))
Bodies: a fixed floor and a dynamic box resting on it.
ROS layer: ChROSPythonManager with a clock handler, a body handler for the box,
           a TF handler linking floor→box, and a custom MyCustomHandler that
           publishes 'Hello, world! At time: <ticker>' as a std_msgs/String at 1 Hz.
Expected behavior: box rests on the floor; ROS topics /clock, ~/output/box_state,
           /tf, and ~/my_topic are published throughout the simulation.
"""

import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.ros as chros

import rclpy.publisher
from std_msgs.msg import String                    # Turn 2: changed from Int64 to String

# === Simulation constants ===
TIME_STEP = 1e-3
SIM_END   = 10.0
RENDER_FPS  = 50.0

# Floor geometry
FLOOR_HX = 3.0   # half-size X
FLOOR_HY = 0.05  # half-height Y
FLOOR_HZ = 3.0   # half-size Z

# Box geometry
BOX_SX  = 0.4
BOX_SY  = 0.4
BOX_SZ  = 0.4
BOX_DENSITY = 1000.0

# Spawn position of box centre (resting on top of floor)
BOX_INIT_Y = FLOOR_HY + BOX_SY / 2.0


# === Custom ROS handler (String message, 1 Hz) ===

class MyCustomHandler(chros.ChROSHandler):
    """Publishes an incrementing string message on a ROS topic at 1 Hz."""

    def __init__(self, topic: str):
        super().__init__(1)                                  # 1 Hz publish rate
        self.topic = topic
        self.publisher: rclpy.publisher.Publisher = None
        self.ticker: int = 0
        self.message: str = "Hello, world! At time: "       # Turn 2: added attribute

    def Initialize(self, interface: chros.ChROSPythonInterface) -> bool:
        self.publisher = interface.GetNode().create_publisher(
            String, self.topic, 1
        )
        return True                                          # MUST return True

    def Tick(self, time: float):
        msg = String()
        msg.data = self.message + str(self.ticker)          # Turn 2: concatenated string
        self.publisher.publish(msg)
        self.ticker += 1


# === System & gravity ===
sys = chrono.ChSystemNSC()
sys.SetGravityY()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
sys.SetSolverType(chrono.ChSolver.Type_PSOR)
sys.GetSolver().AsIterative().SetMaxIterations(100)

# === Contact material ===
mat = chrono.ChContactMaterialNSC()
mat.SetFriction(0.8)
mat.SetRestitution(0.0)

# === Bodies ===
# Floor — fixed, provides a contact surface
floor = chrono.ChBodyEasyBox(
    FLOOR_HX * 2, FLOOR_HY * 2, FLOOR_HZ * 2,
    2000.0, True, True, mat
)
floor.SetFixed(True)
floor.SetPos(chrono.ChVector3d(0, 0, 0))
floor.SetName("base_link")
sys.Add(floor)

# Box — dynamic, rests on the floor
box = chrono.ChBodyEasyBox(
    BOX_SX, BOX_SY, BOX_SZ,
    BOX_DENSITY, True, True, mat
)
box.SetPos(chrono.ChVector3d(0, BOX_INIT_Y, 0))
box.SetName("box")
sys.Add(box)

# === ROS manager ===
ros_manager = chros.ChROSPythonManager()

# 1. Clock handler first
ros_manager.RegisterHandler(chros.ChROSClockHandler())

# 2. Body handler for the box
ros_manager.RegisterHandler(
    chros.ChROSBodyHandler(25, box, "~/output/box_state")
)

# 3. TF handler: floor → box
tf_handler = chros.ChROSTFHandler(30)
tf_handler.AddTransform(floor, floor.GetName(), box, box.GetName())
ros_manager.RegisterHandler(tf_handler)

# 4. Custom Python handler (String message)
ros_manager.RegisterPythonHandler(MyCustomHandler("~/my_topic"))

ros_manager.Initialize()                                    # exactly once, after all registration

# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Chrono ROS Handler Demo — String")
vis.Initialize()                                            # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(3, 3, 3), chrono.ChVector3d(0, 0, 0))  # AFTER Initialize
vis.AddTypicalLights()

# === Precomputed constants ===
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once


# === CSV logging setup ===

# === Main loop ===
frame = 0
try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            sys.DoStepDynamics(TIME_STEP)
            time = sys.GetChTime()
            if not ros_manager.Update(time, TIME_STEP):   # pump ROS after physics step
                break
            if sys.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:               # solver divergence / bad state
    import traceback; traceback.print_exc()
    raise
finally:
    pass
