"""
PyChrono simulation: floor + box MBS with a custom ChROSHandler.

System type : ChSystemNSC (Y-up gravity)
Bodies      : fixed floor, dynamic box
ROS         : ChROSPythonManager with ClockHandler, BodyHandler,
              TFHandler, and a custom Python handler that publishes an
              incrementing Int64 to ~/my_topic at 1 Hz.
Expected    : Box falls under gravity and lands on the floor; ROS graph
              receives /clock, body pose, /tf, and integer messages each
              simulation second.
"""

import os
import csv

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.ros as chros
import rclpy.publisher
from std_msgs.msg import Int64

# === Custom ROS handler ===

class MyCustomHandler(chros.ChROSHandler):
    """Publishes an incrementing Int64 to ~/my_topic at 1 Hz."""

    def __init__(self, topic: str):
        super().__init__(1)  # publish rate: 1 Hz
        self.topic = topic
        self.publisher: rclpy.publisher.Publisher = None  # set in Initialize
        self.ticker: int = 0

    def Initialize(self, interface: chros.ChROSPythonInterface) -> bool:
        """Create the ROS publisher on the node provided by the interface."""
        self.publisher = interface.GetNode().create_publisher(
            Int64, self.topic, 1
        )
        return True  # must return True or the handler is dropped

    def Tick(self, time: float):
        """Publish the current counter value, then increment."""
        msg = Int64()
        msg.data = self.ticker
        self.publisher.publish(msg)
        self.ticker += 1


# === Simulation parameters ===

TIME_STEP    = 1e-3    # physics timestep [s]
SIM_END      = 10.0    # total simulation duration [s]
RENDER_FPS   = 50.0    # render frames per second
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# Body geometry
FLOOR_SX = 10.0   # floor width  [m]
FLOOR_SY =  0.2   # floor thickness [m]
FLOOR_SZ = 10.0   # floor depth  [m]
FLOOR_Y  = -0.1   # floor centre Y (top surface at Y=0)

BOX_SX      = 0.5     # box width  [m]
BOX_SY      = 0.5     # box height [m]
BOX_SZ      = 0.5     # box depth  [m]
BOX_Y       = 2.0     # box initial centre Y (above floor surface)
BOX_DENSITY = 1000.0  # box density [kg/m³]

# === System & gravity ===

sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))  # Y-up
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
sys.SetSolverType(chrono.ChSolver.Type_PSOR)
sys.GetSolver().AsIterative().SetMaxIterations(100)

# === Contact material ===

mat = chrono.ChContactMaterialNSC()
mat.SetFriction(0.8)
mat.SetRestitution(0.2)

# === Bodies ===

# Fixed floor
floor = chrono.ChBodyEasyBox(FLOOR_SX, FLOOR_SY, FLOOR_SZ,
                              2000.0, True, True, mat)
floor.SetPos(chrono.ChVector3d(0.0, FLOOR_Y, 0.0))
floor.SetFixed(True)
floor.SetName("base_link")
floor.GetVisualShape(0).SetTexture(
    chrono.GetChronoDataFile("textures/concrete.jpg")
)
sys.Add(floor)

# Movable box
box = chrono.ChBodyEasyBox(BOX_SX, BOX_SY, BOX_SZ,
                            BOX_DENSITY, True, True, mat)
box.SetPos(chrono.ChVector3d(0.0, BOX_Y, 0.0))
box.SetName("box")
sys.Add(box)

# === ROS Manager ===

ros_manager = chros.ChROSPythonManager()

# 1. Clock handler first — synchronises ROS graph to sim time
ros_manager.RegisterHandler(chros.ChROSClockHandler())

# 2. Body handler — publishes box pose/twist
ros_manager.RegisterHandler(
    chros.ChROSBodyHandler(25, box, "~/box/state")
)

# 3. TF handler — publishes floor → box transform
tf_handler = chros.ChROSTFHandler(30)
tf_handler.AddTransform(floor, floor.GetName(), box, box.GetName())
ros_manager.RegisterHandler(tf_handler)

# 4. Custom Python handler — publishes incrementing Int64
ros_manager.RegisterPythonHandler(MyCustomHandler("~/my_topic"))

# Initialize AFTER all registrations, BEFORE the loop
ros_manager.Initialize()

# === Visualization ===

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Handler ROS Demo — floor + box + custom handler")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.Initialize()                                      # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(4, 3, 6), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()


# === Main loop ===

realtime_timer = chrono.ChRealtimeStepTimer()  # for real-time pacing

frame = 0
try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(RENDER_EVERY):
            sys.DoStepDynamics(TIME_STEP)
            time = sys.GetChTime()  # cache: current sim time
            if not ros_manager.Update(time, TIME_STEP):
                break
            if time >= SIM_END:
                break
        realtime_timer.Spin(TIME_STEP)
except (RuntimeError, ValueError) as exc:  # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
