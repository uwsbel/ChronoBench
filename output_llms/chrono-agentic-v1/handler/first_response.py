"""
PyChrono simulation demonstrating ROS2 integration with a custom ChROSHandler.

System type: ChSystemNSC (rigid-body NSC, Y-up gravity)
Bodies: fixed floor (plane, NSC contact material) + movable box (dynamic, drops under gravity)
ROS: ChROSPythonManager with:
    - ChROSClockHandler  -> /clock
    - ChROSBodyHandler   -> publishes box pose/twist
    - ChROSTFHandler     -> /tf frame (floor -> box)
    - CustomIntHandler   -> publishes an incrementing Int64 counter to ~/my_topic
Expected behavior: box falls from height, bounces off floor; ROS topics publish throughout.
"""

# === Imports ===
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.ros as chros
import rclpy.publisher
from std_msgs.msg import Int64


# === Custom ROS handler — publishes an incrementing integer counter ===

class CustomIntHandler(chros.ChROSHandler):
    """Publishes an incrementing Int64 on a configurable ROS topic at 1 Hz."""

    def __init__(self, topic: str):
        super().__init__(1)               # publish rate: 1 Hz
        self.topic = topic
        self.publisher: rclpy.publisher.Publisher = None
        self.counter: int = 0

    def Initialize(self, interface: chros.ChROSPythonInterface) -> bool:
        """Create the publisher using the rclpy node embedded in the ROS interface."""
        self.publisher = interface.GetNode().create_publisher(Int64, self.topic, 1)
        return True                       # MUST return True to activate the handler

    def Tick(self, time: float):
        """Called at 1 Hz; build and publish the Int64 message."""
        msg = Int64()
        msg.data = self.counter
        self.publisher.publish(msg)
        self.counter += 1


# === Simulation parameters ===

time_step: float = 1e-3          # physics time step [s]
sim_end: float = 10.0            # simulation end time [s]
render_fps: float = 50.0         # Irrlicht render rate [Hz]
render_every: int = max(1, round(1.0 / (render_fps * time_step)))  # # precomputed once

# Floor geometry
floor_sx: float = 10.0   # floor width  [m]
floor_sy: float = 0.2    # floor height [m]
floor_sz: float = 10.0   # floor depth  [m]

# Box geometry
box_sx: float = 0.5   # box width  [m]
box_sy: float = 0.5   # box height [m]
box_sz: float = 0.5   # box depth  [m]
box_density: float = 500.0   # box density [kg/m³]
box_start_y: float = 3.0     # initial box height (center) [m]


# === System & gravity ===

sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))  # Y-up gravity
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)   # required: bodies have collision shapes
sys.SetSolverType(chrono.ChSolver.Type_PSOR)
sys.GetSolver().AsIterative().SetMaxIterations(100)


# === Contact material ===

mat = chrono.ChContactMaterialNSC()
mat.SetFriction(0.6)
mat.SetRestitution(0.2)


# === Bodies ===

# Fixed floor — rests at y = -floor_sy/2 so its top surface is at y = 0
floor = chrono.ChBodyEasyBox(floor_sx, floor_sy, floor_sz, 1000.0, True, True, mat)
floor.SetPos(chrono.ChVector3d(0.0, -floor_sy / 2.0, 0.0))
floor.SetFixed(True)
floor.SetName("base_link")     # TF convention: fixed root frame
floor.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
sys.Add(floor)

# Movable box — starts above the floor
box = chrono.ChBodyEasyBox(box_sx, box_sy, box_sz, box_density, True, True, mat)
box.SetPos(chrono.ChVector3d(0.0, box_start_y, 0.0))
box.SetName("box")
box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
sys.Add(box)


# === ROS manager setup ===

ros_manager = chros.ChROSPythonManager()

# 1. Clock handler FIRST — synchronises the ROS graph to simulation time
ros_manager.RegisterHandler(chros.ChROSClockHandler())

# 2. Body handler — publishes pose/twist of the movable box
ros_manager.RegisterHandler(chros.ChROSBodyHandler(25, box, "~/output/box/state"))

# 3. TF handler — broadcasts floor -> box frame tree
tf_handler = chros.ChROSTFHandler(30)
tf_handler.AddTransform(floor, floor.GetName(), box, box.GetName())
ros_manager.RegisterHandler(tf_handler)

# 4. Custom Python handler — publishes an incrementing Int64 counter
ros_manager.RegisterPythonHandler(CustomIntHandler("~/my_topic"))

# Initialize ONCE after all registrations, before the loop
ros_manager.Initialize()


# === Visualization (Irrlicht) ===

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("PyChrono + ROS2 — Handler Demo")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)   # Y-up world
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(6, 4, 0), chrono.ChVector3d(0, 1, 0))
vis.AddTypicalLights()
vis.AddGrid(
    0.5, 0.5, 40, 40,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    chrono.ChColor(0.4, 0.4, 0.4),
)


# === Review-only setup ===


# === Main loop ===

frame = 0   # review-only — consecutive frame counter for video assembly
realtime_timer = chrono.ChRealtimeStepTimer()    # keeps wall-clock ≈ sim time

try:
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            sys.DoStepDynamics(time_step)
            time = sys.GetChTime()
            if not ros_manager.Update(time, time_step):
                break                         # ROS layer shut down — exit cleanly
            realtime_timer.Spin(time_step)
            if sys.GetChTime() >= sim_end:
                break

except (RuntimeError, ValueError) as exc:    # solver divergence or bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass  # flush/cleanup: any review-only writers are closed in the review-only block below
