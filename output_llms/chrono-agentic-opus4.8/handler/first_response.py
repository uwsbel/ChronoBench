"""ROS-integrated rigid-body simulation: a fixed floor + a movable falling box.

System type: NSC (non-smooth contact) with Bullet collision detection and
gravity along -Z. A fixed floor body acts as the TF root ("base_link"); a free
box body ("box") spawns above it, falls under gravity, and lands on the floor.

The simulation is bridged to ROS2 via pychrono.ros: a ChROSPythonManager owns a
clock handler (/clock), a body handler publishing the box pose/twist, a TF
handler broadcasting the floor->box transform, and a custom Python ChROSHandler
subclass that publishes an incrementing Int64 message on a configurable topic.
Each physics step advances the system, then pumps the ROS layer, while a
real-time timer keeps wall-clock pace with simulated time.
"""

import os
import math

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.ros as chros

import rclpy.publisher
from std_msgs.msg import Int64


# === Custom ROS handler === publishes an incrementing integer at a fixed rate
class IntegerPublisherHandler(chros.ChROSHandler):
    """Custom Chrono::ROS handler that publishes incrementing Int64 messages."""

    def __init__(self, topic, rate):
        super().__init__(rate)                  # publish rate in Hz
        self.topic = topic
        self.publisher: rclpy.publisher.Publisher = None
        self.ticker = 0

    def Initialize(self, interface: chros.ChROSPythonInterface) -> bool:
        # interface.GetNode() is the underlying rclpy node; build the publisher.
        self.publisher = interface.GetNode().create_publisher(Int64, self.topic, 1)
        return True                             # must return True or handler is dropped

    def Tick(self, time: float):
        msg = Int64()
        msg.data = self.ticker
        self.publisher.publish(msg)
        self.ticker += 1


# === Named constants === geometry / physics / ROS configuration
time_step = 1e-3
sim_end = 30.0
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))    # precomputed once

floor_size = (10.0, 10.0, 1.0)      # full extents [m]
floor_z = -1.0                      # floor top sits at z = -0.5
box_size = (1.0, 1.0, 1.0)          # full extents [m]
box_spawn_z = 5.0                   # box starts above the floor, falls under gravity
material_density = 1000.0           # [kg/m^3]
custom_topic = "~/my_topic"         # integer publisher topic
custom_rate = 1.0                   # [Hz]


# === System & gravity === NSC system with Bullet collision, gravity along -Z
sys = chrono.ChSystemNSC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)        # contact present
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

# === Contact material === shared NSC material for floor + box
phys_mat = chrono.ChContactMaterialNSC()
phys_mat.SetFriction(0.5)
phys_mat.SetRestitution(0.0)

# === Bodies === fixed floor (TF root) + a free falling box
floor = chrono.ChBodyEasyBox(floor_size[0], floor_size[1], floor_size[2],
                             material_density, True, True, phys_mat)    # visualize, collide
floor.SetPos(chrono.ChVector3d(0, 0, floor_z))
floor.SetFixed(True)
floor.SetName("base_link")          # conventional TF root frame
sys.Add(floor)

box = chrono.ChBodyEasyBox(box_size[0], box_size[1], box_size[2],
                           material_density, True, True, phys_mat)      # visualize, collide
box.SetPos(chrono.ChVector3d(0, 0, box_spawn_z))
box.SetRot(chrono.QuatFromAngleAxis(0.2, chrono.ChVector3d(1, 0, 0)))
box.SetName("box")                  # TF child frame / body-handler topic name
sys.Add(box)

# === ROS manager === clock + body + TF handlers, plus the custom integer handler
ros_manager = chros.ChROSPythonManager()
ros_manager.RegisterHandler(chros.ChROSClockHandler())                 # /clock first
ros_manager.RegisterHandler(chros.ChROSBodyHandler(25, box, "~/box"))  # box pose/twist

tf_handler = chros.ChROSTFHandler(30)
tf_handler.AddTransform(floor, floor.GetName(), box, box.GetName())
ros_manager.RegisterHandler(tf_handler)

custom_handler = IntegerPublisherHandler(custom_topic, custom_rate)
ros_manager.RegisterPythonHandler(custom_handler)                      # Python subclass

ros_manager.Initialize()            # exactly once, after all registration

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("ROS handler demo: floor + falling box")
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.Initialize()                    # Initialize FIRST (Irrlicht order is inverse of VSG)
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, -14, 6), chrono.ChVector3d(0, 0, 1))
vis.AddTypicalLights()
vis.AddGrid(0.5, 0.5, 40, 40,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, floor_z + 0.5 * floor_size[2]),
                               chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))

# === Main loop === advance physics, publish ROS state, keep real-time pace
realtime_timer = chrono.ChRealtimeStepTimer()   # cache: real-time pacing helper

os.makedirs("cam", exist_ok=True)               # guard against missing output dir

frame = 0
try:
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            time = sys.GetChTime()
            sys.DoStepDynamics(time_step)
            time = sys.GetChTime()
            if not ros_manager.Update(time, time_step):     # publish ROS state
                break
            realtime_timer.Spin(time_step)                  # keep wall-clock ~ sim time
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:       # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
