"""Floor + box rigid-body scene published over ROS2 via pychrono.ros.

System type: NSC (ChSystemNSC). The scene is a fixed textured floor and a single
dynamic textured box resting/settling under gravity. A ChROSPythonManager bridges
the simulation to ROS2: a clock handler publishes /clock, a body handler publishes
the box pose/twist, a TF handler publishes the floor->box transform, and a custom
Python handler publishes an incrementing counter. All handlers tick at 10 Hz.
Expected behavior: the box rests on the floor (no fall-through) while the ROS
handlers stream state at the configured publish rate; the Irrlicht window shows
the textured floor and box.
"""

import os

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.ros as chros

import rclpy.publisher
from std_msgs.msg import Int64


# === Named constants === geometry / physics / publish rate
time_step = 1e-3            # integration step [s]
sim_end = 5.0              # total simulated time [s]
render_fps = 50.0          # review render cadence [frames/s]
publish_rate = 10          # ROS handler publish rate [Hz]

floor_sx, floor_sy, floor_sz = 6.0, 6.0, 0.2     # floor full extents [m]
box_size = 0.5                                    # box full edge length [m]
box_density = 1000.0                              # box material density [kg/m^3]

# Derived placement: box centered above the floor top surface.
floor_top_z = floor_sz / 2.0
box_z = floor_top_z + box_size / 2.0              # box sits on the floor

render_every = max(1, round(1.0 / (render_fps * time_step)))   # precomputed once

# === System & gravity === NSC system with Bullet collision (floor/box contact)
sys = chrono.ChSystemNSC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

# === Contact material === shared NSC material for floor + box
mat = chrono.ChContactMaterialNSC()
mat.SetFriction(0.8)
mat.SetRestitution(0.0)

# === Bodies === fixed textured floor + dynamic textured box
floor = chrono.ChBodyEasyBox(floor_sx, floor_sy, floor_sz, box_density, True, True, mat)
floor.SetPos(chrono.ChVector3d(0, 0, 0))
floor.SetFixed(True)
floor.SetName("base_link")        # conventional TF root frame
floor.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
sys.Add(floor)

box = chrono.ChBodyEasyBox(box_size, box_size, box_size, box_density, True, True, mat)
box.SetPos(chrono.ChVector3d(0, 0, box_z))
box.SetName("box")
box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
sys.Add(box)


# === Custom ROS handler === publishes an incrementing integer at publish_rate Hz
class CounterHandler(chros.ChROSHandler):
    """Publishes a monotonically increasing Int64 counter on a ROS topic."""

    def __init__(self, topic):
        super().__init__(publish_rate)        # publish rate in Hz
        self.topic = topic
        self.publisher: rclpy.publisher.Publisher = None
        self.ticker = 0

    def Initialize(self, interface: chros.ChROSPythonInterface) -> bool:
        self.publisher = interface.GetNode().create_publisher(Int64, self.topic, 1)
        return True

    def Tick(self, time: float):
        msg = Int64()
        msg.data = self.ticker
        self.publisher.publish(msg)
        self.ticker += 1


# === ROS manager === clock first, then body / TF / custom handlers, then Initialize
ros_manager = chros.ChROSPythonManager()
ros_manager.RegisterHandler(chros.ChROSClockHandler())

body_handler = chros.ChROSBodyHandler(publish_rate, box, "~/box/state")
ros_manager.RegisterHandler(body_handler)

tf_handler = chros.ChROSTFHandler(publish_rate)
tf_handler.AddTransform(floor, floor.GetName(), box, box.GetName())
ros_manager.RegisterHandler(tf_handler)

ros_manager.RegisterPythonHandler(CounterHandler("~/counter"))

ros_manager.Initialize()

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Floor + Box over ROS2")
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(4, -4, 3), chrono.ChVector3d(0, 0, 0.3))
vis.AddTypicalLights()
vis.AddGrid(0.5, 0.5, 20, 20,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, floor_top_z + 1e-3), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))

# === Main loop === render-cadence outer loop; physics + ROS publish inner batch


frame = 0
try:
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            time = sys.GetChTime()
            sys.DoStepDynamics(time_step)
            if not ros_manager.Update(time, time_step):
                break
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:    # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
