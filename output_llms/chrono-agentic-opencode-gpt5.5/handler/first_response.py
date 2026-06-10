"""PyChrono ROS handler demo with an NSC floor and movable box.

The simulation builds a gravity-driven rigid-body system with contact materials,
a fixed floor, and a dynamic box that slides on the floor. A ROS2 Python manager
publishes clock, body, transform, and custom integer messages while the Irrlicht
window renders the scene in real time.
"""


import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.ros as chros
import rclpy.publisher
from std_msgs.msg import Int64


# === Constants === named values keep geometry, timing, and ROS rates explicit
time_step = 1e-3
sim_end = 5.0
render_fps = 30.0
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once

floor_size_x = 6.0
floor_size_y = 0.1
floor_size_z = 4.0
floor_y = -floor_size_y / 2.0

box_size_x = 0.6
box_size_y = 0.6
box_size_z = 0.6
box_density = 800.0
box_start_y = box_size_y / 2.0
box_start_speed_x = 0.35

body_publish_rate = 30.0
tf_publish_rate = 30.0
custom_publish_rate = 2.0


class IntegerPublisherHandler(chros.ChROSHandler):
    """Publishes an incrementing integer message on a ROS topic."""

    def __init__(self, topic: str, update_rate: float):
        chros.ChROSHandler.__init__(self, update_rate)
        self.topic = topic
        self.publisher: rclpy.publisher.Publisher | None = None
        self.counter = 0

    def Initialize(self, interface: chros.ChROSPythonInterface) -> bool:
        self.publisher = interface.GetNode().create_publisher(Int64, self.topic, 1)
        return True

    def Tick(self, time: float):
        if self.publisher is None:
            return
        msg = Int64()
        msg.data = self.counter
        self.publisher.publish(msg)
        self.counter += 1


# === System & gravity === NSC contact system for rigid floor-box interaction
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0.0, -9.81, 0.0))
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

contact_mat = chrono.ChContactMaterialNSC()
contact_mat.SetFriction(0.55)
contact_mat.SetRestitution(0.05)


# === Bodies === fixed base_link floor and dynamic box with collision enabled
floor = chrono.ChBodyEasyBox(floor_size_x, floor_size_y, floor_size_z, 1000.0, True, True, contact_mat)
floor.SetName("base_link")
floor.SetFixed(True)
floor.SetPos(chrono.ChVector3d(0.0, floor_y, 0.0))
floor.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
sys.Add(floor)

box = chrono.ChBodyEasyBox(box_size_x, box_size_y, box_size_z, box_density, True, True, contact_mat)
box.SetName("movable_box")
box.SetPos(chrono.ChVector3d(-1.5, box_start_y, 0.0))
box.SetPosDt(chrono.ChVector3d(box_start_speed_x, 0.0, 0.0))
box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/bluewhite.png"))
sys.Add(box)

box_body = box  # cache: reused by ROS handlers and the hot loop
floor_body = floor  # cache: reused by TF handler setup


# === ROS handlers === publish clock, body state, TF, and custom integer topic
ros_manager = chros.ChROSPythonManager("chrono_handler_demo")
ros_manager.RegisterHandler(chros.ChROSClockHandler())
ros_manager.RegisterHandler(chros.ChROSBodyHandler(body_publish_rate, box_body, "~/box"))

tf_handler = chros.ChROSTFHandler(tf_publish_rate)
tf_handler.AddTransform(floor_body, floor_body.GetName(), box_body, box_body.GetName())
ros_manager.RegisterHandler(tf_handler)

integer_handler = IntegerPublisherHandler("~/custom_int", custom_publish_rate)
ros_manager.RegisterPythonHandler(integer_handler)
ros_manager.Initialize()


# === Visualization === Irrlicht scene initialized before adding camera/lights/grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Chrono ROS Handler Demo")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(3.0, 2.2, -4.0), chrono.ChVector3d(0.0, 0.35, 0.0))
vis.AddTypicalLights()
vis.AddGrid(
    0.5,
    0.5,
    12,
    12,
    chrono.ChCoordsysd(chrono.ChVector3d(0.0, 0.0, 0.0), chrono.Q_ROTATE_Z_TO_Y),
    chrono.ChColor(0.35, 0.35, 0.35),
)

realtime_timer = chrono.ChRealtimeStepTimer()


# === Main loop === render, step physics, update ROS, and keep wall-clock real time
frame = 0
try:
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            sim_time = sys.GetChTime()
            box_pos = box_body.GetPos()  # cache: one pose fetch per step
            box_vel = box_body.GetPosDt()  # cache: one velocity fetch per step
            sys.DoStepDynamics(time_step)
            if not ros_manager.Update(sys.GetChTime(), time_step):
                break
            realtime_timer.Spin(time_step)
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:  # solver divergence / invalid state
    print(f"Simulation failed: {exc}")
    raise
except (OSError, IOError) as exc:  # output path or ROS transport I/O failures
    print(f"I/O failure during simulation: {exc}")
    raise
finally:
    pass
