"""Plain NSC MBS scene with ROS2 custom handler publishing string status.

The simulation contains a fixed floor and a falling box with contact in a Y-up
Chrono system. ROS publishes the simulation clock, box body state, a TF transform,
and a custom String message of the form "Hello, world! At time: <ticker>".
"""

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.ros as chros
import rclpy.publisher
from std_msgs.msg import String


# === Constants === direct values keep the ROS handler demo compact and readable
time_step = 1e-3
sim_end = 5.0
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once
floor_size_x = 4.0
floor_size_y = 0.1
floor_size_z = 4.0
box_size = 0.4
box_density = 1000.0


class MyCustomHandler(chros.ChROSHandler):
    """Publishes a String message at 1 Hz through ChROSPythonManager."""

    def __init__(self, topic):
        super().__init__(1)
        self.topic = topic
        self.publisher: rclpy.publisher.Publisher = None
        self.ticker = 0
        self.message = "Hello, world! At time: "

    def Initialize(self, interface: chros.ChROSPythonInterface) -> bool:
        self.publisher = interface.GetNode().create_publisher(String, self.topic, 1)
        return True

    def Tick(self, time: float):
        msg = String()
        msg.data = self.message + str(self.ticker)
        self.publisher.publish(msg)
        self.ticker += 1


# === System & gravity === NSC contact system for the floor-box interaction
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0.0, -9.81, 0.0))
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Bodies === fixed root floor plus one dynamic box observed by ROS handlers
contact_mat = chrono.ChContactMaterialNSC()
contact_mat.SetFriction(0.6)
contact_mat.SetRestitution(0.0)

floor = chrono.ChBodyEasyBox(floor_size_x, floor_size_y, floor_size_z, 1000.0, True, True, contact_mat)
floor.SetName("base_link")
floor.SetFixed(True)
floor.SetPos(chrono.ChVector3d(0.0, -floor_size_y / 2.0, 0.0))
floor.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
sys.Add(floor)

box = chrono.ChBodyEasyBox(box_size, box_size, box_size, box_density, True, True, contact_mat)
box.SetName("box")
box.SetPos(chrono.ChVector3d(0.0, 1.0, 0.0))
box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/bluewhite.png"))
sys.Add(box)

floor_body = floor  # cache: named root body reused by TF and logging
box_body = box  # cache: named dynamic body reused by ROS body handler and logging

# === ROS handlers === clock first, then custom String, body, and TF publishers
ros_manager = chros.ChROSPythonManager()
ros_manager.RegisterHandler(chros.ChROSClockHandler())
custom_handler = MyCustomHandler("~/my_topic")
ros_manager.RegisterPythonHandler(custom_handler)
ros_manager.RegisterHandler(chros.ChROSBodyHandler(25, box_body, "~/box"))
tf_handler = chros.ChROSTFHandler(25)
tf_handler.AddTransform(floor_body, floor_body.GetName(), box_body, box_body.GetName())
ros_manager.RegisterHandler(tf_handler)
ros_manager.Initialize()

# === Visualization === Irrlicht window initialized before adding scene elements
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("ROS custom String handler")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(2.2, 1.4, -3.0), chrono.ChVector3d(0.0, 0.4, 0.0))
vis.AddTypicalLights()


# === Main loop === render and step physics with ROS once per step
try:

    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        for _ in range(render_every):
            time = sys.GetChTime()  # cache: used for logging and ROS update this step
            box_pos = box_body.GetPos()  # cache: body pose fetched once this step
            box_vel = box_body.GetPosDt()  # cache: body velocity fetched once this step
            sys.DoStepDynamics(time_step)
            if not ros_manager.Update(sys.GetChTime(), time_step):
                break
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:  # guard solver divergence or invalid state
    import traceback

    traceback.print_exc()
    raise
except (OSError, IOError) as exc:  # guard recording file-system failures
    import traceback

    traceback.print_exc()
    raise
finally:
    pass
