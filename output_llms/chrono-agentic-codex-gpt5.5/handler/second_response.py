"""Plain MBS ROS handler demo using an NSC contact system.

The simulation contains a fixed floor and a falling box so standard body and TF
ROS handlers have real Chrono state to publish. A custom Python ChROSHandler
publishes a std_msgs/String message with a fixed prefix and the current ticker,
so the topic emits "Hello, world! At time: <n>" at one hertz.
"""

import csv
import os

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.ros as chros
import rclpy.publisher
from std_msgs.msg import String


# === Constants ===
TIME_STEP = 0.005
SIM_END = 6.0
RENDER_FPS = 30.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

FLOOR_SIZE_X = 5.0
FLOOR_SIZE_Y = 0.2
FLOOR_SIZE_Z = 5.0
BOX_SIZE = 0.6
BOX_START_Y = 2.0
BODY_DENSITY = 1000.0


# === Custom ROS handler ===
class MyCustomHandler(chros.ChROSHandler):
    """Publishes a String message with an incrementing ticker."""

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


# === System & gravity ===
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0.0, -9.81, 0.0))
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

contact_mat = chrono.ChContactMaterialNSC()
contact_mat.SetFriction(0.6)
contact_mat.SetRestitution(0.1)


# === Bodies ===
floor = chrono.ChBodyEasyBox(
    FLOOR_SIZE_X,
    FLOOR_SIZE_Y,
    FLOOR_SIZE_Z,
    BODY_DENSITY,
    True,
    True,
    contact_mat,
)
floor.SetName("base_link")
floor.SetFixed(True)
floor.SetPos(chrono.ChVector3d(0.0, -FLOOR_SIZE_Y / 2.0, 0.0))
sys.Add(floor)

box = chrono.ChBodyEasyBox(
    BOX_SIZE,
    BOX_SIZE,
    BOX_SIZE,
    BODY_DENSITY,
    True,
    True,
    contact_mat,
)
box.SetName("box")
box.SetPos(chrono.ChVector3d(0.0, BOX_START_Y, 0.0))
sys.Add(box)

box_body = box  # cache: body handle reused by ROS handlers and the loop


# === ROS handlers ===
ros_manager = chros.ChROSPythonManager()
ros_manager.RegisterHandler(chros.ChROSClockHandler())
ros_manager.RegisterHandler(chros.ChROSBodyHandler(25, box_body, "~/box"))
tf_handler = chros.ChROSTFHandler(25)
tf_handler.AddTransform(floor, floor.GetName(), box_body, box_body.GetName())
ros_manager.RegisterHandler(tf_handler)
ros_manager.RegisterPythonHandler(MyCustomHandler("~/my_topic"))
ros_manager.Initialize()


# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("PyChrono ROS Custom String Handler")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(4.0, 3.0, -5.0), chrono.ChVector3d(0.0, 0.8, 0.0))
vis.AddTypicalLights()
vis.AddGrid(
    0.5,
    0.5,
    12,
    12,
    chrono.ChCoordsysd(chrono.ChVector3d(0.0, 0.01, 0.0), chrono.Q_ROTATE_X_TO_Y),
    chrono.ChColor(0.35, 0.35, 0.35),
)


# === Main loop ===
frame = 0

try:

    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        for _ in range(RENDER_EVERY):
            box_pos = box_body.GetPos()  # cache: position fetched once for this step
            box_vel = box_body.GetPosDt()  # cache: velocity fetched once for this step
            sim_time = sys.GetChTime()  # cache: used for ROS update and logging
            sys.DoStepDynamics(TIME_STEP)
            sim_time = sys.GetChTime()
            if not ros_manager.Update(sim_time, TIME_STEP):
                break
            if sim_time >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:  # solver divergence / invalid runtime state
    import traceback

    traceback.print_exc()
    raise
except (OSError, IOError) as exc:  # disk or permission failure during review output
    import traceback

    traceback.print_exc()
    raise
finally:
    pass
