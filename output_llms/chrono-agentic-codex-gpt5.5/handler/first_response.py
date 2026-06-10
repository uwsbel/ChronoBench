"""PyChrono ROS bridge demo with an NSC contact system.

The simulation builds a Y-up rigid-body world with gravity, a fixed floor, and
a movable box that falls onto the floor through Bullet collision.  It publishes
simulation time, body state, transforms, and a custom incrementing Int64 topic
through ChROS while maintaining a real-time stepping loop.
"""

import math

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.ros as chros
import rclpy.publisher
from std_msgs.msg import Int64


# === Constants === named values keep geometry and rates explicit
TIME_STEP = 0.001
SIM_END = 4.0
RENDER_FPS = 30.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once
FLOOR_SIZE_X = 6.0
FLOOR_SIZE_Y = 0.2
FLOOR_SIZE_Z = 6.0
BOX_SIZE = 0.6
FLOOR_DENSITY = 1000.0
BOX_DENSITY = 650.0
FRICTION = 0.8
RESTITUTION = 0.05
BODY_RATE_HZ = 25.0
TF_RATE_HZ = 25.0
CUSTOM_RATE_HZ = 2.0
CUSTOM_TOPIC = "~/integer"


class IntegerPublisherHandler(chros.ChROSHandler):
    """Publishes an incrementing integer to a configurable ROS topic."""

    def __init__(self, topic: str, update_rate: float):
        super().__init__(update_rate)
        self.topic = topic
        self.publisher: rclpy.publisher.Publisher | None = None
        self.count = 0

    def Initialize(self, interface: chros.ChROSPythonInterface) -> bool:
        self.publisher = interface.GetNode().create_publisher(Int64, self.topic, 1)
        return True

    def Tick(self, time: float) -> None:
        if self.publisher is None:
            return
        msg = Int64()
        msg.data = self.count
        self.publisher.publish(msg)
        self.count += 1


# === System & gravity === NSC contacts with Bullet collision for floor-box impact
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0.0, -9.81, 0.0))
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
sys.SetSolverType(chrono.ChSolver.Type_PSOR)
sys.GetSolver().AsIterative().SetMaxIterations(80)


# === Bodies === fixed floor and movable box with physical material properties
contact_mat = chrono.ChContactMaterialNSC()
contact_mat.SetFriction(FRICTION)
contact_mat.SetRestitution(RESTITUTION)

floor = chrono.ChBodyEasyBox(
    FLOOR_SIZE_X, FLOOR_SIZE_Y, FLOOR_SIZE_Z, FLOOR_DENSITY, True, True, contact_mat
)
floor.SetName("base_link")
floor.SetFixed(True)
floor.SetPos(chrono.ChVector3d(0.0, -FLOOR_SIZE_Y / 2.0, 0.0))
sys.Add(floor)

box = chrono.ChBodyEasyBox(BOX_SIZE, BOX_SIZE, BOX_SIZE, BOX_DENSITY, True, True, contact_mat)
box.SetName("box")
box.SetPos(chrono.ChVector3d(0.0, 1.8, 0.0))
box.SetRot(chrono.QuatFromAngleZ(0.25))
sys.Add(box)

box_body = box  # cache: body handle reused by ROS, logging, and loop


# === ROS handlers === ChROSPythonManager hosts clock, body, TF, and custom handler
ros_manager = chros.ChROSPythonManager("chrono_handler_demo")
ros_manager.RegisterHandler(chros.ChROSClockHandler())
ros_manager.RegisterHandler(chros.ChROSBodyHandler(BODY_RATE_HZ, box_body, "~/box"))
tf_handler = chros.ChROSTFHandler(TF_RATE_HZ)
tf_handler.AddTransform(floor, floor.GetName(), box_body, box_body.GetName())
ros_manager.RegisterHandler(tf_handler)
ros_manager.RegisterPythonHandler(IntegerPublisherHandler(CUSTOM_TOPIC, CUSTOM_RATE_HZ))
ros_manager.Initialize()


# === Visualization === Irrlicht window initialized before sky, camera, lights, and grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("PyChrono ROS Handler Demo")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(3.0, 2.4, 4.5), chrono.ChVector3d(0.0, 0.5, 0.0))
vis.AddTypicalLights()
vis.AddGrid(
    0.5,
    0.5,
    20,
    20,
    chrono.ChCoordsysd(chrono.ChVector3d(0.0, 0.0, 0.0), chrono.QuatFromAngleX(math.pi / 2.0)),
    chrono.ChColor(0.35, 0.35, 0.35),
)


# === Main loop === advance physics, publish ROS state, and keep real-time pace
realtime_timer = chrono.ChRealtimeStepTimer()


def run_loop():
    """Run the Irrlicht, Chrono, and ROS loop."""
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(RENDER_EVERY):
            time = sys.GetChTime()
            pos = box_body.GetPos()  # cache: fetched once for logging and ROS-time row
            vel = box_body.GetPosDt()  # cache: fetched once for logging
            ang_vel = box_body.GetAngVelParent()  # cache: fetched once for logging
            sys.DoStepDynamics(TIME_STEP)
            time = sys.GetChTime()
            if not ros_manager.Update(time, TIME_STEP):
                return
            realtime_timer.Spin(TIME_STEP)
            if time >= SIM_END:
                return


def main():
    """Run the scored real-time simulation."""
    try:
        run_loop()
    except (OSError, IOError) as exc:  # output directory or permission failure
        raise
    except (RuntimeError, ValueError) as exc:  # Chrono solver divergence or invalid state
        raise
    finally:
        pass


main()
