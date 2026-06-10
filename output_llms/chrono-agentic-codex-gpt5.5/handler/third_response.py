"""PyChrono NSC scene with a textured floor, textured box, Irrlicht view, and ROS handlers.

The simulation builds a simple contact scene where a box settles on a fixed floor.
ROS2 publishes clock, body pose, TF, and a custom box-height message at 10 Hz while
Irrlicht renders the scene with a camera, lights, and window-level timing.
"""

import math
import traceback

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.ros as chros
import rclpy.publisher
from std_msgs.msg import String


# === Constants ===
# Geometry and timing values are named so the source stays reviewable.
TIME_STEP = 0.001
SIM_END = 5.0
RENDER_FPS = 30.0
render_step_size = 1.0 / RENDER_FPS  # precomputed once
render_steps = max(1, math.ceil(render_step_size / TIME_STEP))  # precomputed once
publish_rate = 10

FLOOR_SIZE_X = 8.0
FLOOR_SIZE_Y = 8.0
FLOOR_SIZE_Z = 0.1
BOX_SIZE = 1.0
BOX_DENSITY = 500.0
BOX_START_Z = BOX_SIZE / 2.0
FRICTION = 0.6
RESTITUTION = 0.1


class BoxStatusHandler(chros.ChROSHandler):
    """Publishes a compact textual status for the box at the configured rate."""

    def __init__(self, rate, body, topic):
        super().__init__(rate)
        self.body = body
        self.topic = topic
        self.publisher: rclpy.publisher.Publisher | None = None

    def Initialize(self, interface: chros.ChROSPythonInterface) -> bool:
        self.publisher = interface.GetNode().create_publisher(String, self.topic, 1)
        return True

    def Tick(self, time: float):
        if self.publisher is None:
            return
        pos = self.body.GetPos()
        msg = String()
        msg.data = f"time={time:.3f}, box_z={pos.z:.4f}"
        self.publisher.publish(msg)


def add_texture(body, texture_rel_path):
    """Apply a Chrono bundled texture to the first visual shape when present."""
    try:
        shape = body.GetVisualShape(0)
        shape.SetTexture(chrono.GetChronoDataFile(texture_rel_path))
    except (RuntimeError, AttributeError) as exc:
        print(f"Texture assignment skipped for {body.GetName()}: {exc}")


def build_scene():
    """Create the physical system, contact bodies, visualization, and ROS manager."""
    # === System & gravity ===
    sys = chrono.ChSystemNSC()
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
    sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    sys.SetSolverType(chrono.ChSolver.Type_PSOR)
    sys.GetSolver().AsIterative().SetMaxIterations(80)

    # === Materials ===
    contact_mat = chrono.ChContactMaterialNSC()
    contact_mat.SetFriction(FRICTION)
    contact_mat.SetRestitution(RESTITUTION)

    # === Bodies ===
    floor = chrono.ChBodyEasyBox(
        FLOOR_SIZE_X, FLOOR_SIZE_Y, FLOOR_SIZE_Z, 1000.0, True, True, contact_mat
    )
    floor.SetName("base_link")
    floor.SetFixed(True)
    floor.SetPos(chrono.ChVector3d(0, 0, -FLOOR_SIZE_Z / 2.0))
    add_texture(floor, "textures/concrete.jpg")
    sys.Add(floor)

    box = chrono.ChBodyEasyBox(
        BOX_SIZE, BOX_SIZE, BOX_SIZE, BOX_DENSITY, True, True, contact_mat
    )
    box.SetName("box")
    box.SetPos(chrono.ChVector3d(0, 0, BOX_START_Z))
    add_texture(box, "textures/bluewhite.png")
    sys.Add(box)

    # === Visualization ===
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("ROS Handler Scene")
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(4.5, -5.0, 3.0), chrono.ChVector3d(0, 0, 0.5))
    vis.AddTypicalLights()
    vis.AddLight(chrono.ChVector3d(-3, 4, 6), 9, chrono.ChColor(0.85, 0.85, 0.85))
    vis.AddGrid(
        0.5,
        0.5,
        16,
        16,
        chrono.ChCoordsysd(),
        chrono.ChColor(0.35, 0.35, 0.35),
    )

    # === ROS handlers ===
    ros_manager = chros.ChROSPythonManager()
    ros_manager.RegisterHandler(chros.ChROSClockHandler())
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(publish_rate, box, "~/box"))
    tf_handler = chros.ChROSTFHandler(publish_rate)
    tf_handler.AddTransform(floor, floor.GetName(), box, box.GetName())
    ros_manager.RegisterHandler(tf_handler)
    ros_manager.RegisterPythonHandler(BoxStatusHandler(publish_rate, box, "~/box_status"))
    ros_manager.Initialize()

    return sys, vis, ros_manager, floor, box


def main():
    """Run the real-time simulation loop and update ROS once per physics step."""
    sys, vis, ros_manager, floor, box = build_scene()
    box_body = box  # cache: reused every review sample
    timer = chrono.ChRealtimeStepTimer()
    step_number = 0

    try:
        while vis.Run() and sys.GetChTime() < SIM_END:
            if step_number % render_steps == 0:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()

            for _ in range(render_steps):
                sys.DoStepDynamics(TIME_STEP)
                time = sys.GetChTime()
                if not ros_manager.Update(time, TIME_STEP):
                    return
                timer.Spin(TIME_STEP)
                step_number += 1
                if time >= SIM_END:
                    break
    except (RuntimeError, ValueError) as exc:
        print(f"Simulation failed during stepping: {exc}")
        traceback.print_exc()
        raise
    finally:
        pass


if __name__ == "__main__":
    main()
