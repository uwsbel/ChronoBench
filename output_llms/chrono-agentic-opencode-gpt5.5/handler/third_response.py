"""ROS handler demo with a textured floor and falling box.

This self-contained PyChrono 9.0 NSC simulation builds a plain MBS scene with
contact between a fixed textured floor and a dynamic textured box. It registers
a custom Python ROS handler, body-state publishing, and TF publishing at a
shared 10 Hz rate, then advances the box under gravity while Irrlicht renders
the scene at a throttled cadence.
"""

import math
import traceback

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.ros as chros
from std_msgs.msg import Int64


class CounterHandler(chros.ChROSHandler):
    """Publishes an incrementing integer through a Python ROS handler."""

    def __init__(self, publish_rate, topic):
        chros.ChROSHandler.__init__(self, publish_rate)
        self.topic = topic
        self.publisher = None
        self.ticker = 0

    def Initialize(self, interface):
        self.publisher = interface.GetNode().create_publisher(Int64, self.topic, 1)
        return True

    def Tick(self, time):
        msg = Int64()
        msg.data = self.ticker
        self.publisher.publish(msg)
        self.ticker += 1


# === Constants === direct demo parameters for ROS, rendering, and contact bodies
time_step = 1.0e-3
sim_end = 3.0
publish_rate = 10.0
render_fps = 50.0
render_step_size = 1.0 / render_fps  # precomputed once: Irrlicht update interval
render_steps = max(1, math.ceil(render_step_size / time_step))  # precomputed once

floor_size_x = 10.0
floor_size_y = 0.2
floor_size_z = 10.0
box_size = 0.8
floor_y = -floor_size_y / 2.0
box_start_y = 3.0


# === System & gravity === NSC rigid contact scene with Bullet collision
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0.0, -9.81, 0.0))
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

contact_material = chrono.ChContactMaterialNSC()
contact_material.SetFriction(0.6)
contact_material.SetRestitution(0.1)

# === Bodies === fixed textured floor and one dynamic textured box for ROS state output
floor = chrono.ChBodyEasyBox(floor_size_x, floor_size_y, floor_size_z, 1000.0, True, True, contact_material)
floor.SetName("base_link")
floor.SetFixed(True)
floor.SetPos(chrono.ChVector3d(0.0, floor_y, 0.0))
floor.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
sys.Add(floor)

box = chrono.ChBodyEasyBox(box_size, box_size, box_size, 1000.0, True, True, contact_material)
box.SetName("box")
box.SetPos(chrono.ChVector3d(0.0, box_start_y, 0.0))
box.SetPosDt(chrono.ChVector3d(0.25, 0.0, 0.0))
box.SetAngVelParent(chrono.ChVector3d(0.0, 0.0, 3.0))
box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/bluewhite.png"))
sys.Add(box)

box_body = box  # cache: body reused by ROS handlers and loop logging

# === ROS handlers === clock, body, TF, and custom Python handler at a shared rate
ros_manager = chros.ChROSPythonManager()
ros_manager.RegisterHandler(chros.ChROSClockHandler())
ros_manager.RegisterHandler(chros.ChROSBodyHandler(publish_rate, box_body, "~/box"))

tf_handler = chros.ChROSTFHandler(publish_rate)
tf_handler.AddTransform(floor, floor.GetName(), box_body, box_body.GetName())
ros_manager.RegisterHandler(tf_handler)
ros_manager.RegisterPythonHandler(CounterHandler(publish_rate, "~/counter"))
ros_manager.Initialize()

# === Visualization === Irrlicht window with camera, sky, logo, and lights
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("PyChrono ROS Handler Demo")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(4.0, 3.0, 5.0), chrono.ChVector3d(0.3, 1.0, 0.0))
vis.AddTypicalLights()

# === Main loop === render every render_steps physics ticks and publish ROS state
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

try:

    while vis.Run() and sys.GetChTime() < sim_end:
        if step_number % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        sys.DoStepDynamics(time_step)
        sim_time = sys.GetChTime()

        if not ros_manager.Update(sim_time, time_step):
            break

        box_pos = box_body.GetPos()  # cache: one pose fetch for logging and checks
        box_vel = box_body.GetPosDt()  # cache: one velocity fetch for logging
        box_euler = box_body.GetRot().GetCardanAnglesXYZ()  # cache: one rotation conversion


        realtime_timer.Spin(time_step)
        step_number += 1
except (RuntimeError, ValueError) as exc:  # solver divergence or invalid state
    traceback.print_exc()
    raise
except (OSError, IOError) as exc:  # disk or ROS middleware I/O failures
    traceback.print_exc()
    raise
finally:
    pass
