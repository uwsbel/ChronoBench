"""
PyChrono 9.0.x simulation: Floor + Box MBS with Custom ROS Handler (Irrlicht visualization).

System type: ChSystemNSC (NSC rigid body contact).
Bodies: a fixed floor and a dynamic box resting on the floor.
ROS: ChROSPythonManager with a custom Python ChROSHandler subclass (publishes an
     incrementing Int64 counter at 1 Hz), ChROSBodyHandler (publishes box pose),
     ChROSTFHandler (publishes TF frames), and ChROSClockHandler.
Visualization: Irrlicht window with sky box, camera, and typical lights.
Rendering is throttled via render_step_size / render_steps so the loop runs at a
reasonable frame rate without calling vis.Run() every 1 ms physics step.
Expected behavior: the box rests on the floor under gravity; ROS topics /clock,
~/output/box, and /tf are published; the custom handler increments a counter on
~/output/handler/number.
"""

# === Imports ===
import os
import math

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.ros as chros

import rclpy.publisher
from std_msgs.msg import Int64


# === Simulation parameters ===
time_step = 1e-3        # physics step size [s]
sim_end   = 10.0        # simulation duration [s]

# Rendering cadence (matched to input3 variables)
render_fps       = 50.0
render_step_size = 1.0 / render_fps          # time between rendered frames [s]
render_steps     = max(1, round(render_step_size / time_step))  # physics steps per frame; precomputed once

# ROS publish rate
publish_rate = 10  # [Hz] used for body/TF/clock handlers

# Body geometry
floor_sx, floor_sy, floor_sz = 10.0, 0.2, 10.0   # floor full extents [m]
box_sx,   box_sy,   box_sz   = 0.5,  0.5,  0.5   # box full extents [m]
box_density = 1000.0                              # [kg/m³]

# Positions (Y-up: floor top surface at y = floor_sy/2 = 0.1)
floor_y   = 0.0                      # floor center Y
box_spawn_y = floor_sy / 2.0 + box_sy / 2.0   # box center resting on floor top


# === Custom ROS handler ===
class MyCustomHandler(chros.ChROSHandler):
    """Publishes an incrementing Int64 counter on ~/output/handler/number at 1 Hz."""

    def __init__(self, topic: str):
        super().__init__(1)   # 1 Hz
        self.topic = topic
        self.publisher: rclpy.publisher.Publisher = None
        self.counter = 0

    def Initialize(self, interface: chros.ChROSPythonInterface) -> bool:
        self.publisher = interface.GetNode().create_publisher(Int64, self.topic, 1)
        return True

    def Tick(self, time: float) -> None:
        msg = Int64()
        msg.data = self.counter
        self.publisher.publish(msg)
        self.counter += 1


# === System & gravity ===
sys = chrono.ChSystemNSC()
sys.SetGravityY()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Contact material ===
mat = chrono.ChContactMaterialNSC()
mat.SetFriction(0.8)
mat.SetRestitution(0.01)

# === Bodies ===
# Fixed floor
floor = chrono.ChBodyEasyBox(floor_sx, floor_sy, floor_sz, 1000.0, True, True, mat)
floor.SetFixed(True)
floor.SetPos(chrono.ChVector3d(0.0, floor_y, 0.0))
floor.SetName("base_link")
floor.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
sys.Add(floor)

# Dynamic box
box = chrono.ChBodyEasyBox(box_sx, box_sy, box_sz, box_density, True, True, mat)
box.SetPos(chrono.ChVector3d(0.0, box_spawn_y, 0.0))
box.SetName("box")
box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/bluewhite.png"))
sys.Add(box)

# === ROS manager ===
ros_manager = chros.ChROSPythonManager()

# Clock handler first
ros_manager.RegisterHandler(chros.ChROSClockHandler())

# Body handler — publishes box pose
ros_manager.RegisterHandler(chros.ChROSBodyHandler(publish_rate, box, "~/output/box"))

# TF handler — floor → box
tf_handler = chros.ChROSTFHandler(publish_rate)
tf_handler.AddTransform(floor, floor.GetName(), box, box.GetName())
ros_manager.RegisterHandler(tf_handler)

# Custom Python handler
ros_manager.RegisterPythonHandler(MyCustomHandler("~/output/handler/number"))

# Initialize after all registration
ros_manager.Initialize()

# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Handler Demo — Floor + Box + ROS")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.Initialize()                        # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(3, 2, 4), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()

# === Recording setup ===


# === Main loop ===
step_number = 0    # tracks physics steps for render throttling
frame = 0

try:
    while vis.Run() and sys.GetChTime() < sim_end:
        time = sys.GetChTime()

        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        # Advance physics render_steps at a time
        for _ in range(render_steps):
            t = sys.GetChTime()
            if t >= sim_end:
                break
            sys.DoStepDynamics(time_step)
            if not ros_manager.Update(t, time_step):
                break
            step_number += 1

except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
