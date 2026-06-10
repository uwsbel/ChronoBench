"""
PyChrono simulation: floor + box rigid-body scene with a custom ROS handler.

System: ChSystemNSC (Y-up gravity). A fixed floor body and a dynamic box body
are created with NSC contact materials. A custom ChROSHandler subclass publishes
an incrementing Int64 counter, alongside ChROSBodyHandler (box pose) and
ChROSTFHandler (floor→box TF). Irrlicht visualization shows the scene with
textured floor and box, sky box, camera, and lights. Conditional rendering is
driven by step_number / render_steps (10 Hz publish rate).
"""

import os
import math
import csv
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.ros as chros
import rclpy.publisher
from std_msgs.msg import Int64


# === Constants ===
time_step       = 1e-3          # physics step [s]
sim_end         = 10.0          # simulation duration [s]
publish_rate    = 10            # ROS publish rate [Hz]
render_fps      = 50.0          # Irrlicht render rate [Hz]
render_step_size = 1.0 / render_fps   # seconds per render frame
render_steps     = max(1, round(render_step_size / time_step))  # physics steps per render frame; precomputed once

# Floor geometry
floor_sx = 10.0   # full width  [m]
floor_sy = 0.2    # full height [m]
floor_sz = 10.0   # full depth  [m]
floor_y  = -0.1   # Y centre (half-height below zero)

# Box geometry
box_sx = 1.0      # full width  [m]
box_sy = 1.0      # full height [m]
box_sz = 1.0      # full depth  [m]
box_y  = box_sy / 2.0 + 0.0   # resting on floor surface (floor top at y=0)

# === Custom ROS Handler ===

class MyCustomHandler(chros.ChROSHandler):
    """Publishes an incrementing Int64 counter on ~/output/handler/data at publish_rate Hz."""

    def __init__(self, topic: str):
        super().__init__(publish_rate)
        self.topic = topic
        self.publisher: rclpy.publisher.Publisher = None
        self.ticker = 0

    def Initialize(self, interface: chros.ChROSPythonInterface) -> bool:
        self.publisher = interface.GetNode().create_publisher(Int64, self.topic, 1)
        return True  # MUST return True on success

    def Tick(self, time: float):
        msg = Int64()
        msg.data = self.ticker
        self.publisher.publish(msg)
        self.ticker += 1


# === System & gravity ===
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
sys.SetSolverType(chrono.ChSolver.Type_PSOR)
sys.GetSolver().AsIterative().SetMaxIterations(100)

# === Contact material ===
mat = chrono.ChContactMaterialNSC()
mat.SetFriction(0.8)
mat.SetRestitution(0.0)

# === Bodies ===

# Fixed floor body with texture
floor = chrono.ChBodyEasyBox(floor_sx, floor_sy, floor_sz, 1000.0, True, True, mat)
floor.SetPos(chrono.ChVector3d(0, floor_y, 0))
floor.SetFixed(True)
floor.SetName("base_link")
floor.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
sys.Add(floor)

# Dynamic box body with texture
box = chrono.ChBodyEasyBox(box_sx, box_sy, box_sz, 500.0, True, True, mat)
box.SetPos(chrono.ChVector3d(0, box_y, 0))
box.SetFixed(False)
box.SetName("box")
box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/bluewhite.png"))
sys.Add(box)

# === ROS Manager ===
ros_manager = chros.ChROSPythonManager()

# 1. Clock handler first
ros_manager.RegisterHandler(chros.ChROSClockHandler())

# 2. Body handler — publishes box pose/twist
ros_manager.RegisterHandler(chros.ChROSBodyHandler(publish_rate, box, "~/output/body/data"))

# 3. TF handler — floor (base_link) → box frame
tf_handler = chros.ChROSTFHandler(publish_rate)
tf_handler.AddTransform(floor, floor.GetName(), box, box.GetName())
ros_manager.RegisterHandler(tf_handler)

# 4. Custom Python handler
ros_manager.RegisterPythonHandler(MyCustomHandler("~/output/handler/data"))

# Initialize ONCE after all registration
ros_manager.Initialize()

# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Handler Demo — ROS Custom Handler")
vis.Initialize()                                          # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(6, 4, -6), chrono.ChVector3d(0, 0.5, 0))  # AFTER Initialize
vis.AddTypicalLights()
vis.AddGrid(1.0, 1.0, 20, 20,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))

# === Review-only setup ===


# === Main loop ===
step_number = 0   # tracks physics steps for conditional rendering / ROS updates
frame = 0         # consecutive frame counter for review video

try:
    while vis.Run() and sys.GetChTime() < sim_end:
        if step_number % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()


        sys.DoStepDynamics(time_step)
        time = sys.GetChTime()  # cache: fetched once per step, reused for ROS update

        if not ros_manager.Update(time, time_step):
            break

        step_number += 1

except (RuntimeError, ValueError) as exc:  # solver divergence / bad physics state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
