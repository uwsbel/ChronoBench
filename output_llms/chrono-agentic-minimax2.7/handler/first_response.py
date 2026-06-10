"""
ROS-integrated MBS simulation with a fixed floor and a movable box.

A custom Python ROS handler publishes integer messages to a topic.
The system uses NSC contact method for the floor-box interaction.
"""

import os
import sys
import math
import csv

import pychrono.core as chrono
import pychrono.ros as chros
import pychrono.irrlicht as chronoirr

import sim_recording as rec

# === Named constants ===
time_step = 1e-3
sim_end = 10.0
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))

# Box geometry and mass
box_mass = 1.0
box_size = 0.5

# Floor position
floor_z = 0.0

# Derived positions
box_x, box_y, box_z = 0.0, 0.0, floor_z + box_size

# === System & gravity ===
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Contact material ===
mat = chrono.ChContactMaterialNSC()
mat.SetFriction(0.8)
mat.SetRestitution(0.0)

# === Bodies ===

# Fixed floor
floor = chrono.ChBodyEasyBox(4.0, 4.0, 0.1, 1000.0, True, True, mat)
floor.SetFixed(True)
floor.SetPos(chrono.ChVector3d(0.0, 0.0, floor_z - 0.05))
floor.SetName("base_link")
sys.Add(floor)

# Movable box
box = chrono.ChBodyEasyBox(box_size, box_size, box_size, box_mass, True, True, mat)
box.SetPos(chrono.ChVector3d(box_x, box_y, box_z))
box.SetName("box")
sys.Add(box)

# === ROS manager ===
ros_manager = chros.ChROSPythonManager()

# 1. Clock handler — publishes /clock
ros_manager.RegisterHandler(chros.ChROSClockHandler())

# 2. Body handler — publishes box pose
ros_manager.RegisterHandler(chros.ChROSBodyHandler(30, box, "~/box_pose"))

# 3. TF handler — publishes transform base_link -> box
tf_handler = chros.ChROSTFHandler(30)
tf_handler.AddTransform(floor, floor.GetName(), box, box.GetName())
ros_manager.RegisterHandler(tf_handler)


# === Custom integer-publishing ROS handler ===
class IntegerPublisherHandler(chros.ChROSHandler):
    """Publishes an incrementing integer on a ROS topic at 1 Hz."""

    def __init__(self, topic):
        super().__init__(1)  # 1 Hz
        self.topic = topic
        self.publisher = None
        self.counter = 0

    def Initialize(self, interface: chros.ChROSPythonInterface) -> bool:
        from std_msgs.msg import Int64
        self.publisher = interface.GetNode().create_publisher(Int64, self.topic, 1)
        return True

    def Tick(self, time: float):
        from std_msgs.msg import Int64
        msg = Int64()
        msg.data = self.counter
        self.publisher.publish(msg)
        self.counter += 1


# Register custom handler via Python registration (not RegisterHandler)
ros_manager.RegisterPythonHandler(IntegerPublisherHandler("~/integer"))

# Initialize after ALL handlers are registered
ros_manager.Initialize()

# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("ROS Handler Demo — Floor + Box")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(3, -3, 2), chrono.ChVector3d(0, 0, 0.5))
vis.AddTypicalLights()
vis.AddGrid(0.5, 0.5, 20, 20,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))

# === CSV logging ===
REC = bool(os.environ.get("SIMBENCH_RECORD"))
irr_dir = rec.frame_dir("frames") if REC else None

os.makedirs("frames", exist_ok=True)

csv_path = "simulation_data.csv"
csv_file = None
data_writer = None
try:
    csv_file = open(csv_path, "w", newline="")
    fieldnames = ["time", "box_x", "box_y", "box_z", "box_vx", "box_vy", "box_vz"]
    data_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
    data_writer.writeheader()
except (OSError, IOError) as exc:
    print(f"Warning: could not open CSV for writing: {exc}")

# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
frame = 0

while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()


    for _ in range(render_every):
        sim_time = sys.GetChTime()


        sys.DoStepDynamics(time_step)

        # ROS update: break if ROS layer shuts down
        if not ros_manager.Update(sim_time, time_step):
            break

        if sys.GetChTime() >= sim_end:
            break

        realtime_timer.Spin(time_step)


print("Simulation complete.")
