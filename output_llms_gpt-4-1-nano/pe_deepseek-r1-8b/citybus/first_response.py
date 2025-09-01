# Import necessary modules
import os
import math
import numpy as np
import chrono as chrono
from pychrono import robot, vehicle, irrrlicht as chronoirr

# Initialize the simulation system
sys = chrono.ChSystemSMC()
chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), "data"))  # Set data path

# Initialize visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('CityBus Simulation')
vis.AddLogo(chrono.GetChronoDataFile('pychrono_alpha.png'))
vis.AddSkyBox()
cam_pos = chrono.ChVector3d(0, 3, 6)
cam_rot = chrono.QuatFromAngleX(math.pi / 2)  # Looking along Y-axis
vis.AddCamera(cam_pos, cam_rot)

# Initialize vehicle
vehicle = veh.ARTcar(sys)
vehicle.SetContactMethod(chrono.ChContactMethodVehicle)
vehicle.SetChassisCollisionType(chrono.ChCollisionTypeVehicle)
vehicle.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(math.pi / 2)))
vehicle.Initialize()

# Vehicle parts
# Chassis
chassis = chrono.ChBody()
chassis.SetMass(100)
chassis.SetInertiaXX(chrono.ChVector3d(5, 5, 5))
chassis.SetVisualShape(chrono.ChVisualShapeBox(chrono.ChVector3d(2, 2, 0.5)))
sys.Add(chassis)

# Engine
engine = chrono.ChBody()
engine.SetMass(10)
engine.SetVisualShape(chrono.ChVisualShapeSphere(0.5))
sys.Add(engine)

# Transmission
transmission = chrono.ChBody()
transmission.SetMass(5)
transmission.SetVisualShape(chrono.ChVisualShapeBox(chrono.ChVector3d(1, 1, 1)))
sys.Add(transmission)

# Suspension and Wheels
# Left suspension
suspension = chrono.ChBody()
suspension.SetMass(2)
suspension.SetVisualShape(chrono.ChVisualShapeCylinder(0.3, 0.5))
sys.Add(suspension)

# Wheel (left)
wheel_left = chrono.ChBody()
wheel_left.SetMass(10)
wheel_left.SetVisualShape(chrono.ChVisualShapeSphere(0.5))
wheel_left.SetPos(chrono.ChVector3d(-1, 0, 0))
wheel_left.EnableCollision(True)
sys.Add(wheel_left)

# Create suspension arm joint
joint_susp = chrono.ChLinkLockRevolute()
joint_susp.Initialize(chassis, suspension, chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(math.pi / 2)))
sys.Add(joint_susp)

# Right suspension
suspension_r = chrono.ChBody()
suspension_r.SetMass(2)
suspension_r.SetVisualShape(chrono.ChVisualShapeCylinder(0.3, 0.5))
sys.Add(suspension_r)

# Wheel (right)
wheel_right = chrono.ChBody()
wheel_right.SetMass(10)
wheel_right.SetVisualShape(chrono.ChVisualShapeSphere(0.5))
wheel_right.SetPos(chrono.ChVector3d(1, 0, 0))
wheel_right.EnableCollision(True)
sys.Add(wheel_right)

# Create right suspension arm joint
joint_susp_r = chrono.ChLinkLockRevolute()
joint_susp_r.Initialize(chassis, suspension_r, chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(math.pi / 2)))
sys.Add(joint_susp_r)

# Transmission to engine
joint_trans = chrono.ChLinkLockPrismatic()
joint_trans.Initialize(chassis, engine, chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(math.pi / 2)))
sys.Add(joint_trans)

# Transmission to suspension
joint_trans_susp = chrono.ChLinkLockPrismatic()
joint_trans_susp.Initialize(engine, suspension, chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(math.pi / 2)))
sys.Add(joint_trans_susp)

# Driver system
driver_pos = chrono.ChVector3d(0, 0, 0)
driver = veh.RigidDriver(sys, driver_pos, True, True)
driver.SetDriverControls(True, 0, 0)  # Initial controls: no steering, full throttle, no brake

# Visualization setup
# Chassis
chassis_vis = chrono.ChVisualShapeBox(chrono.ChVector3d(2, 2, 0.5))
chassis_vis.SetColor(chrono.ChColor(1, 0.5, 0))
chassis.SetVisualShape(chassis_vis)

# Engine
engine_vis = chrono.ChVisualShapeSphere(0.5)
engine_vis.SetColor(chrono.ChColor(0, 1, 0))
engine.SetVisualShape(engine_vis)

# Transmission
transmission_vis = chrono.ChVisualShapeBox(chrono.ChVector3d(1, 1, 1))
transmission_vis.SetColor(chrono.ChColor(0, 0, 1))
transmission.SetVisualShape(transmission_vis)

# Suspension and Wheels
suspension_vis = chrono.ChVisualShapeCylinder(0.3, 0.5)
suspension_vis.SetColor(chrono.ChColor(0, 0.5, 0))
suspension.SetVisualShape(suspension_vis)

wheel_vis_left = chrono.ChVisualShapeSphere(0.5)
wheel_vis_left.SetColor(chrono.ChColor(1, 0, 0))
wheel_left.SetVisualShape(wheel_vis_left)

wheel_vis_right = chrono.ChVisualShapeSphere(0.5)
wheel_vis_right.SetColor(chrono.ChColor(1, 0, 0))
wheel_right.SetVisualShape(wheel_vis_right)

# Terrain
terrain = veh.RigidTerrain(sys)
terrainMaterial = terrain.GetMaterial()
terrainMaterial.SetTexture(chrono.GetChronoDataFile('terrain_texture.png'))
terrainMaterial.SetRecomputeMesh(True)
terrain.AddPatch(
    patchMaterial=terrainMaterial,
    patchCoords=chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(math.pi)),
    patchLength=10,
    patchWidth=10
)
terrain.Initialize()

# Update camera position based on vehicle position and orientation
def update_camera():
    global cam_pos, cam_rot
    vehicle_pos = vehicle.GetPosition()
    vehicle_rot = vehicle.GetOrientation()
    cam_pos = vehicle_pos + chrono.ChVector3d(0, 5, 5)
    cam_rot = chrono.QuatFromAngleX(math.pi / 2) * vehicle_rot

# Simulation loop
t = 0
target_frame = 1000  # 50 FPS
target_time = 0.01

while True:
    t += target_time
    if t >= target_frame:
        t = 0
        sys.DoStepDynamics(target_time)
        update_camera()
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
    if not vis.Run():
        break

# Cleanup
sys.Dump()
vis.Destroy()