import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math as m
import os
import errno
# =============================================================================

# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with: 
#chrono.SetChronoDataPath('path/to/data')

veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
chassis_vis_type = veh.VisualizationType_MESH
suspension_vis_type = veh.VisualizationType_PRIMITIVES
steering_vis_type = veh.VisualizationType_PRIMITIVES
wheel_vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_TMEASY

# Rigid terrain
terrainHeight = 0      # terrain height (FLAT terrain only)
terrainLength = 200.0  # size in X direction
terrainWidth = 200.0   # size in Y direction

# Poon chassis tracked by the camera
trackPoint = chrono.ChVector3d(0.0, 0.0, 0.2)

# Contact method
contact_method = chrono.ChContactMethod_SMC
contact_vis = False

# Simulation step sizes
step_size = 3e-3
tire_step_size = step_size

# Simulation end time
t_end = 1000

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# =============================================================================


print( "Copyright (c) 2017 projectchrono.org\n")

# --------------
# Create systems
# --------------

# Create the ARTcar vehicle, set parameters, and initialize
car = veh.ARTcar()
car.SetContactMethod(contact_method)
car.SetChassisCollisionType(chassis_collision_type)
car.SetChassisFixed(False)
car.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
car.SetTireType(tire_model)
car.SetTireStepSize(tire_step_size)
car.SetMaxMotorVoltageRatio(0.16)
car.SetStallTorque(0.3)
car.SetTireRollingResistance(0.06)

car.Initialize()

tire_vis_type = veh.VisualizationType_MESH  # : VisualizationType::PRIMITIVES

car.SetChassisVisualizationType(chassis_vis_type)
car.SetSuspensionVisualizationType(suspension_vis_type)
car.SetSteeringVisualizationType(steering_vis_type)
car.SetWheelVisualizationType(wheel_vis_type)
car.SetTireVisualizationType(tire_vis_type)

car.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create the terrain
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(car.GetSystem())
patch = terrain.AddPatch(patch_mat, 
                         chrono.CSYSNORM, 
                         terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Create the vehicle Irrlicht interface
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('dart')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(car.GetVehicle())

# Create the driver system
driver = veh.ChInteractiveDriverIRR(vis)

# Set the time response for steering and throttle keyboard inputs.
steering_time = 1.0  # time to go from 0 to +1 (or from 0 to -1)
throttle_time = 1.0  # time to go from 0 to +1
braking_time = 0.3   # time to go from 0 to +1
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()

# ---------------
# Simulation loop
# ---------------

# output vehicle mass
print( "VEHICLE MASS:      " + str(car.GetVehicle().GetMass()))
print( "DRIVELINE TYPE:    " + car.GetVehicle().GetDriveline().GetTemplateName())
print( "TIRE TYPE:         " + car.GetVehicle().GetTire(0, veh.LEFT).GetTemplateName())

realtime_timer = chrono.ChRealtimeStepTimer()
while vis.Run() :
    time = car.GetSystem().GetChTime()

    # Render scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Get driver inputs
    driver_inputs = driver.GetInputs()

    # Update modules (process inputs from other modules)
    driver.Synchronize(time)
    terrain.Synchronize(time)
    car.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance simulation for one timestep for all modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    car.Advance(step_size)
    vis.Advance(step_size)

    # Roll the string for at least 2 seconds
    if (time > t_end) : 
        break