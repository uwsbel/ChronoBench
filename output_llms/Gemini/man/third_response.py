import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens
import math
import numpy as np

# -----------------------------------------------------------------------------------
#  Create and initialize the vehicle system
# -----------------------------------------------------------------------------------

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_TMEASY

# Rigid terrain
terrainHeight = 0      # terrain height
terrainLength = 100.0  # size in X direction
terrainWidth = 100.0   # size in Y direction

# Poon chassis tracked by the camera
trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)

# Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# Create the MAN vehicle, set parameters, and initialize
vehicle = veh.MAN_10t() 
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)
vehicle.Initialize()

vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# -----------------------------------------------------------------------------------
#  Create the terrain and add random boxes
# -----------------------------------------------------------------------------------

# Create the terrain
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat, 
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 
    terrainLength, terrainWidth)

# **Change texture to "grass.jpg"**
patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Create and add random boxes
box_material = chrono.ChMaterialSurfaceNSC()
box_material.SetFriction(0.9)
box_material.SetRestitution(0.01)

n_boxes = 10
for i in range(n_boxes):
    size = chrono.ChVector3d(chrono.ChRandom() * 2 + 0.5, chrono.ChRandom() * 2 + 0.5, chrono.ChRandom() * 2 + 0.5)
    pos = chrono.ChVector3d(chrono.ChRandom() * 80 - 40, chrono.ChRandom() * 80 - 40, size.z / 2 + 0.1)
    rot = chrono.ChQuaterniond(chrono.ChRandom(), chrono.ChRandom(), chrono.ChRandom(), chrono.ChRandom()).GetNormalized()

    box = chrono.ChBodyEasyBox(size.x, size.y, size.z, 1000, True, True, box_material)
    box.SetPos(pos)
    box.SetRot(rot)
    vehicle.GetSystem().Add(box)

# -----------------------------------------------------------------------------------
#  Create the vehicle Irrlicht interface
# -----------------------------------------------------------------------------------

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('MAN 10t Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 15.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# -----------------------------------------------------------------------------------
#  Create the driver system
# -----------------------------------------------------------------------------------

driver = veh.ChInteractiveDriverIRR(vis)

# Set the time response for steering and throttle keyboard inputs.
steering_time = 1.0  # time to go from 0 to +1 (or from 0 to -1)
throttle_time = 1.0  # time to go from 0 to +1
braking_time = 0.3   # time to go from 0 to +1
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()

# -----------------------------------------------------------------------------------
#  Create the sensor manager and lidar sensor
# -----------------------------------------------------------------------------------

manager = sens.ChSensorManager(vehicle.GetSystem())
manager.SetVerbose(False)
manager.SetUpdateFrequency(50)

# Create a lidar sensor and add it to the sensor manager
lidar = sens.ChLidarSensor(
    vehicle.GetChassis(),
    chrono.ChFrameD(chrono.ChVectorD(0, 0, 2.0)),
    sens.ChLidarSensor.VerticalBeamLayout.BEAM_LAYOUT_VERTICAL,
    512, # horizontal samples
    16,  # vertical beams
    chrono.CH_C_PI,
    100.0,
    -1.0,
    +3.0,
)
lidar.SetName("Lidar")
lidar.SetVisualisation(True)
lidar.SetFilterBackpoints(True)
lidar.SetEnvelope(chrono.ChCollisionShapeSphere(0.1))
manager.AddSensor(lidar)

# -----------------------------------------------------------------------------------
#  Run the simulation loop
# -----------------------------------------------------------------------------------

# output vehicle mass
print( "VEHICLE MASS: ",  vehicle.GetVehicle().GetMass())

# Number of simulation steps between miscellaneous events
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation frame counter
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run() :
    time = vehicle.GetSystem().GetChTime()
    # Render scene and output POV-Ray data
    if (step_number % render_steps == 0) :
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1
    # Get driver inputs
    driver_inputs = driver.GetInputs()

    # Update modules (process inputs from other modules)
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)
    
    # **Update sensor manager**
    manager.Update()

    # Advance simulation for one timestep for all modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Increment frame number
    step_number += 1

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)