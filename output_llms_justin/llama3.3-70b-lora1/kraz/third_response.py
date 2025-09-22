import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# =============================================================================

# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with: 
#chrono.SetChronoDataPath('path/to/data')

veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc1 = chrono.ChVector3d(0, 0.1, 0.5)
initRot1 = chrono.ChQuaterniond(1, 0, 0, 0)

initLoc2 = chrono.ChVector3d(-12, -4, 1.0)
initRot2 = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_RIGID

# Rigid terrain
terrainHeight = 0      # terrain height
terrainLength = 200.0  # size in X direction
terrainWidth = 200.0   # size in Y direction

# Poon chassis tracked by the camera
trackPoint = chrono.ChVector3d(0,0, 0.6)

# Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# Update rate in Hz for interactive driver
update_rate = 100

# =============================================================================

# Create the kraz vehicle, set parameters, and initialize
vehicle1 = veh.Kraz()
vehicle1.SetContactMethod(contact_method)
vehicle1.SetChassisCollisionType(chassis_collision_type)
vehicle1.SetChassisFixed(False)
vehicle1.SetInitPosition(chrono.ChCoordsysd(initLoc1, initRot1))
vehicle1.Initialize()

vehicle1.SetChassisVisualizationType(vis_type)
vehicle1.SetSteeringVisualizationType(vis_type)
vehicle1.SetSuspensionVisualizationType(vis_type, vis_type)
vehicle1.SetWheelVisualizationType(vis_type, vis_type)
vehicle1.SetTireVisualizationType(vis_type, vis_type)

vehicle1.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

print("Vehicle mass:          " + str(vehicle1.GetVehicle().GetMass()))
# =============================================================================

# Create the terrain

terrain1 = veh.RigidTerrain(vehicle1.GetSystem())
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain1.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), terrainLength, terrainWidth)
terrain1.Initialize()

# =============================================================================

# Create the Interactive Driver module
driver1 = veh.ChInteractiveDriverIRR(vis)

# Set the time response for steering and throttle keyboard inputs.
steering_time = 1.0  # time to go from 0 to +1 (or from 0 to -1)
throttle_time = 1.0  # time to go from 0 to +1
braking_time = 0.3   # time to go from 0 to +1
driver1.SetSteeringDelta(render_step_size / steering_time)
driver1.SetThrottleDelta(render_step_size / throttle_time)
driver1.SetBrakingDelta(render_step_size / braking_time)

driver1.Initialize()

# =============================================================================

print( "VEHICLE MASS: ",  vehicle1.GetVehicle().GetMass())

# Create the first vehicle Irrlicht interface
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Kraz')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 25.0, 1.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle1.GetVehicle())

# =============================================================================

# Create the car vehicle, set parameters, and initialize
vehicle2 = veh.HMMWV()
vehicle2.SetContactMethod(contact_method)
vehicle2.SetChassisCollisionType(chassis_collision_type)
vehicle2.SetChassisFixed(False)
vehicle2.SetInitPosition(chrono.ChCoordsysd(initLoc2, initRot2))
vehicle2.Initialize()

vehicle2.SetChassisVisualizationType(vis_type)
vehicle2.SetSteeringVisualizationType(vis_type)
vehicle2.SetSuspensionVisualizationType(vis_type, vis_type)
vehicle2.SetWheelVisualizationType(vis_type, vis_type)
vehicle2.SetTireVisualizationType(vis_type, vis_type)

vehicle2.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

print("Vehicle mass:          " + str(vehicle2.GetVehicle().GetMass()))

# =============================================================================

# Create the second terrain

terrain2 = veh.RigidTerrain(vehicle2.GetSystem())
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain2.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), terrainLength, terrainWidth)
terrain2.Initialize()

# =============================================================================

# Create the driver
steering = 0
throttle = 0.1
braking = 0.0
# =============================================================================

# Simulation loop

# Number of simulation steps between miscellaneous events
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation frame counter s
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

vehicle1_state = veh.vector_EntryXME()
vehicle2_state = veh.vector_EntryXME()

max_time = 300

# =============================================================================

while vis.Run() :
    time1 = vehicle1.GetSystem().GetChTime()

    if (time1 > max_time) : break

    # Render scene and output POV-Ray data
    if (step_number % render_steps == 0) :
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Get driver inputs
    driver_inputs1 = driver1.GetInputs()

    # Update modules (process inputs from other modules)
    driver1.Synchronize(time1, driver_inputs1, terrain1)
    terrain1.Synchronize(time1)
    vehicle1.Synchronize(time1, driver_inputs1, terrain1)
    vis.Synchronize(time1, driver_inputs1)

    # Advance simulation for one timestep for all modules
    driver1.Advance(step_size)
    terrain1.Advance(step_size)
    vehicle1.Advance(step_size)
    vis.Advance(step_size)

    time2 = vehicle2.GetSystem().GetChTime()

    if (time2 > max_time) : break

    # Update modules (process inputs from other modules)
    driver_inputs2 = veh.DriverInputs()
    driver_inputs2.m_steering = steering
    driver_inputs2.m_throttle = throttle
    driver_inputs2.m_braking = braking
    terrain2.Synchronize(time2)
    vehicle2.Synchronize(time2, driver_inputs2, terrain2)
    driver_inputs2.m_steering = 0 # no steering

    # Advance simulation for one timestep for all modules
    terrain2.Advance(step_size)
    vehicle2.Advance(step_size)

    # Store the state at current time
    vehicle1_state.push_back(veh.EntryXME(chrono.ChTime(time1), vehicle1.GetChassisBody().GetPos(), vehicle1.GetChassisBody().GetRot()))
    vehicle2_state.push_back(veh.EntryXME(chrono.ChTime(time2), vehicle2.GetChassisBody().GetPos(), vehicle2.GetChassisBody().GetRot()))

    # Increment frame number
    step_number += 1

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)

# =============================================================================

print("Sim time:", time1)
print("FPS:", render_frame / time1)
print("Real time:", realtime_timer.GetStepCount() * step_size)

# =============================================================================

# Save output to JSON files
import os
import json

out_dir = "output/"
if not os.path.isdir(out_dir) :
    os.makedirs(out_dir)

output1 = {
    'time': [],
    'pos': [],
    'rot': []
}

output2 = {
    'time': [],
    'pos': [],
    'rot': []
}

for entry in vehicle1_state :
    output1['time'].append(entry.time)
    output1['pos'].append([entry.pos.x, entry.pos.y, entry.pos.z])
    output1['rot'].append([entry.rot.e00, entry.rot.e01, entry.rot.e02, entry.rot.e10, entry.rot.e11, entry.rot.e12, entry.rot.e20, entry.rot.e21, entry.rot.e22])

for entry in vehicle2_state :
    output2['time'].append(entry.time)
    output2['pos'].append([entry.pos.x, entry.pos.y, entry.pos.z])
    output2['rot'].append([entry.rot.e00, entry.rot.e01, entry.rot.e02, entry.rot.e10, entry.rot.e11, entry.rot.e12, entry.rot.e20, entry.rot.e21, entry.rot.e22])

with open(out_dir + 'vehicle1.json', 'w') as f1, open(out_dir + 'vehicle2.json', 'w') as f2 :
    json.dump(output1, f1, indent=4)
    json.dump(output2, f2, indent=4)