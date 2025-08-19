import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# =============================================================================

#print("Copyright (c) 2017 projectchrono.org\nChrono version: ", chrono.CHRONO_VERSION , "\n\n")

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(-15, 0, 1.2)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_RIGID

# Rigid terrain
terrainHeight = 0      # terrain height
terrainLength = 100.0  # size in X direction
terrainWidth = 100.0   # size in Y direction

# Poon chassis tracked by the camera
trackPoint = chrono.ChVector3d(0.0, 0.0, 1.71)

# Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# Create the HMMWV vehicle, set parameters, and initialize
vehicle = veh.HMMWV_Full()
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

# Create the rigid terrain
patch_mat = veh.RigidTerrain.RigidTerrainMaterialNs(0.01, 0.9)
patch = veh.RigidTerrain.RigidTerrainPatchIrregularHeight(chrono.GetChronoDataFile('terrain/rigid_plane.txt'), patch_mat, 75e3, 20)
terrain = veh.RigidTerrain(vehicle.GetSystem())
terrain.AddPatch(patch)
terrain.Initialize()

# Set terrain visualization modes
terrain.SetPlotType(veh.RigidTerrain.PLOT_PRESSURE, 0, core.ChColor(0,0,0))
patch.SetColor(veh.RigidTerrain.PLOT_PRESSURE, core.ChColor(0,0,0))
patch.SetColor(veh.RigidTerrain.PLOT_HEIGHT, core.ChColor(0,0,0))

# Create the vehicle Irrlicht interface
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Create and initialize the vehicle Irrlicht interface
vis.BindAll()

# Create the interactive driver system
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

# Number of simulation steps between miscellaneous events
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation frame counter and rendering frame count
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

# ---------------------------------------------
# Create a text file to save output data
try:
    # Create a text bulletin
    mbuf = open('output/data.txt', 'w')
except:
    print('Error creating file.')
    exit(13)

# ---------------
# Simulation loop
# ---------------

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

    # Advance simulation for one timestep for all modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Update modules and check for errors
    if not(vehicle.Debug() ):
        print( "Error in vehicle. Aborting.")
        break

    # Resume (in case of window focus gains/losses, etc.)
    vis.Resume()

    # Print vehicle output (if provided in vehicle CSV file)
    if (step_number % 100 == 0) :
        print( "Time = {0:.3f}s OUTPUT:\n".format(time) + vehicle.GetVehicle().GetOutput(true))

    # Write output data
    if (step_number % 1 == 0) :
        veh_data = vehicle.GetVehicle().GetState()

        # Access and print various vehicle states
        mbuf.write( str(time) + "\t" +
                    str(veh_data.m_pos[0]) + " " + str(veh_data.m_pos[1]) + " " + str(veh_data.m_pos[2]) + " " +
                    str(veh_data.m_rot.GetX1()) + " " + str(veh_data.m_rot.GetY1()) + " " + str(veh_data.m_rot.GetZ1()) + " " +
                    str(veh_data.m_rot.GetX2()) + " " + str(veh_data.m_rot.GetY2()) + " " + str(veh_data.m_rot.GetZ2()) + " " +
                    str(veh_data.m_rot.GetX3()) + " " + str(veh_data.m_rot.GetY3()) + " " + str(veh_data.m_rot.GetZ3()) + " " +
                    str(veh_data.m_posdot[0]) + " " + str(veh_data.m_posdot[1]) + " " + str(veh_data.m_posdot[2]) + " " +
                    str(veh_data.m_w[0]) + " " + str(veh_data.m_w[1]) + " " + str(veh_data.m_w[2]) + "\n" )

    # Increment frame number
    step_number += 1

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)

# -----------------
# End simulation
# -----------------

print( "End simulation \n")
mbuf.close()
print("error happened with only start ```python")