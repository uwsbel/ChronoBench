import pychrono as chrono
import pychrono.robot as robot
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math as m

# =============================================================================

# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with: 
#chrono.SetChronoDataPath('path/to/data')

veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(0, 0, 1.6)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
chassis_vis_type = veh.VisualizationType_MESH
suspension_vis_type = veh.VisualizationType_PRIMITIVES
steering_vis_type = veh.VisualizationType_PRIMITIVES
wheel_vis_type = veh.VisualizationType_MESH

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_TMEASY

# Poon chassis tracked by the camera
trackPoint = chrono.ChVector3d(0.0, 0.0, 1.75)

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Simulation end time
tend = 15

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# Output directories
out_dir = "./CURROVER"

# =============================================================================


print( "Copyright (c) 2017 projectchrono.org\n")

# --------------
# Create systems
# --------------

# Create the CURVEROVER vehicle, set parameters, and initialize
curverover = veh.CURVEROVER()
curverover.SetContactMethod(chrono.ChContactMethod_NSC)
curverover.SetChassisFixed(False)
curverover.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
curverover.SetTireType(tire_model)
curverover.SetTireStepSize(tire_step_size)
curverover.SetInitFwdVel(0.0)
curverover.Initialize()

curverover.SetChassisVisualizationType(chassis_vis_type)
curverover.SetSuspensionVisualizationType(suspension_vis_type)
curverover.SetSteeringVisualizationType(steering_vis_type)
curverover.SetWheelVisualizationType(wheel_vis_type)

suspF = curverover.GetVehicle().GetSuspension(0)
suspR = curverover.GetVehicle().GetSuspension(1)
springFL = suspF.GetSpring(veh.LEFT)
shockFL = suspF.GetShock(veh.LEFT)
springFR = suspF.GetSpring(veh.RIGHT)
shockFR = suspF.GetShock(veh.RIGHT)
springRL = suspR.GetSpring(veh.LEFT)
shockRL = suspR.GetShock(veh.LEFT)
springRR = suspR.GetSpring(veh.RIGHT)
shockRR = suspR.GetShock(veh.RIGHT)

print(curverover)

# ------------------
# Create the terrain
# ------------------

terrain = veh.RigidTerrain(curverover.GetSystem())
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, 
                         chrono.CSYSNORM, 
                         200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 1.0))
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
terrain.Initialize()

# -------------------------------------
# Create the vehicle Irrlicht interface
# Create the driver system
# -------------------------------------

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('curverover')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(curverover.GetVehicle())

driver = veh.ChInteractiveDriverIRR(vis)

# Set the time response for steering and throttle keyboard inputs.
steering_time = 1.0  # time to go from 0 to +1 (or from 0 to -1)
throttle_time = 1.0  # time to go from 0 to +1
braking_time = 0.3   # time to go from 0 to +1
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()

# ---------------------------
# Initialize output
# ---------------------------
#if not chrono.ChGetLog() == "none":
#    out_file = curverover.GetVehicle().GetName()
#    print("Writing output to {}".format(out_dir + "/" + out_file + ".csv"))
#    curverover.GetVehicle().ExportComponentList()
#    curverover.GetVehicle().SetChassisOutput(True)
#    curverover.GetVehicle().SetOutput(veh.COV_OUTPUT_TYPE_CHRONO)
#    curverover.GetVehicle().SetOutputFile(out_dir + "/" + out_file + ".csv")

# ---------------
# Simulation loop
# ---------------

render_steps = m.ceil(render_step_size / step_size)

# Output substep counter
step_number = 0

curverover.GetVehicle().EnableRealtime(True)

while vis.Run() :
    time = curverover.GetSystem().GetChTime()

    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Get driver inputs
    driver_inputs = driver.GetInputs()

    # Update modules (process inputs from other modules)
    driver.Synchronize(time)
    terrain.Synchronize(time)
    curverover.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance simulation for one timestep for all modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    curverover.Advance(step_size)
    vis.Advance(step_size)

    # Increment frame number
    step_number += 1