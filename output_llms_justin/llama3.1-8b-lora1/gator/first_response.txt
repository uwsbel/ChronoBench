import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

def main():
    #print("Copyright (c) 2017 projectchrono.org\nChrono version: ", CHRONO_VERSION , "\n\n")

    #  Create the Gator vehicle, set parameters, and initialize
    gator = veh.Gator()
    gator.SetContactMethod(contact_method)
    gator.SetChassisFixed(False) 
    gator.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
    gator.SetEngineType(veh.EngineModelType_SIMPLE_MAP)
    gator.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
    gator.SetTireType(tire_model)
    gator.SetTireStepSize(tire_step_size)
    gator.Initialize()

    gator.SetChassisVisualizationType(chassis_vis_type)
    gator.SetSuspensionVisualizationType(suspension_vis_type)
    gator.SetSteeringVisualizationType(steering_vis_type)
    gator.SetWheelVisualizationType(wheel_vis_type)
    gator.SetTireVisualizationType(tire_vis_type)

    gator.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    # Create the terrain
    terrain = veh.RigidTerrain(gator.GetSystem())
    if (contact_method == chrono.ChContactMethod_NSC):
        patch_mat = chrono.ChContactMaterialNSC()
        patch_mat.SetFriction(0.9)
        patch_mat.SetRestitution(0.01)
    elif (contact_method == chrono.ChContactMethod_SMC):
        patch_mat = chrono.ChContactMaterialSMC()
        patch_mat.SetFriction(0.9)
        patch_mat.SetRestitution(0.01)
        patch_mat.SetYoungModulus(2e7)
    patch = terrain.AddPatch(patch_mat, 
                             chrono.CSYSNORM, 
                             terrainLength, terrainWidth)
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    # Create the vehicle Irrlicht interface
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('Gator')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(trackPoint, 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AttachVehicle(gator.GetVehicle())

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

    # This is schedule to occur after the vehicle is initialized
    gator.GetVehicle().EnableRealtime(True)

    # Simulation loop
    while vis.Run() :
        time = gator.GetSystem().GetChTime()

        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        # Get driver inputs
        driver_inputs = driver.GetInputs()

        # Update modules (process inputs from other modules)
        driver.Synchronize(time)
        terrain.Synchronize(time)
        gator.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        # Advance simulation for one timestep for all modules
        driver.Advance(render_step_size)
        terrain.Advance(render_step_size)
        gator.Advance(render_step_size)
        vis.Advance(render_step_size)

    return 0

# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with: 
#chrono.SetChronoDataPath('path/to/data')
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(0, 0, 1.6)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = chrono.ChCollisionType_PRIMITIVES

# Rigid terrain
terrainHeight = -0.2
terrainLength = 15
terrainWidth = 4.8

# Type of contact method (NSC, SCC, or SMC)
contact_method = chrono.ChContactMethod_SMC

# Type of tire model (RIGID, RIGID_MESH, FIALA, PAC89)
tire_model = veh.TMeasy

# Mesh visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
chassis_vis_type = veh.VisualizationType_PRIMITIVES
suspension_vis_type = veh.VisualizationType_PRIMITIVES
steering_vis_type = veh.VisualizationType_PRIMITIVES
wheel_vis_type = veh.VisualizationType_MESH
tire_vis_type = veh.VisualizationType_MESH

# Poonter location for the camera
trackPoint = chrono.ChVector3d(0.0, 0.0, 1.75)

# Simulation step sizes
step_size = 1e-3
tire_step_size = 1e-3
render_step_size = 1e-3

main()