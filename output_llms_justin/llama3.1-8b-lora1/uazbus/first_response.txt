import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

def main():
    #print("Copyright (c) 2017 projectchrono.org\nChrono version: ", CHRONO_VERSION , "\n\n")

    #  Create the UAZBUS vehicle, set parameters, and initialize
    uaz = veh.UAZBUS()
    uaz.SetContactMethod(contact_method)
    uaz.SetChassisCollisionType(chassis_collision_type)
    uaz.SetChassisFixed(False) 
    uaz.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
    uaz.SetEngineType(veh.EngineModelType_SIMPLE_MAP)
    uaz.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
    uaz.SetDriveType(veh.DrivelineTypeWV_AWD)
    uaz.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
    uaz.SetTireType(tire_model)
    uaz.SetTireStepSize(tire_step_size)
    uaz.Initialize()

    uaz.SetChassisVisualizationType(chassis_vis_type)
    uaz.SetSuspensionVisualizationType(suspension_vis_type)
    uaz.SetSteeringVisualizationType(steering_vis_type)
    uaz.SetWheelVisualizationType(wheel_vis_type)
    uaz.SetTireVisualizationType(tire_vis_type)

    uaz.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    # Create the terrain
    terrain = veh.RigidTerrain(uaz.GetSystem())
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch = terrain.AddPatch(patch_mat, 
                             chrono.CSYSNORM, 
                             200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 1.0))
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    terrain.Initialize()

    # Create the vehicle Irrlicht interface
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('UAZBUS')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 0.75), 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AttachVehicle(uaz.GetVehicle())

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

    # Set the simulation timestep
    step_size = uaz.GetSystem().GetSolver().GetStepSize()

    # Number of simulation timesteps between two render frames.
    render_steps = int(render_step_size / step_size)

    # Simulation loop
    while vis.Run() :
        driver_inputs = driver.GetInputs()

        # Update vehicle
        uaz.GetVehicle().SetEngineForce(0.0)
        uaz.GetVehicle().SetDrivelineInput(0, driver_inputs.m_steering)
        uaz.GetVehicle().SetDrivelineInput(1, driver_inputs.m_throttle)
        uaz.GetVehicle().SetDrivelineInput(2, driver_inputs.m_braking)
        uaz.Update(step_size)

        # Update terrain
        terrain.Update(step_size)

        # Update visual system
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        # Advance driver system for internal processes (sensing, actuation)
        driver.Synchronize(step_size)

        # Advance the state of the entire system (processes for all modules)
        sys = uaz.GetSystem()
        sys.DoStepDynamics(step_size)

    return 0

# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with: 
#chrono.SetChronoDataPath('path/to/data')

veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(0, 0, 0.4)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Collision type for chassis
chassis_collision_type = veh.CollisionType_NONE

# Type of contact method
contact_method = chrono.ChContactMethod_NSC

# Rigid terrain
patch_step_size = 1e-3

# Visualization types for vehicle parts
chassis_vis_type = veh.VisualizationType_MESH
suspension_vis_type =  veh.VisualizationType_PRIMITIVES
steering_vis_type = veh.VisualizationType_PRIMITIVES
wheel_vis_type = veh.VisualizationType_MESH
tire_vis_type = veh.VisualizationType_MESH

# Vehicle tire type
tire_model = veh.TireModelType_TMEASY

# Simulation step size
step_size = 3e-3

# Time interval for two render frames
render_step_size = 1.0 / 50  # FPS = 50

main()