import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math as m

# =============================================================================

def main():
    #print("Copyright (c) 2017 projectchrono.org\nChrono version: ", CHRONO_VERSION , "\n\n")

    #  Create the various modules of the vehicle simulation system

    # Create the vehicle, set parameters, and initialize
    driver = veh.ChDriver()
    vehicle = veh.ChTruckFEDA()
    vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
    vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
    vehicle.SetChassisFixed(False) 
    vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
    vehicle.SetEngineType(veh.EngineModelType_SHAFTS)
    vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
    vehicle.SetTireType(tire_model)
    vehicle.SetTireStepSize(tire_step_size)
    vehicle.Initialize()

    vehicle.SetChassisVisualizationType(chassis_vis_type)
    vehicle.SetSuspensionVisualizationType(suspension_vis_type)
    vehicle.SetSteeringVisualizationType(steering_vis_type)
    vehicle.SetWheelVisualizationType(wheel_vis_type)
    vehicle.SetTireVisualizationType(tire_vis_type)

    vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    # Create the terrain

    terrain = veh.ChTerrain(vehicle.GetSystem(), patch_mat)
    patch1 = terrain.AddPatch(patch_mat, 
                              chrono.CSYSNORM, 
                              terrainLength, terrainWidth)
    patch1.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    patch1.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    terrain.Initialize()

    # Create the vehicle Irrlicht interface
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('FEDA')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(trackPoint, 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AttachVehicle(vehicle)

    # Create the interactive driver system
    driver.Initialize()

    # Simulation loop
    
    vehicle.EnableRealtime(True)

    while vis.Run() :
        time = vehicle.GetSystem().GetChTime()

        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        # Get driver inputs
        driver_inputs = driver.GetInputs()

        # Update modules (process inputs from other modules)
        driver.Synchronize(time)
        vehicle.Synchronize(time, driver_inputs, terrain)
        terrain.Synchronize(time)
        vis.Synchronize(time, driver_inputs)

        # Advance simulation for one timestep for all modules
        driver.Advance(step_size)
        vehicle.Advance(step_size)
        terrain.Advance(step_size)
        vis.Advance(step_size)

    return 0


# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with: 
#chrono.SetChronoDataPath('path/to/data')

veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
chassis_vis_type = veh.VisualizationType_PRIMITIVES
suspension_vis_type =  veh.VisualizationType_PRIMITIVES
steering_vis_type = veh.VisualizationType_PRIMITIVES
wheel_vis_type = veh.VisualizationType_MESH
tire_vis_type = veh.VisualizationType_MESH 

# Type of tire model (RIGID, RIGID_MESH, FIALA, PAC89)
tire_model = veh.TireModelType_TMEASY

# Patch material for terrain
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

# Terrain dimensions
terrainHeight = 0 Julien      # terrain height (FLAT terrain only)
terrainLength = 16.0        # size in X direction
terrainWidth = 8.0          # size in Y direction

# Point on chassis tracked by the camera
trackPoint = chrono.ChVector3d(0.0, 0.0, 1.75)

# Simulation step sizes
step_size = 1e-3;
tire_step_size = 1e-3;

main()