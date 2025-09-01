import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with: 
#chrono.SetChronoDataPath('path/to/data')

# If true, use provided callback to change camera vertical zoom
change_camera_height = True

# Look at definition of main entry point for this test
def main():
    #print("Copyright (c) 2017 projectchrono.org\nChrono version: ", CHRONO_VERSION , "\n\n")

    # Create the vehicle system
    vehicle = veh.RigidChassisVehicle(chassis_file, None)

    # Create and initialize the vehicle tires
    for ia in vehicle.GetTireAssemblies():
        tire = ia.GetTire()
        # Initialize the tire mesh visualization
        vis = veh.VisualizationType_NONE
        if tire.HasVisualizationMesh():
            vis = veh.VisualizationType_MESH

        # Set tire pressure (default: 32kPa)
        tire.SetPressure(35000)
        # Set unauthorized visualization type
        tire.SetVisualizationType(vis)
        # Initialize tire
        vehicle.InitializeTire(ia.GetId(), veh.PointContactMethod_HARD)

    # Create and initialize the powertrain systems
    engine = veh.ReadEngineJSON(engine_file)
    transmission = veh.ReadTransmissionJSON(transmission_file)
    powertrain = veh.ChPowertrainAssembly(engine, transmission)
    vehicle.InitializePowertrain(powertrain, veh.ThrottleControlType_NONE)

    vehicle.GetVehicle().SetChassisCollisionType(veh.CollisionType_NONE);
    vehicle.GetVehicle().SetChassisFixed(False);
    vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET);

    vehicle.Initialize()

    # Set the initial vehicle position
    vehicle.GetChassisBody().Translate(chrono.ChVector3d(-75, 0, 0.5))
    vehicle.GetChassisBody().Rotate(chrono.ChQuaternion1d(1, 0, 0, 0))
    vehicle.GetChassisBody().SetPos(chrono.ChVector3d(-75, 0, 1.6))
    vehicle.GetChassisBody().SetRot(chrono.ChQuaternion1d(1, 0, 0, 0))

    # Set the engine speed and gear in the powertrain
    vehicle.SetThrottle(0.1)
    vehicle.SetGear(1)
    vehicle.GetVehicle().EnableRealtime(True)

    # Create the ground
    terrain = veh.RigidTerrain(vehicle.GetSystem())
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    terrain.AddPatch(patch_mat, 
                     chrono.CSYSNORM, 
                     200, 200)
    terrain.Initialize()

    # Create the vehicle Irrlicht interface
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('ChronicARGo: ARGo (single track, Rigid Chassis)')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 0.75), 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AttachVehicle(vehicle.GetVehicle())

    # Create the interactive driver system
    driver = veh.ChInteractiveDriverIRR(vis)
    driver.SetSteeringDelta(0.02)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.06)
    driver.Initialize()

    # Simulation loop
    while vis.Run() :
        # Render scene
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        # Collect output from modules (for interactive apps)
        driver_inputs = driver.GetInputs()

        # Update modules (for interactive apps)
        time = vehicle.GetSystem().GetChTime()
        driver.Synchronize(time)
        vehicle.Synchronize(time, driver_inputs)
        terrain.Synchronize(time)
        vis.Synchronize(time, driver_inputs)

        # Advance simulation for one timestep for all modules
        driver.Advance(step_size)
        vehicle.Advance(step_size)
        terrain.Advance(step_size)
        vis.Advance(step_size)

if __name__ == "__main__":
    main()