import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math as m
import errno
import os
import math


#// =============================================================================

def main() : 
    #print("Copyright (c) 2017 projectchrono.org\nChrono version: ", CHRONO_VERSION , "\n\n")

    # Create the vehicle system
    vehicle = veh.BMW_E90_Sedan()
    vehicle.SetContactMethod(chrono.ChContactMethod_SMC)
    vehicle.SetChassisFixed(False);
    vehicle.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(-10, -2, 0.6),chrono.QUNIT))
    vehicle.SetEngineType(veh.EngineModelType_SIMPLE)
    vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
    vehicle.SetDriveType(veh.DrivelineTypeWV_RWD)
    vehicle.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
    vehicle.SetTireType(veh.TireModelType_TMEASY)
    vehicle.SetTireStepSize(tire_step_size)
    vehicle.Initialize()

    vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    vehicle.EnableRealtime(True)

    # Create the vehicle Irrlicht interface
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('BMW E90 Sedan')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AttachVehicle(vehicle.GetVehicle())

    # Create the interactive driver system
    driver = veh.ChInteractiveDriverIRR(vis)
    driver.SetSteeringDelta(0.06)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.06)
    driver.Initialize()

    vehicle.GetVehicle().SetDriver(driver.GetDriver())

    # Generate terrain in the form of arc of a circle
    terrain = veh.RigidTerrain(vehicle.GetSystem())

    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)

    terrain.AddPatch(patch_mat, 
                     chrono.ChCoordsysd(chrono.ChVector3d(-10, 0, 0.4), chrono.QUNIT), 
                     300, 60)
    terrain.Initialize()

    # Create the vehicle Irrlicht interface
    vis.AddTerrain(terrain);

    # Initialize simulation
    vehicle.Initialize()

    terrain.Initialize()

    # Simulation loop
    while vis.Run() :
        time = vehicle.GetSystem().GetChTime()

        vis.BeginScene()
        vis.Render()
        vis.EndScene()

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

# =============================================================================

# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with: 
#chrono.SetChronoDataPath('path/to/data')

veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Simulation step sizes
step_size = 3e-3;
tire_step_size = 1e-3;

main()