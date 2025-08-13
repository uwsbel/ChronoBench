import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

def main():
    #print("Copyright (c) 2017 projectchrono.org\nChrono version: ", CHRONO_VERSION , "\n\n")

    # Create the system
    sys = chrono.ChSystemNSC()
    sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0,0,-9.81))

    # Create the terrain
    terrain = veh.RigidTerrain(sys)
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 200, 100)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    terrain.Initialize()

    # Create the Viper rover
    rover = veh.Viper(sys)
    rover.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, -0.5, 1.0), chrono.QUNIT))
    rover.SetEngineType(veh.EngineModelType_SIMPLE)
    rover.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
    rover.SetDriveType(veh.DrivelineTypeWV_AWD)
    rover.SetTireType(veh.TireModelType_RIGID)
    rover.SetTireStepSize(1e-3)
    rover.Initialize()

    # Create the driver system
    driver = veh.ChDriver(rover.GetVehicle())
    driver.Initialize()

    # Create the vehicle Irrlicht interface
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('Viper rover')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 0.75), 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AttachVehicle(rover.GetVehicle())

    # Driver interactive system
    vis.EnableRealtime(True)

    # Set the time response for steering and throttle keyboard inputs.
    steering_time = 10.0  # time to go from 0 to +1 (centiseconds)
    throttle_time = 10.0  # time to go from 0 to +1 (centiseconds)
    brake_time = 2.0      # time to go from 0 to +1 (centiseconds)
    driver.SetSteeringDelta(render_step_size / (2.0 * steering_time / 100.0))
    driver.SetThrottleDelta(render_step_size / (2.0 * throttle_time / 100.0))
    driver.SetBrakeDelta(render_step_size / (2.0 * brake_time / 100.0))

    # --- Render settings ---

    # Set application timer
    t1 = chrono.ChSimulationTime(0)

    while vis.Run() :
        time = rover.GetVehicle().GetChTime()

        # Draw scene
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        # Update modules (process inputs from other modules)
        driver.Synchronize(time)
        terrain.Synchronize(time)
        rover.Synchronize(time, driver, terrain)
        vis.Synchronize(time, driver)

        # Advance simulation for one timestep for all modules
        driver.Advance(step_size)
        terrain.Advance(step_size)
        rover.Advance(step_size)
        vis.Advance(step_size)

        # Advance time for entire system
        sys.Advance(step_size)

        # Get the current time of the system
        t = rover.GetVehicle().GetChTime()

        # Check for driver termination
        if (driver.WasInterrupted()) :
            break

        # Render step size
        render_step_size = 1e-3

# Main
main()