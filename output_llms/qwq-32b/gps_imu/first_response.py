import pychrono as chrono
import pychrono.vehicle as vehicle
import pychrono.irrlicht as chronoirr

def main():
    # Initialize core system
    my_system = chrono.ChSystemNSC()
    my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

    # Setup Irrlicht visualization
    app = chronoirr.ChIrrApp(my_system, 'HMMWV Simulation', chronoirr.dimension2du(1024,768))
    app.SetSymbolscale(0.01)
    app.Initialize()
    app.AddTypicalSky()
    app.AddTypicalLogo()
    app.AddTypicalCamera(chrono.ChVectorD(5, 5, 5), chrono.ChVectorD(0, 0, 0))
    app.AddTypicalLights()

    # Create HMMWV vehicle
    hmmwv = vehicle.HMMWV()
    hmmwv.Initialize(my_system)
    hmmwv.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1), chrono.QUNIT))
    app.Add(hmmwv.GetVehicle())

    # Create terrain
    terrain = vehicle.RigidTerrain(my_system)
    patch_mat = chrono.ChMaterialSurfaceNSC()
    patch_mat.SetFriction(0.9)
    terrain.Initialize(vehicle.GetDataFile("prowheel/soil.dat"), patch_mat, True, 0.1)
    app.Add(terrain.GetGroundBody())

    # Setup driver with constant throttle
    driver = vehicle.HMMWVDriver()
    driver.Initialize(hmmwv.GetVehicle())
    driver.SetThrottle(0.5)

    # IMU sensor attached to chassis
    imu = chrono.ChIMU()
    imu.SetBody(hmmwv.GetChassis())
    hmmwv.AddSensor(imu)

    # Simulation parameters
    step_size = 0.01
    time = 0.0
    print(f"Vehicle mass: {hmmwv.GetVehicleMass():.2f} kg")

    # Main simulation loop
    while app.GetDevice().run():
        time = my_system.GetChTime()

        # Update driver and vehicle state
        driver.Synchronize(time)
        hmmwv.Synchronize(time, driver)
        hmmwv.Advance(time + step_size)

        # Update sensors
        imu_data = imu.GetReading()
        chassis_pos = hmmwv.GetChassis().GetPos()

        # Rendering and synchronization
        app.BeginScene()
        app.DrawAll()
        app.EndScene()

        # Advance physics system
        my_system.DoStepDynamics(step_size)

    app.GetDevice().closeGraphicalWindow()

if __name__ == '__main__':
    main()