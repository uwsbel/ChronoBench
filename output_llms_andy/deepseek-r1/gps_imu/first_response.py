import os
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import pychrono.sensor as sens

def main():
    # 1. Initialize PyChrono environment
    chrono.SetChronoDataPath(os.path.join(os.environ.get('CHRONO_DATA_DIR', ''), 'data/'))
    system = chrono.ChSystemNSC()
    system.SetGravity(chrono.ChVectorD(0, 0, -9.81))

    # 2. Create HMMWV vehicle
    vehicle = veh.HMMWV_Full(system)
    vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1.0)))
    vehicle.Initialize()
    vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)

    # 3. Create terrain
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChMaterialSurfaceNSC()
    patch = terrain.AddPatch(patch_mat, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 1), 100, 100)
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    terrain.Initialize()

    # 4. Create driver system
    driver = veh.ChInteractiveDriverIRR(vehicle.GetVehicle())
    driver.SetSteeringDelta(0.02)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.06)
    driver.Initialize()

    # 5. Create Irrlicht visualization
    vis = irr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowTitle('HMMWV Simulation')
    vis.SetWindowSize(1280, 720)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVectorD(6, 3, 1.5), chrono.ChVectorD(0, 0, 1))
    vis.AddTypicalLights()
    vehicle.GetVehicle().AddVisualizationAssets(vis)

    # 6. Create and attach sensors
    manager = sens.ChSensorManager(system)
    manager.scene.AddPointLight(chrono.ChVectorF(100, 100, 100), chrono.ChColor(1, 1, 1), 5000)

    # IMU sensor attached to chassis
    imu_offset = chrono.ChVectorD(0, 0, 0.5)
    imu_rot = chrono.ChQuaternionD(1, 0, 0, 0)
    imu = sens.ChAccelerometerSensor(
        vehicle.GetChassisBody(),          # body to attach to
        50,                                # update rate [Hz]
        chrono.ChFrameD(imu_offset, imu_rot)  # offset pose
    )
    imu.SetName("IMU")
    imu.PushFilter(sens.ChFilterAccelAccess())
    manager.AddSensor(imu)

    # GPS sensor attached to chassis
    gps_offset = chrono.ChVectorD(-1.5, 0, 1.0)
    gps_rot = chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 0, 1))
    gps = sens.ChGPSSensor(
        vehicle.GetChassisBody(),          # body to attach to
        10,                                # update rate [Hz]
        chrono.ChFrameD(gps_offset, gps_rot) # offset pose
    )
    gps.SetName("GPS")
    gps.PushFilter(sens.ChFilterGPSAccess())
    manager.AddSensor(gps)

    # 7. Simulation loop
    step_size = 0.002
    while vis.Run():
        time = system.GetChTime()

        # Update driver inputs
        driver.Synchronize(time)

        # Update vehicle systems
        vehicle.Synchronize(time, driver.GetInputs(), terrain)
        terrain.Synchronize(time)

        # Advance simulation
        manager.Update()
        system.DoStepDynamics(step_size)

        # Update visualization
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        # Output vehicle mass (chassis only for simplicity)
        if abs(time % 0.1) < step_size:
            print(f"Time: {time:.2f}s - Vehicle Mass: {vehicle.GetVehicle().GetMass():.1f} kg")

if __name__ == "__main__":
    main()