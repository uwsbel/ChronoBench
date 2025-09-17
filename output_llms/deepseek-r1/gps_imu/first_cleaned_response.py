import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import pychrono.sensor as sens

def main():
    
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVectorD(0, 0, -9.81))

    
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
    hmmwv.SetChassisFixed(False)
    hmmwv.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1.0), chrono.QUNIT))
    hmmwv.SetTireType(veh.TireModelType_RIGID)
    hmmwv.Initialize()
    hmmwv.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv.SetWheelVisualizationType(veh.VisualizationType_NONE)
    hmmwv.SetTireVisualizationType(veh.VisualizationType_PRIMITIVES)
    system.Add(hmmwv.GetSystem())

    
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChMaterialSurfaceNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysD(), 100, 100)
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    
    driver = veh.ChInteractiveDriverIRR(hmmwv.GetVehicle())
    driver.SetSteeringDelta(0.02)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.06)
    driver.Initialize()

    
    manager = sens.ChSensorManager(system)
    manager.scene.AddPointLight(chrono.ChVectorF(100, 100, 100), chrono.ChColor(1, 1, 1), 5000)
    
    
    imu_offset = chrono.ChVectorD(0, 0, 0.5)
    imu_rot = chrono.ChQuaternionD(1, 0, 0, 0)
    imu = sens.ChAccelerometerSensor(
        hmmwv.GetChassisBody(),  
        50,                     
        chrono.ChFrameD(imu_offset, imu_rot),  
    )
    imu.SetName("IMU")
    imu.PushFilter(sens.ChFilterAccelAccess())
    manager.AddSensor(imu)

    
    gps_offset = chrono.ChVectorD(-1.5, 0, 0.5)
    gps = sens.ChGPSSensor(
        hmmwv.GetChassisBody(),  
        10,                     
        chrono.ChFrameD(gps_offset, imu_rot),  
    )
    gps.SetName("GPS")
    gps.PushFilter(sens.ChFilterGPSAccess())
    manager.AddSensor(gps)

    
    vis = irr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("HMMWV Simulation")
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVectorD(6, 3, 1.5), chrono.ChVectorD(0, 0, 0.5))
    vis.AddTypicalLights()

    
    step_size = 0.002
    frame = 0
    print(f"Vehicle mass: {hmmwv.GetVehicle().GetMass()} kg")

    while vis.Run():
        time = system.GetChTime()
        
        
        driver_inputs = driver.GetInputs()
        driver.Synchronize(time)
        
        
        hmmwv.Synchronize(time, driver_inputs, terrain)
        terrain.Synchronize(time)
        
        
        manager.Update()
        driver.Advance(step_size)
        hmmwv.Advance(step_size)
        terrain.Advance(step_size)
        vis.Advance(step_size)
        vis.Render()
        system.DoStepDynamics(step_size)
        
        
        if frame % 100 == 0:
            imu_data = imu.GetMostRecentAccelBuffer()
            gps_data = gps.GetMostRecentGPSBuffer()
            if imu_data.HasAcceleration():
                print(f"IMU Accel: {imu_data.Acceleration}")
            if gps_data.HasPosition():
                print(f"GPS Position: {gps_data.Position}")
        
        frame += 1

if __name__ == "__main__":
    main()