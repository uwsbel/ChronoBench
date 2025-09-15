import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import pychrono.sensor as sens

def main():
    
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())
    veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")
    
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

    
    vehicle = veh.HMMWV_Reduced()
    vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
    vehicle.SetChassisCollisionType(veh.CollisionType_MESH)
    vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0)))
    vehicle.Initialize()
    vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)

    
    terrain = veh.RigidTerrain(system)
    patch = terrain.AddPatch(chrono.ChCoordsysD(), 200, 100, 0.1)
    patch.SetTexture(veh.TerrainTextureType_GRADED, 200, 100)
    terrain.Initialize()

    
    app = irr.ChIrrApp(vehicle.GetSystem(), "HMMWV Simulation", irr.dimension2d(1280, 720))
    app.AddTypicalLights()
    app.AddTypicalCamera(irr.vector3df(0, 2, -5))
    app.AssetBindAll()
    app.AssetUpdateAll()

    
    manager = sens.ChSensorManager(system)
    manager.scene.AddPointLight(chrono.ChVectorD(0, 100, 0), chrono.ChColor(1,1,1), 1000)

    
    imu = sens.ChIMUSensor(
        vehicle.GetChassisBody(),             
        50,                                   
        chrono.ChFrameD(chrono.ChVectorD(0, 0.5, 0)),  
        sens.NoiseNormal(chrono.ChVectorD(0.001,0.001,0.001), chrono.ChVectorD(0.001,0.001,0.001))
    )
    manager.AddSensor(imu)

    
    gps = sens.ChGPSSensor(
        vehicle.GetChassisBody(),             
        50,                                   
        chrono.ChFrameD(chrono.ChVectorD(0, 0.5, 0)),  
        chrono.ChVectorD(0, 0, 0),            
        sens.NoiseNormal(chrono.ChVectorD(0.1,0.1,0.1))
    )
    manager.AddSensor(gps)

    
    driver = veh.ChInteractiveDriverIRR(app.GetDevice())
    driver.SetSteeringDelta(0.02)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.06)
    vehicle.GetVehicle().SetDriver(driver)

    
    print(f"Vehicle mass: {vehicle.GetVehicle().GetMass()} kg")

    
    time_step = 0.02
    while app.GetDevice().run():
        time = system.GetChTime()
        
        
        app.BeginScene()
        app.DrawAll()
        app.EndScene()
        
        
        driver.Synchronize(time)
        vehicle.Synchronize(time, driver.GetInputs(), terrain)
        terrain.Synchronize(time)
        app.Synchronize()
        app.Advance(time_step)
        
        
        system.DoStepDynamics(time_step)
        manager.Update()

        
        if imu.GetMostRecentFrame():
            print(f"IMU Accel: {imu.GetMostRecentFrame().Acceleration}")
        if gps.GetMostRecentFrame():
            print(f"GPS Position: {gps.GetMostRecentFrame().Position}")

if __name__ == "__main__":
    main()