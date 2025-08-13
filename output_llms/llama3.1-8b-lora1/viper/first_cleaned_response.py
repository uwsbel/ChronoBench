import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

def main():
    

    
    sys = chrono.ChSystemNSC()
    sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0,0,-9.81))

    
    terrain = veh.RigidTerrain(sys)
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 200, 100)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    terrain.Initialize()

    
    rover = veh.Viper(sys)
    rover.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, -0.5, 1.0), chrono.QUNIT))
    rover.SetEngineType(veh.EngineModelType_SIMPLE)
    rover.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
    rover.SetDriveType(veh.DrivelineTypeWV_AWD)
    rover.SetTireType(veh.TireModelType_RIGID)
    rover.SetTireStepSize(1e-3)
    rover.Initialize()

    
    driver = veh.ChDriver(rover.GetVehicle())
    driver.Initialize()

    
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('Viper rover')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 0.75), 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AttachVehicle(rover.GetVehicle())

    
    vis.EnableRealtime(True)

    
    steering_time = 10.0  
    throttle_time = 10.0  
    brake_time = 2.0      
    driver.SetSteeringDelta(render_step_size / (2.0 * steering_time / 100.0))
    driver.SetThrottleDelta(render_step_size / (2.0 * throttle_time / 100.0))
    driver.SetBrakeDelta(render_step_size / (2.0 * brake_time / 100.0))

    

    
    t1 = chrono.ChSimulationTime(0)

    while vis.Run() :
        time = rover.GetVehicle().GetChTime()

        
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        
        driver.Synchronize(time)
        terrain.Synchronize(time)
        rover.Synchronize(time, driver, terrain)
        vis.Synchronize(time, driver)

        
        driver.Advance(step_size)
        terrain.Advance(step_size)
        rover.Advance(step_size)
        vis.Advance(step_size)

        
        sys.Advance(step_size)

        
        t = rover.GetVehicle().GetChTime()

        
        if (driver.WasInterrupted()) :
            break

        
        render_step_size = 1e-3


main()