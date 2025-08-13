import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

def main():
    

    
    kraz = veh.Kraz()
    kraz.SetContactMethod(chrono.ChContactMethod_NSC)
    kraz.SetChassisCollisionType(veh.CollisionType_NONE)
    kraz.SetChassisFixed(False) 
    kraz.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(-10, -1100, 0.5), chrono.QUNIT))
    kraz.SetEngineType(veh.EngineModelType_SIMPLE)
    kraz.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
    kraz.SetDriveType(veh.DrivelineTypeWV_AWD)
    kraz.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
    kraz.SetTireType(veh.TireModelType_TMEASY)
    kraz.SetTireStepSize(1e-3)
    kraz.Initialize()

    kraz.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    kraz.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    kraz.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    kraz.SetWheelVisualizationType(veh.VisualizationType_NONE)
    kraz.SetTireVisualizationType(veh.VisualizationType_PRIMITIVES)

    kraz.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    
    terrain = veh.RigidTerrain(kraz.GetSystem())
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch = terrain.AddPatch(patch_mat, 
                             chrono.CSYSNORM, 
                             200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 1.0))
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    terrain.Initialize()

    
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('Kraz')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, -1.0, 0.0), 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AttachVehicle(kraz.GetVehicle())

    
    driver = veh.ChInteractiveDriverIRR(vis)

    
    steering_time = 1.0  
    throttle_time = 1.0  
    braking_time = 0.3   
    driver.SetSteeringDelta(10 * steering_time)
    driver.SetThrottleDelta(10 * throttle_time)
    driver.SetBrakingDelta(10 * braking_time)

    driver.Initialize()

    

    
    
    

    
    kraz.Initialize()

    terrain.Initialize()

    
    driver.Initialize()

    

    vis.Initialize()

    
    step_size = 1e-3
    time_end = 100

    
    vis.SetChronoSystem(kraz.GetSystem())

    
    driver.SetVehicle(kraz.GetVehicle())

    
    while vis.Run() :
        time = kraz.GetChassis().GetPos().x

        
        driver.Synchronize(time)
        terrain.Synchronize(time)
        kraz.Synchronize(time, step_size, driver)
        vis.Synchronize(time, step_size)

        
        driver.Advance(step_size)
        terrain.Advance(step_size)
        kraz.Advance(step_size)
        vis.Advance(step_size)

    return 0






main()