import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

def main():
    

    

    
    m113 = veh.M113()
    m113.SetContactMethod(chrono.ChContactMethod_NSC)
    m113.SetChassisCollisionType(veh.CollisionType_NONE)
    m113.SetChassisFixed(False) 
    m113.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(-75, -35, 1.6), chrono.ChQuaterniond(1, 0, 0, 0)))
    m113.SetEngineType(veh.EngineModelType_SIMPLE)
    m113.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
    m113.SetDriveType(veh.DrivelineTypeWV_AWD)
    m113.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
    m113.Initialize()

    m113.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    m113.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    m113.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    m113.SetWheelVisualizationType(veh.VisualizationType_NONE)
    m113.SetTireVisualizationType(veh.VisualizationType_PRIMITIVES)

    m113.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    

    terrain = veh.RigidTerrain(m113.GetSystem())
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
    vis.SetWindowTitle('M113')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AttachVehicle(m113.GetVehicle())

    
    driver = veh.ChInteractiveDriverIRR(vis)

    
    steering_time = 1.0  
    throttle_time = 1.0  
    braking_time = 0.3   
    driver.SetSteeringDelta(render_step_size / steering_time)
    driver.SetThrottleDelta(render_step_size / throttle_time)
    driver.SetBrakingDelta(render_step_size / braking_time)

    driver.Initialize()

    
    m113.GetVehicle().EnableRealtime(True)

    while vis.Run() :
        time = m113.GetSystem().GetChTime()

        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        
        driver_inputs = driver.GetInputs()

        
        driver.Synchronize(time)
        terrain.Synchronize(time)
        m113.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        
        driver.Advance(step_size)
        terrain.Advance(step_size)
        m113.Advance(step_size)
        vis.Advance(step_size)

    return 0






veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVector3d(-75, -35, 1.6)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

step_size = 1e-3
render_step_size = step_size

main()