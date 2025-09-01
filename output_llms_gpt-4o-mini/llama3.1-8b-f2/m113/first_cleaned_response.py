import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math as m



def main():
    

    
    m113 = veh.M113()
    m113.SetContactMethod(chrono.ChContactMethod_NSC)
    m113.SetChassisFixed(False);
    m113.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
    m113.SetEngineType(veh.EngineModelType_SIMPLE)
    m113.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
    m113.SetDriveType(veh.DrivelineTypeWV_AWD)
    m113.SetTireType(tire_model)
    m113.SetTireStepSize(tire_step_size)
    m113.Initialize()

    m113.SetChassisVisualizationType(chassis_vis_type)
    m113.SetSuspensionVisualizationType(suspension_vis_type)
    m113.SetSteeringVisualizationType(steering_vis_type)
    m113.SetWheelVisualizationType(wheel_vis_type)
    m113.SetTireVisualizationType(tire_vis_type)

    m113.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    

    terrain = veh.RigidTerrain(m113.GetSystem())
    if (contact_method == chrono.ChContactMethod_NSC):
        patch_mat = chrono.ChContactMaterialNSC()
        patch_mat.SetFriction(0.9)
        patch_mat.SetRestitution(0.01)
    elif (contact_method == chrono.ChContactMethod_SMC):
        patch_mat = chrono.ChContactMaterialSMC()
        patch_mat.SetFriction(0.9)
        patch_mat.SetRestitution(0.01)
        patch_mat.SetYoungModulus(2e7)
    patch = terrain.AddPatch(patch_mat, 
                             chrono.CSYSNORM, 
                             terrainLength, terrainWidth)
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('M113')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(trackPoint, 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AttachVehicle(m113.GetVehicle())

    
    driver = veh.ChInteractiveDriverIRR(vis)

    
    steering_time = 1.0  
    throttle_time = 1.0  
    braking_time = 0.3   
    driver.SetSteeringDelta(10 * step_size / steering_time)
    driver.SetThrottleDelta(10 * step_size / throttle_time)
    driver.SetBrakingDelta(10 * step_size / braking_time)

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


initLoc = chrono.ChVector3d(0, 0, 1.1)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


chassis_vis_type = veh.VisualizationType_PRIMITIVES
suspension_vis_type =  veh.VisualizationType_PRIMITIVES
steering_vis_type = veh.VisualizationType_PRIMITIVES
wheel_vis_type = veh.VisualizationType_MESH
tire_vis_type = veh.VisualizationType_MESH 


tire_model = veh.TireModelType_TMEASY


terrainHeight = 0;      
terrainLength = 100.0;  
terrainWidth = 100.0;   


trackPoint = chrono.ChVector3d(0.0, 0.0, 0.0)


contact_method = chrono.ChContactMethod_SMC


step_size = 1e-3;
tire_step_size = 1e-3;

main()