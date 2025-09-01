import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math as m



def main():
    

    
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(contact_method)
    hmmwv.SetChassisFixed(False) 
    hmmwv.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
    hmmwv.SetEngineType(engine_model)
    hmmwv.SetTransmissionType(transmission_model)
    hmmwv.SetDriveType(drive_type)
    hmmwv.SetSteeringType(steering_type)
    hmmwv.SetTireType(tire_model)
    hmmwv.Initialize()

    
    terrain = veh.RigidTerrain(hmmwv.GetSystem())
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch = terrain.AddPatch(patch_mat, 
                             chrono.CSYSNORM, 
                             terrainLength, 0, 
                             terrainWidth, terrainWidth)
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('HMMWV')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(trackPoint, 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AttachVehicle(hmmwv.GetVehicle())

    
    driver = veh.ChInteractiveDriverIRR(vis)

    
    steering_time = 1.0  
    throttle_time = 1.0  
    braking_time = 0.3   
    driver.SetSteeringDelta(render_step_size / steering_time)
    driver.SetThrottleDelta(render_step_size / throttle_time)
    driver.SetBrakingDelta(render_step_size / braking_time)

    driver.Initialize()

    
    
    time = 0.0

    
    while vis.Run() :
        time = hmmwv.GetSystem().GetChTime()

        
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        
        driver_inputs = driver.GetInputs()

        
        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        
        driver.Advance(render_step_size)
        terrain.Advance(render_step_size)
        hmmwv.Advance(render_step_size)
        vis.Advance(render_step_size)

    return 0







veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVector3d(0, 0, 1.6)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


chassis_collision_type = veh.COLL_NONE


trackPoint = chrono.ChVector3d(0.0, 0.0, 1.75) 
render_step_size = 1e-3


steering_time = 1.0  
throttle_time = 1.0  
braking_time = 0.3   


contact_method = chrono.ChContactMethod_SMC
contact_vis = False


engine_model = veh.EngineModelType_SIMPLE


transmission_model = veh.TransmissionModelType_AUTOMATIC_SHAFTS


drive_type = veh.DrivelineTypeWV_Full
steering_type = veh.SteeringTypeWV_PITMAN_ARM


tire_model = veh.TireModelType_TMEASY


terrainHeight = 0;      
terrainLength = 100.0;  
terrainWidth = 100.0;   
terrainPatchMat = True; 



main()