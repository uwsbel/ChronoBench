import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math as m
import os
import math




def main() : 
    

    

    
    kraz = veh.Kraz()
    kraz.SetContactMethod(chrono.ChContactMethod_SMC)
    kraz.SetChassisCollisionType(veh.CollisionType_NONE)
    kraz.SetChassisFixed(False) 
    kraz.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(-10, -0.2, 0.5), chrono.ChQuaterniond(1, 0, 0, 0)))
    kraz.SetEngineType(veh.EngineModelType_SHAFTS);
    kraz.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS);
    kraz.SetDriveType(veh.DrivelineTypeWV_RWD)
    kraz.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
    kraz.SetTireType(veh.TireModelType_TMEASY)
    kraz.SetTireStepSize(tire_step_size)
    kraz.Initialize()

    kraz.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    kraz.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    kraz.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    kraz.SetWheelVisualizationType(veh.VisualizationType_NONE)
    kraz.SetTireVisualizationType(veh.VisualizationType_MESH)

    kraz.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    

    terrain = veh.RigidTerrain(kraz.GetSystem())
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
    vis.SetWindowTitle('KRAZ')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 0.75), 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AttachVehicle(kraz.GetVehicle())

    
    driver = veh.ChInteractiveDriverIRR(vis)

    
    steering_time = 1.0  
    throttle_time = 1.0  
    braking_time = 0.3   
    driver.SetSteeringDelta(10)
    driver.SetThrottleDelta(0.2)
    driver.SetBrakingDelta(0.2)
    driver.SetSteeringTime(steering_time)
    driver.SetThrottleTime(throttle_time)
    driver.SetBrakingTime(braking_time)
    driver.Initialize()

    

    
    render_steps = m.ceil(1.0 / irrigt_step_size)

    
    step_number = 0

    kraz.GetVehicle().EnableRealtime(True)

    while vis.Run() :

        time = kraz.GetSystem().GetChTime()

        
        if (time >= t_end):
            break

        
        if (step_number % render_steps == 0) :
            vis.Render()
            vis.Advance(step_size)
            driver.Advance(step_size)

        
        driver_inputs = driver.GetInputs()
        kraz.Synchronize(time, driver_inputs)
        terrain.Synchronize(time)
        driver.Synchronize(time, driver_inputs)
        vis.Synchronize(time, driver_inputs)

        
        driver.Advance(step_size)
        terrain.Advance(step_size)
        kraz.Advance(step_size)
        vis.Advance(step_size)

        
        

        
        
        
        
        
        
        
        
        
        
        

        
        step_number += 1

    return 0








veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVector3d(-10, -0.2, 0.5)


chassis_vis_type = veh.VisualizationType_PRIMITIVES


chassis_collision_type = veh.CollisionType_NONE


tire_model = veh.TireModelType_TMEASY


terrainHeight = 0;      
terrainLength = 100.0;  
terrainWidth = 100.0;   


trackPoint = chrono.ChVector3d(0.0, 0.0, 1.75)


contact_method = chrono.ChContactMethod_SMC


step_size = 1e-3;
tire_step_size = 1e-3;


t_end = 1000;


irrigt_step_size = 1.0 / 50;  


main()