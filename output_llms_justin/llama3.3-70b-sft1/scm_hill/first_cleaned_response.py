import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens
import math
import os




def main():
    

    

    
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(contact_method)
    hmmwv.SetChassisCollisionType(chassis_collision_type)
    hmmwv.SetChassisFixed(False) 
    hmmwv.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
    hmmwv.SetEngineType(engine_model)
    hmmwv.SetTransmissionType(transmission_model)
    hmmwv.SetDriveType(drive_type)
    hmmwv.SetSteeringType(steering_type)
    hmmwv.SetTireType(tire_model)
    hmmwv.SetTireStepSize(tire_step_size)
    hmmwv.Initialize()

    hmmwv.SetChassisVisualizationType(chassis_vis_type)
    hmmwv.SetSuspensionVisualizationType(suspension_vis_type)
    hmmwv.SetSteeringVisualizationType(steering_vis_type)
    hmmwv.SetWheelVisualizationType(wheel_vis_type)
    hmmwv.SetTireVisualizationType(tire_vis_type)

    hmmwv.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    
    terrain = veh.SCMTerrain(hmmwv.GetSystem())
    terrain.SetSoilParameters(2e6,   
                              0,     
                              1.1,   
                              0,     
                              30,    
                              0.01,  
                              2e8,   
                              3e4    
    )

    
    terrain.AddMovingPatch(hmmwv.GetChassisBody(), 
                           chrono.ChVector3d(0, 0, 0), 
                           terrainLength,      
                           terrainWidth,       
                           delta                
    )

    
    terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0, 0.1)

    
    terrain.Initialize(terrainLength, terrainWidth, delta, initLoc + chrono.ChVector3d(0, 0, -hmmwv.GetVehicle().GetGroundClearance() - 0.1))

    
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('HMMWV Deformable Soil Demo')
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

    
    hmmwv.GetVehicle().EnableRealtime(True)

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

        
        driver.Advance(step_size)
        terrain.Advance(step_size)
        hmmwv.Advance(step_size)
        vis.Advance(step_size)

    return 0







veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVector3d(0, 0, 1.6)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


chassis_vis_type = veh.VisualizationType_MESH
suspension_vis_type =  veh.VisualizationType_PRIMITIVES
steering_vis_type = veh.VisualizationType_PRIMITIVES
wheel_vis_type = veh.VisualizationType_MESH
tire_vis_type = veh.VisualizationType_MESH 


chassis_collision_type = veh.CollisionType_NONE


engine_model = veh.EngineModelType_SHAFTS
transmission_model = veh.TransmissionModelType_AUTOMATIC_SHAFTS


drive_type = veh.DrivelineTypeWV_AWD


steering_type = veh.SteeringTypeWV_PITMAN_ARM


tire_model = veh.TireModelType_TMEASY







terrainHeight = 0;      
terrainLength = 20.0;   
terrainWidth = 20.0;    
delta = 0.05;            


trackPoint = chrono.ChVector3d(0.0, 0.0, 1.75)


contact_method = chrono.ChContactMethod_SMC
contact_vis = False


step_size = 2e-3;
tire_step_size = 1e-3;


render_step_size = 1.0 / 50;  



main()