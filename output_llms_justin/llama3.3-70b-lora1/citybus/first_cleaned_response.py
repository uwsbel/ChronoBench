import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math as m
import errno
import os
import math




def main() : 
    

    
    vehicle = veh.CityBus()
    vehicle.SetContactMethod(chrono.ChContactMethod_SMC)
    vehicle.SetChassisCollisionType(veh.CollisionType_NONE);
    vehicle.SetChassisFixed(False); 
    vehicle.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(-10, -2, 0.6), chrono.ChQuaterniond(1, 0, 0, 0)))
    vehicle.SetEngineType(veh.EngineModelType_SHAFTS);
    vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS);
    vehicle.SetDriveType(veh.DrivelineTypeWV_AWD)
    vehicle.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
    vehicle.SetTireType(veh.TireModelType_TMEASY)
    vehicle.SetTireStepSize(tire_step_size)
    vehicle.Initialize()

    
    terrain = veh.RigidTerrain(vehicle.GetSystem())
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    if (contact_method == chrono.ChContactMethod_NSC):
        patch_mat.SetYoungModulus(2e7)
    patch = terrain.AddPatch(patch_mat, 
                             chrono.CSYSNORM, 
                             200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 1.0))
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    terrain.Initialize()

    
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('CityBus')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AttachVehicle(vehicle.GetVehicle())

    
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

    
    while vis.Run() :
        time = vehicle.GetSystem().GetChTime()

        
        driver_inputs = driver.GetInputs()

        
        driver.Synchronize(time)
        terrain.Synchronize(time)
        vehicle.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        
        driver.Advance(step_size)
        terrain.Advance(step_size)
        vehicle.Advance(step_size)
        vis.Advance(step_size)

        
        p = vehicle.GetChassisBody().GetPos()
        q = vehicle.GetChassisBody().GetRot()
        vb = vehicle.GetChassisVelBody()
        print('\n' + str(time) + '  ' + str(p.x) + '  ' + str(p.y) + '  ' + str(p.z) +
              '  ' + str(q.x) + '  ' + str(q.y) + '  ' + str(q.z) + '  ' + str(q.w) +
              '  ' + str(vb.x) + '  ' + str(vb.y) + '  ' + str(vb.z) + '\n\n')

        if (vis.MyHandler):    
            vis.MyHandler()


    return 0








veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVector3d(-10, -2, 0.6)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


chassis_vis_type = veh.VisualizationType_MESH
suspension_vis_type = veh.VisualizationType_PRIMITIVES
steering_vis_type = veh.VisualizationType_PRIMITIVES
wheel_vis_type = veh.VisualizationType_MESH
tire_vis_type = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_NONE


tire_model = veh.TireModelType_TMEASY


terrainHeight = 0;      
terrainLength = 100.0;  
terrainWidth = 100.0;   


trackPoint = chrono.ChVector3d(0.0, 0.0, 1.75)


contact_method = chrono.ChContactMethod_SMC


step_size = 1e-3;
tire_step_size = 1e-3;

main()