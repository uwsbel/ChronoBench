import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math as m
import errno
import os
import math




def main():
    print("Copyright (c) 2017 projectchrono.org" + "\n\n")

    

    
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

    

    
    terrain = veh.SCMTerrain(hmmwv.GetSystem())
    if (contact_method == chrono.ChContactMethod_NSC):
        patch_mat = chrono.ChContactMaterialNSC()
        patch_mat.SetFriction(0.9)
        patch_mat.SetRestitution(0.01)
    elif (contact_method == chrono.ChContactMethod_SMC):
        patch_mat = chrono.ChContactMaterialSMC()
        patch_mat.SetFriction(0.9)
        patch_mat.SetRestitution(0.01)
        patch_mat.SetYoungModulus(2e7)
    patch_mat.SetFriction(0.9)
    patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM,
                             round(terrainLength / mesh_length), round(terrainWidth / mesh_length))
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 20, 20)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))

    
    terrain.SetGrain(0.02, 0.00, 0.02);
    terrain.SetRod(0.02, 0.00, 0.02)
    
    terrain.SetXmin(-200.0)
    terrain.SetXmax(200.0)
    terrain.SetYmin(-100.0)
    terrain.SetYmax(300.0)

    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    

    
    height = terrain.GetHeight()
    for i in range(-200, 200, 1):
        for j in range(-100, 300, 1):
            
            zh = 0
            if (abs(i) < 75):
                if (j > 100):
                    zh = -8 + (j - 100) / 12.0
                if (j < -50):
                    zh = +10 - (j + 50) / 7.0
            height.AddPoint(i, j, zh)

    terrain.Initialize()

    
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('HMMWV-SCM')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 0.2), 6.0, 0.5)
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

    

    
    render_steps = m.ceil(render_step_size / step_size)
    debug_steps = m.ceil(debug_step_size / step_size)

    
    step_number = 0
    render_frame = 0

    if (contact_vis):
        vis.SetSymbolscale(1.0)
    
    realtime_timer = chrono.ChRealtimeStepTimer()
    while vis.Run() :
        time = hmmwv.GetSystem().GetChTime()

        
        if (step_number % render_steps == 0) :
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

        
        step_number += 1

        
        realtime_timer.Spin(step_size)

    return 0







veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVector3d(0, 0, 1.1)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


chassis_vis_type = veh.VisualizationType_MESH
suspension_vis_type = veh.VisualizationType_PRIMITIVES
steering_vis_type = veh.VisualizationType_PRIMITIVES
wheel_vis_type = veh.VisualizationType_MESH
tire_vis_type = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_NONE


terrainHeight = 0      
terrainLength = 100.0  
terrainWidth = 100.0   


trackPoint = chrono.ChVector3d(0.0, 0.0, 1.75)


contact_method = chrono.ChContactMethod_SMC
contact_vis = False


step_size = 3e-3;
tire_step_size = 1e-3;


render_step_size = 1.0 / 50;  


out_dir = os.path.join(os.path.dirname(__file__), "HMMWV")
out_dir1 = os.path.join(out_dir, "vehicle")
out_dir2 = os.path.join(out_dir, "driver")


povray = True









veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVector3d(0, 0, 1.1)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


chassis_vis_type = veh.VisualizationType_MESH
suspension_vis_type = veh.VisualizationType_PRIMITIVES
steering_vis_type = veh.VisualizationType_PRIMITIVES
wheel_vis_type = veh.VisualizationType_MESH
tire_vis_type = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_NONE


terrainHeight = 0      
terrainLength = 100.0  
terrainWidth = 100.0   


trackPoint = chrono.ChVector3d(0.0, 0.0, 1.75)


contact_method = chrono.ChContactMethod_SMC
contact_vis = False


step_size = 3e-3;
tire_step_size = 1e-3;


render_step_size = 1.0 / 50;  


out_dir = os.path.join(os.path.dirname(__file__), "HMMWV")
out_dir1 = os.path.join(out_dir, "vehicle")
out_dir2 = os.path.join(out_dir, "driver")


povray = True


engine_model = veh.EngineModelType_SHAFTS

drive_type = veh.DrivelineTypeWV_AWD

braking_type = veh.BrakingTypeWV_PROPORTIONAL

diff_type = veh.DifferentialTypeWV_OPEN

transmission_model = veh.TransmissionModelType_AUTOMATIC
steering_type = veh.SteeringTypeWV_PITMAN_ARM

tire_model = veh.TireModelType_TMEASY


json_spec = veh.GetDataFile("full_car/vehicle/HMMWV_JSON.json")


main()