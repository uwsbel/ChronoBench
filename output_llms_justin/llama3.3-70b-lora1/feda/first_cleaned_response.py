import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math as m
import errno
import os
import math




def main() : 
    

    

    
    feda = veh.FEDA()
    feda.SetContactMethod(contact_method)
    feda.SetChassisCollisionType(chassis_collision_type)
    feda.SetChassisFixed(False) 
    feda.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
    feda.SetTireType(tire_model)
    feda.SetTireStepSize(tire_step_size)
    feda.Initialize()

    

    terrain = veh.RigidTerrain(feda.GetSystem())
    if (contact_method == chrono.ChContactMethod_NSC) :
        patch_mat = chrono.ChContactMaterialNSC()
        patch_mat.SetFriction(0.9)
        patch_mat.SetRestitution(0.01)
    elif (contact_method == chrono.ChContactMethod_SMC) :
        patch_mat = chrono.ChContactMaterialSMC()
        patch_mat.SetFriction(0.9)
        patch_mat.SetRestitution(0.01)
        patch_mat.SetYoungModulus(2e7)
    patch = terrain.AddPatch(patch_mat, 
                             chrono.CSYSNORM, 
                             300, 50)
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('FED-Alpha')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(trackPoint, 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AttachVehicle(feda.GetVehicle())

    
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

    

    
    render_steps = 1
    debug_steps = 1

    
    step_number = 0
    render_frame = 0

    if (contact_vis):
        vis.SetSymbolscale(1.0)
    if (terrain_vis):
        vis.BindTerrain(terrain);

    
    
    

    
    render_step_size = 1 / float(render_steps) / 60
    step = 0.0
    arc_length = 0

    realtime_timer = chrono.ChRealtimeStepTimer()
    while vis.Run() :
        time = feda.GetSystem().GetChTime()

        
        driver_inputs = driver.GetInputs()

        
        driver.Synchronize(time)
        terrain.Synchronize(time)
        feda.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        
        if (step % render_steps == 0) : 
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            render_frame += 1

        
        driver.Advance(step_size)
        terrain.Advance(step_size)
        feda.Advance(step_size)
        vis.Advance(step_size)

        
        step += 1

        
        realtime_timer.Spin(step_size)

    return 0







veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


chassis_vis_type = veh.VisualizationType_MESH
suspension_vis_type = veh.VisualizationType_PRIMITIVES
steering_vis_type = veh.VisualizationType_PRIMITIVES
wheel_vis_type = veh.VisualizationType_MESH
tire_vis_type = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_NONE


tire_model = veh.TireModelType_PAC02


terrainHeight = 0;      
terrainLength = 100.0;  
terrainWidth = 100.0;   


trackPoint = chrono.ChVector3d(0.0, 0.0, 1.75)


contact_method = chrono.ChContactMethod_SMC


step_size = 1e-3;
tire_step_size = 1e-3;


t_end = 1000;


render_step_size = 1.0 / 50;  


out_dir = os.path.join(os.path.dirname(__file__), "FEDAlpha_demo")
out_dir1 = os.path.join(out_dir, "test_1")
out_dir2 = os.path.join(out_dir, "test_2")

is_first_run = not os.path.isdir(out_dir)
if is_first_run:
    print("First run. Creating output directories:\n")
    print(" ").join(["  ", out_dir, out_dir1, out_dir2])
    os.mkdir(out_dir)
    os.mkdir(out_dir1)
    os.mkdir(out_dir2)
else:
    print("Output directories already exist.\n")


povray = False
if (povray) :
    vis.SetSymbolscale(0.01)
    vis.EnablePovray(output_dir)
    if contact_vis:
        vis.SetSymbolscale(1.0)


track_point = chrono.ChVector3d(2.5, 0, 1.9)
if (contact_vis) :
    vis.SetSymbolscale(1.0)
if (terrain_vis) :
    vis.BindTerrain(terrain);

main()