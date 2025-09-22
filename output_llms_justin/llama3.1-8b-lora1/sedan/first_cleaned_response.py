import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr



def main():
    

    
    
    vis_params = veh.VisualizationParameters( mesh  
                                            | veh.CHASSIS_MESH 
                                            | veh.SUSPENSION_MESH 
                                            | veh.SHOCK_ABSORBER_MESH 
                                            | veh.WHEEL_MESH 
                                            | veh.TIRE_MESH 
                                            )
    
    e90 = veh.BMW_E90_Sedan()
    e90.SetContactMethod(chrono.ChContactMethod_NSC)
    e90.SetChassisFixed(False)
    e90.SetChassisVisualization(vis_params)
    e90.SetSuspensionVisualization(vis_params)
    e90.SetSteeringVisualization(vis_params)
    e90.SetWheelVisualization(vis_params)
    e90.SetTireVisualization(vis_params)
    e90.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
    e90.Initialize()
    e90.SetChassisCollisionType(veh.CollisionType_NONE)
    e90.SetChassisCollisionShapeType(veh.CollisionShapeType_BBOX)
    e90.SetChassisCollisionShapeDimensions(chrono.ChVector3d(4.8, 1.7, 1.0))
    e90.SetTireType(tire_model)
    e90.SetTireStepSize(tire_step_size)

    
    terrain = veh.RigidTerrain(e90.GetSystem())
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
    vis.SetWindowTitle('BMW E90')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AttachVehicle(e90.GetVehicle())

    
    driver = veh.ChInteractiveDriverIRR(vis)

    
    steering_time = 0.5  
    throttle_time = 1.0  
    braking_time = 0.3   
    driver.SetSteeringDelta(render_step_size / steering_time)
    driver.SetThrottleDelta(render_step_size / throttle_time)
    driver.SetBrakingDelta(render_step_size / braking_time)

    driver.Initialize()

    
    steering_time = 0.5  
    throttle_time = 1.0  
    braking_time = 0.3   
    driver.SetSteeringDelta(render_step_size / steering_time)
    driver.SetThrottleDelta(render_step_size / throttle_time)
    driver.SetBrakingDelta(render_step_size / braking_time)

    driver.Initialize()

    
    
    

    
    global_datapath = chrono.GetChronoDataPath() + 'vehicle/'

    
    if (out_pdf):
        print('Output will be written to the PDF output directory')
    if (out_ascii):
        print('Output will be written to the ASCII output directory')

    
    realtime_timer = chrono.ChRealtimeStepTimer()
    while vis.Run() :
        time = e90.GetVehicle().GetChassis().GetPos().x

        
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        
        driver_inputs = driver.GetInputs()

        
        driver.Synchronize(time)
        terrain.Synchronize(time)
        e90.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        
        driver.Advance(step_size)
        terrain.Advance(step_size)
        e90.Advance(step_size)
        vis.Advance(step_size)

        
        if (driver_inputs.m_steering > 0.1):
            driver.SetSteeringDelta(0)
        if (driver_inputs.m_throttle > 0.1):
            driver.SetThrottleDelta(0)
        if (driver_inputs.m_braking > 0.1):
            driver.SetBrakingDelta(0)

        
        if (out_pdf && time >= t_out_pdf[0] && time <= t_out_pdf[1]) :
            driver.WritePDF(output_dir_pdf, "driver_inputs", 2, out_pdf_steps)
            e90.WritePDF(output_dir_pdf, "chassis", 2, out_pdf_steps)
            terrain.WritePDF(output_dir_pdf, "terrain", 2, out_pdf_steps)
            vis.WritePDF(output_dir_pdf, "vehicle_vis", 2, out_pdf_steps)
            driver.Synchronize(time)
            terrain.Synchronize(time)
            e90.Synchronize(time, driver_inputs, terrain)
            vis.Synchronize(time, driver_inputs)
        if (out_ascii && time >= t_out_ascii[0] && time <= t_out_ascii[1]) :
            driver.WriteASCII(output_dir_ascii, "driver_inputs", out_ascii_steps)
            e90.WriteASCII(output_dir_ascii, "chassis", out_ascii_steps)
            terrain.WriteASCII(output_dir_ascii, "terrain", out_ascii_steps)
            vis.WriteASCII(output_dir_ascii, "vehicle_vis", out_ascii_steps)

        
        driver.Advance(step_size)
        terrain.Advance(step_size)
        e90.Advance(step_size)
        vis.Advance(step_size)

        
        if (driver_inputs.m_steering > 0.1):
            driver.SetSteeringDelta(0)
        if (driver_inputs.m_throttle > 0.1):
            driver.SetThrottleDelta(0)
        if (driver_inputs.m_braking > 0.1):
            driver.SetBrakingDelta(0)

        
        
        
        
        
        

        
        
        
        time = driver.GetTime()
        timeAsset = time * render_step_size

        
        my_system.GetChTime().SetTime(time)

        
        my_vis.Render()

        
        if (steering_inputs.m_steering > 0.1):
            steering_inputs.SetSteeringDelta(0)
        if (throttle_inputs.m_throttle > 0.1):
            throttle_inputs.SetThrottleDelta(0)
        if (braking_inputs.m_braking > 0.1):
            braking_inputs.SetBrakingDelta(0)

        
        my_system.GetChTime().SetTime(time)

        
        driver.Advance(step_size)
        terrain.Advance(step_size)
        e90.Advance(step_size)
        vis.Advance(step_size)

        
        if (out_pdf && time >= t_out_pdf[0] && time <= t_out_pdf[1]) :
            driver.WritePDF(output_dir_pdf, "driver_inputs", 2, out_pdf_steps)
            e90.WritePDF(output_dir_pdf, "chassis", 2, out_pdf_steps)
            terrain.WritePDF(output_dir_pdf, "terrain", 2, out_pdf_steps)
            vis.WritePDF(output_dir_pdf, "vehicle_vis", 2, out_pdf_steps)
            driver.Synchronize(time)
            terrain.Synchronize(time)
            e90.Synchronize(time, driver_inputs, terrain)
            vis.Synchronize(time, driver_inputs)
        if (out_ascii && time >= t_out_ascii[0] && time <= t_out_ascii[1]) :
            driver.WriteASCII(output_dir_ascii, "driver_inputs", out_ascii_steps)
            e90.WriteASCII(output_dir_ascii, "chassis", out_ascii_steps)
            terrain.WriteASCII(output_dir_ascii, "terrain", out_ascii_steps)
            vis.WriteASCII(output_dir_ascii, "vehicle_vis", out_ascii_steps)

        
        driver.Advance(step_size)
        terrain.Advance(step_size)
        e90.Advance(step_size)
        vis.Advance(step_size)

        
        if (driver_inputs.m_steering > 0.1):
            driver.SetSteeringDelta(0)
        if (driver_inputs.m_throttle > 0.1):
            driver.SetThrottleDelta(0)
        if (driver_inputs.m_braking > 0.1):
            driver.SetBrakingDelta(0)

        
        
        
        time = driver.GetTime()
        timeAsset = time * render_step_size

        
        my_system.GetChTime().SetTime(time)

        
        my_vis.Render()

        
        if (steering_inputs.m_steering > 0.1):
            steering_inputs.SetSteeringDelta(0)
        if (throttle_inputs.m_throttle > 0.1):
            throttle_inputs.SetThrottleDelta(0)
        if (braking_inputs.m_braking > 0.1):
            braking_inputs.SetBrakingDelta(0)

        
        my_system.GetChTime().SetTime(time)

        
        driver.Advance(step_size)
        terrain.Advance(step_size)
        e90.Advance(step_size)
        vis.Advance(step_size)

        
        if (out_pdf && time >= t_out_pdf[0] && time <= t_out_pdf[1]) :
            driver.WritePDF(output_dir_pdf, "driver_inputs", 2, out_pdf_steps)
            e90.WritePDF(output_dir_pdf, "chassis", 2, out_pdf_steps)
            terrain.WritePDF(output_dir_pdf, "terrain", 2, out_pdf_steps)
            vis.WritePDF(output_dir_pdf, "vehicle_vis", 2, out_pdf_steps)
            driver.Synchronize(time)
            terrain.Synchronize(time)
            e90.Synchronize(time, driver_inputs, terrain)
            vis.Synchronize(time, driver_inputs)
        if (out_ascii && time >= t_out_ascii[0] && time <= t_out_ascii[1]) :
            driver.WriteASCII(output_dir_ascii, "driver_inputs", out_ascii_steps)
            e90.WriteASCII(output_dir_ascii, "chassis", out_ascii_steps)
            terrain.WriteASCII(output_dir_ascii, "terrain", out_ascii_steps)
            vis.WriteASCII(output_dir_ascii, "vehicle_vis", out_ascii_steps)

        
        driver.Advance(step_size)
        terrain.Advance(step_size)
        e90.Advance(step_size)
        vis.Advance(step_size)

        
        if (driver_inputs.m_steering > 0.1):
            driver.SetSteeringDelta(0)
        if (driver_inputs.m_throttle > 0.1):
            driver.SetThrottleDelta(0)
        if (driver_inputs.m_braking > 0.1):
            driver.SetBrakingDelta(0)

        
        
        
        time = driver.GetTime()
        timeAsset = time * render_step_size

        
        my_system.GetChTime().SetTime(time)

        
        my_vis.Render()

        
        if (steering_inputs.m_steering > 0.1):
            steering_inputs.SetSteeringDelta(0)
        if (throttle_inputs.m_throttle > 0.1):
            throttle_inputs.SetThrottleDelta(0)
        if (braking_inputs.m_braking > 0.1):
            braking_inputs.SetBrakingDelta(0)

        
        my_system.GetChTime().SetTime(time)

        
        driver.Advance(step_size)
        terrain.Advance(step_size)
        e90.Advance(step_size)
        vis.Advance(step_size)

        
        if (out_pdf && time >= t_out_pdf[0] && time <= t_out_pdf[1]) :
            driver.WritePDF(output_dir_pdf, "driver_inputs", 2, out_pdf_steps)
            e90.WritePDF(output_dir_pdf, "chassis", 2, out_pdf_steps)
            terrain.WritePDF(output_dir_pdf, "terrain", 2, out_pdf_steps)
            vis.WritePDF(output_dir_pdf, "vehicle_vis", 2, out_pdf_steps)
            driver.Synchronize(time)
            terrain.Synchronize(time)
            e90.Synchronize(time, driver_inputs, terrain)
            vis.Synchronize(time, driver_inputs)
        if (out_ascii && time >= t_out_ascii[0] && time <= t_out_ascii[1]) :
            driver.WriteASCII(output_dir_ascii, "driver_inputs", out_ascii_steps)
            e90.WriteASCII(output_dir_ascii, "chassis", out_ascii_steps)
            terrain.WriteASCII(output_dir_ascii, "terrain", out_ascii_steps)
            vis.WriteASCII(output_dir_ascii, "vehicle_vis", out_ascii_steps)

        
        driver.Advance(step_size)
        terrain.Advance(step_size)
        e90.Advance(step_size)
        vis.Advance(step_size)

        
        if (driver_inputs.m_steering > 0.1):
            driver.SetSteeringDelta(0)
        if (driver_inputs.m_throttle > 0.1):
            driver.SetThrottleDelta(0)
        if (driver_inputs.m_braking > 0.1):
            driver.SetBrakingDelta(0)

        
        
        
        time = driver.GetTime()
        timeAsset = time * render_step_size

        
        my_system.GetChTime().SetTime(time)

        
        my_vis.Render()

        
        if (steering_inputs.m_steering > 0.1):
            steering_inputs.SetSteeringDelta(0)
        if (throttle_inputs.m_throttle > 0.1):
            throttle_inputs.SetThrottleDelta(0)
        if (braking_inputs.m_braking > 0.1):
            braking_inputs.SetBrakingDelta(0)

        
        my_system.GetChTime().SetTime(time)

        
        driver.Advance(step_size)
        terrain.Advance(step_size)
        e90.Advance(step_size)
        vis.Advance(step_size)

        
        if (out_pdf && time >= t_out_pdf[0] && time <= t_out_pdf[1]) :
            driver.WritePDF(output_dir_pdf, "driver_inputs", 2, out_pdf_steps)
            e90.WritePDF(output_dir_pdf, "chassis", 2, out_pdf_steps)
            terrain.WritePDF(output_dir_pdf, "terrain", 2, out_pdf_steps)
            vis.WritePDF(output_dir_pdf, "vehicle_vis", 2, out_pdf_steps)
            driver.Synchronize(time)
            terrain.Synchronize(time)
            e90.Synchronize(time, driver_inputs, terrain)
            vis.Synchronize(time, driver_inputs)
        if (out_ascii && time >= t_out_ascii[0] && time <= t_out_ascii[1]) :
            driver.WriteASCII(output_dir_ascii, "driver_inputs", out_ascii_steps)
            e90.WriteASCII(output_dir_ascii, "chassis", out_ascii_steps)
            terrain.WriteASCII(output_dir_ascii, "terrain", out_ascii_steps)
            vis.WriteASCII(output_dir_ascii, "vehicle_vis", out_ascii_steps)

        
        driver.Advance(step_size)
        terrain.Advance(step_size)
        e90.Advance(step_size)
        vis.Advance(step_size)

        
        if (driver_inputs.m_steering > 0.1):
            driver.SetSteeringDelta(0)
        if (driver_inputs.m_throttle > 0.1):
            driver.SetThrottleDelta(0)
        if (driver_inputs.m_braking > 0.1):
            driver.SetBrakingDelta(0)

        
        
        
        time = driver.GetTime()
        timeAsset = time * render_step_size

        
        my_system.GetChTime().SetTime(time)

        
        my_vis.Render()