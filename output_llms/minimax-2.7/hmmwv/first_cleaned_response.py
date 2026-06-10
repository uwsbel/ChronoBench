import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import os


veh.SetDataPath(os.path.join(os.path.dirname(__file__), 'vehicle/data/'))


def main():
    print("=" * 60)
    print("PyChrono HMMWV Simulation")
    print("=" * 60)
    
    
    
    
    
    
    my_system = chrono.ChSystemNSC()
    my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  
    my_system.SetSolverType(chrono.ChSolver.Type_PSSOR)
    my_system.SetMaxIteration(50)
    my_system.SetStepSize(1e-3)
    
    
    
    
    
    
    terrain_material = veh.ChMaterialSurfaceNSC()
    
    
    terrain = veh.RigidTerrain(my_system, terrain_material)
    
    
    
    terrain.Initialize(
        chrono.ChVectorD(0, 0, 0),  
        200,                          
        200,                          
        0.2,                          
        True                          
    )
    
    
    try:
        terrain.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"))
    except:
        
        print("Warning: Terrain texture not found, using default")
    
    
    terrain.SetColor(chrono.ChColor(0.5, 0.5, 0.5))
    
    
    
    
    
    
    vehicle_position = chrono.ChVectorD(0, 1.5, 0)
    vehicle_orientation = chrono.ChQuaternionD(1, 0, 0, 0)  
    
    
    chassis_material = veh.ChMaterialSurfaceNSC()
    
    
    print("\nInitializing HMMWV vehicle...")
    my_hmmwv = veh.HMMWV_Reduced()
    
    
    my_hmmwv.SetContactMethod(veh.ChContactMethod_NSC)
    
    
    my_hmmwv.SetChassisMaterial(chassis_material)
    my_hmmwv.SetWheelMaterial(chassis_material)
    
    
    my_hmmwv.Initialize(
        vehicle_position,
        vehicle_orientation
    )
    
    
    my_hmmwv.SetChassisCollision(True)
    my_hmmwv.SetWheelCollision(True)
    
    
    tire_vis = veh.TireVisualizationType_PROXY
    for axle_idx in range(my_hmmwv.GetNumberOfAxles()):
        for wheel_idx in range(2):
            my_hmmwv.SetTireVisualizationType(axle_idx, wheel_idx, tire_vis)
    
    
    for axle_idx in range(my_hmmwv.GetNumberOfAxles()):
        for wheel_idx in range(2):
            my_hmmwv.SetTireModelType(
                axle_idx, 
                wheel_idx, 
                veh.TireModelType_TMEASY
            )
    
    
    tire_material = veh.ChMaterialSurfaceNSC()
    for axle_idx in range(my_hmmwv.GetNumberOfAxles()):
        for wheel_idx in range(2):
            my_hmmwv.SetTireMaterial(tire_material, axle_idx, wheel_idx)
    
    
    my_hmmwv.InitializeTires(
        veh.TireCollisionType_RIGID,  
        veh.TireVisualizationType_PROXY  
    )
    
    print(f"Vehicle initialized at position: {vehicle_position}")
    print(f"Number of axles: {my_hmmwv.GetNumberOfAxles()}")
    
    
    
    
    
    print("\nSetting up driver system...")
    
    
    my_driver = veh.ChDriver(my_hmmwv.GetChassis())
    
    
    driver_inputs = veh.DriverInputs()
    driver_inputs.m_steering = 0.0
    driver_inputs.m_throttle = 0.0
    driver_inputs.m_braking = 0.0
    
    
    my_driver.Initialize()
    
    
    my_driver.SetTimeDelay(0.3)
    
    print("Driver system initialized")
    
    
    
    
    
    print("\nSetting up Irrlicht visualization...")
    
    
    my_app = irr.ChIrrApp(
        my_system,              
        "PyChrono HMMWV Simulation",  
        irr.dimension2du(1280, 720),  
        irr.VIEW_ERDEBUG | irr.VIEW_ERRAW,  
        irr.EVENT_DRIVER | irr.EVENT_KEYBOARD,  
        50                      
    )
    
    
    my_app.AddTypicalSky()
    my_app.AddTypicalLogo(veh.GetDataFile("vehicle/logo/chrono_logo_alpha.png"))
    my_app.AddTypicalLights(
        irr.dimension2df(0.15, 0.15),  
        100,                            
        0.6, 0.5, 1                     
    )
    
    
    my_app.AddTypicalCamera(
        irr.vector3df(6, 4, -6),        
        irr.vector3df(0, 2, 0)          
    )
    
    
    my_hmmwv.AddVisualizationAssets(my_app.GetSceneManager(), my_system)
    
    
    terrain.AddVisualizationAssets(my_app.GetSceneManager())
    
    
    
    
    
    
    speed_text = irr.ChIrrGuiManager.addStaticText(
        irr.cstring_to_wide("Speed: 0 km/h"),
        irr.rect<s32>(10, 10, 250, 40),
        True,
        True
    )
    
    
    
    
    
    print("\n" + "=" * 60)
    print("Starting Simulation")
    print("=" * 60)
    print("\nControls:")
    print("  W - Accelerate")
    print("  S - Brake/Reverse")
    print("  A - Steer Left")
    print("  D - Steer Right")
    print("  Space - Emergency Brake")
    print("  R - Reset")
    print("  ESC - Exit")
    print("\nSimulation running at 50 FPS real time...")
    print("=" * 60)
    
    
    simulation_step_size = 1e-3  
    frame_rate = 50              
    simulation_time = 0
    frame_count = 0
    
    
    my_app.SetTimestep(simulation_step_size)
    my_app.SetTryRealtime(False)  
    
    while my_app.GetDevice().run():
        
        my_app.BeginScene()
        
        
        my_app.GetVideoDriver().beginScene(True, True, irr.SColor(255, 100, 100, 100))
        
        
        my_app.DrawAll()
        
        
        my_driver.Update()
        
        
        driver_inputs = my_driver.GetInputs()
        
        
        my_hmmwv.Synchronize(
            frame_count,
            driver_inputs,
            my_system.GetChTime()
        )
        
        
        my_app.DoStep()
        
        
        simulation_time = my_system.GetChTime()
        frame_count += 1
        
        
        if frame_count % 10 == 0:  
            chassis_velocity = my_hmmwv.GetChassis().GetPos_dt()
            speed_ms = chassis_velocity.Length()
            speed_kmh = speed_ms * 3.6
            
            
            status_text = f"Time: {simulation_time:.2f}s | Speed: {speed_kmh:.1f} km/h | FPS: {frame_rate}"
            irr.ChIrrGuiManager.setCaption(status_text)
        
        
        my_app.GetVideoDriver().endScene()
        
        
        if irr.ChIrrEventReceiverKey.getKeyState(irr.KEY_KEY_R):
            
            my_system.Clear()
            my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
            terrain.Initialize(
                chrono.ChVectorD(0, 0, 0),
                200, 200, 0.2, True
            )
            my_hmmwv.Initialize(vehicle_position, vehicle_orientation)
            my_hmmwv.InitializeTires(
                veh.TireCollisionType_RIGID,
                veh.TireVisualizationType_PROXY
            )
            simulation_time = 0
            frame_count = 0
            print("\nSimulation reset!")
    
    print("\n" + "=" * 60)
    print("Simulation Complete")
    print(f"Total frames: {frame_count}")
    print(f"Total simulation time: {simulation_time:.2f} seconds")
    print("=" * 60)


if __name__ == "__main__":
    main()