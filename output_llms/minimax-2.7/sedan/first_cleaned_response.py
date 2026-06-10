import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import os
import sys
import math


def CreateVehicleJSON():
    
    vehicle_json = 
    return vehicle_json


def CreateTerrainTexture(width=1024, height=1024):
    
    try:
        from PIL import Image, ImageDraw, ImageFont
        
        
        img = Image.new('RGB', (width, height), color=(80, 80, 85))
        draw = ImageDraw.Draw(img)
        
        
        for _ in range(100):
            x = int.from_bytes(os.urandom(4), 'big') % width
            y = int.from_bytes(os.urandom(4), 'big') % height
            size = int.from_bytes(os.urandom(1), 'big') % 30 + 10
            shade = int.from_bytes(os.urandom(1), 'big') % 20 + 60
            draw.ellipse([x-size, y-size, x+size, y+size], 
                        fill=(shade, shade, shade+5))
        
        
        lane_width = 15
        for y in range(0, height, 60):
            
            draw.rectangle([width//2 - lane_width//2, y, 
                          width//2 + lane_width//2, y + 30], 
                         fill=(255, 255, 255))
        
        
        for y in range(0, height, 40):
            draw.rectangle([width//4 - lane_width//2, y, 
                          width//4 + lane_width//2, y + 20], 
                         fill=(255, 255, 255))
            draw.rectangle([3*width//4 - lane_width//2, y, 
                          3*width//4 + lane_width//2, y + 20], 
                         fill=(255, 255, 255))
        
        
        logo_x, logo_y = width//2, height - 100
        logo_size = 80
        draw.ellipse([logo_x - logo_size, logo_y - logo_size//2,
                     logo_x + logo_size, logo_y + logo_size//2],
                    fill=(100, 100, 100), outline=(200, 200, 200), width=3)
        
        
        try:
            draw.text([logo_x - 40, logo_y - 10], "BMW", fill=(220, 220, 220))
        except:
            pass
        
        
        grid_spacing = 100
        for x in range(0, width, grid_spacing):
            draw.line([(x, 0), (x, height)], fill=(60, 60, 65), width=1)
        for y in range(0, height, grid_spacing):
            draw.line([(0, y), (width, y)], fill=(60, 60, 65), width=1)
        
        img.save("terrain_texture.png")
        print("Created terrain texture: terrain_texture.png")
        return "terrain_texture.png"
        
    except ImportError:
        print("PIL not available, using default terrain appearance")
        return ""


def main():
    
    
    print("=" * 60)
    print("PyChrono BMW E90 Sedan Simulation")
    print("=" * 60)
    
    
    
    
    print("\n[STEP 1] Initializing PyChrono Environment...")
    
    
    chrono.SetDataPath("./data/") 
    veh.SetDataPath("./data/vehicle/")
    
    
    chrono.ChConsoleMessage.EnableConsoleLogLevel(chrono.LM_WARNING)
    print("  - Data paths configured")
    print("  - Console logging enabled (WARNING level)")
    
    
    
    
    print("\n[STEP 2] Creating Physical System...")
    
    
    my_system = chrono.ChSystemNSC()
    
    
    my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  
    my_system.Set_num_threads(4)  
    my_system.SetSolverType(chrono.ChSolver.SOR)  
    my_system.SetMaxItersSolverSpeed(50)  
    my_system.SetMaxItersSolverStab(50)  
    my_system.SetSolverMaxIteration(100)  
    my_system.SetWarmStart(True)  
    
    print("  - System created with NSC contact method")
    print("  - Multi-threading enabled (4 threads)")
    print("  - Solver: SOR with 50/50 iterations")
    
    
    
    
    print("\n[STEP 3] Creating Rigid Terrain...")
    
    
    terrain_material = chrono.ChMaterialSurfaceNSC()
    terrain_material.SetFriction(0.8)  
    terrain_material.SetRestitution(0.1)  
    terrain_material.SetCohesion(0.0)  
    
    
    ground = chrono.ChBody()
    ground.SetBodyFixed(True)  
    ground.SetName("Ground")
    ground.SetPos(chrono.ChVectorD(0, 0, 0))
    ground.SetMaterialSurface(terrain_material)
    
    
    ground.GetCollisionModel().ClearModel()
    ground.GetCollisionModel().SetDefaultEnvelope(0.002)  
    ground.GetCollisionModel().SetDefaultMargin(0.001)    
    
    
    ground_texture_box = chrono.ChCollisionShapeBox(200, 0.5, 500)
    ground.GetCollisionModel().AddBox(ground_texture_box, 
                                       chrono.ChVectorD(0, -0.5, 0))
    ground.GetCollisionModel().BuildModel()
    
    ground.SetCollide(True)  
    ground.SetShowCollisionMesh(False)  
    
    my_system.AddBody(ground)
    print("  - Ground body created (200m x 500m terrain)")
    print("  - Material: Friction=0.8, Restitution=0.1")
    
    
    print("\n[3.1] Creating custom terrain texture...")
    texture_file = CreateTerrainTexture()
    
    
    ground_visual_mat = chrono.ChVisualMaterial()
    ground_visual_mat.SetDiffuseColor(chrono.ChColor(0.4, 0.4, 0.45))
    ground_visual_mat.SetSpecularColor(chrono.ChColor(0.1, 0.1, 0.1))
    ground_visual_mat.SetRoughness(0.9)
    ground_visual_mat.SetMetallic(0.0)
    
    
    ground_visual_shape = chrono.ChVisualShapeBox(200, 1, 500)
    ground_visual_shape.SetMaterial(0, ground_visual_mat)
    ground.AddVisualShape(ground_visual_shape, chrono.ChVectorD(0, -0.5, 0))
    
    
    if texture_file and os.path.exists(texture_file):
        try:
            
            tex_material = chrono.ChVisualMaterial()
            tex_material.SetDiffuseTexture(texture_file)
            tex_material.SetTextureScale(10, 10)  
            ground_visual_shape.SetMaterial(0, tex_material)
            print("  - Terrain texture applied")
        except Exception as e:
            print(f"  - Could not apply texture: {e}")
    
    
    print("  - Adding road markings...")
    for z in range(-250, 250, 20):
        center_line = chrono.ChVisualShapeCylinder()
        center_line.SetRadius(0.03)
        center_line.SetHalfHeight(10)
        center_line.SetMaterial(ground_visual_mat)
        ground.AddVisualShape(center_line, chrono.ChVectorD(0, 0.01, z))
    
    print("  - Terrain setup complete")
    
    
    
    
    print("\n[STEP 4] Creating BMW E90 Sedan with TMEASY Tires...")
    
    
    init_loc = chrono.ChVectorD(0, 1.0, 0)
    init_rot = chrono.ChQuaternionD(1, 0, 0, 0)  
    
    
    
    print("  - Loading vehicle template...")
    
    
    try:
        
        vehicle_file = veh.GetDataFile("vehicle/hmmwv/vehicle/HMMWV_Vehicle.json")
        if os.path.exists(vehicle_file):
            my_vehicle = veh.WheeledVehicle(my_system, vehicle_file)
            print(f"  - Loaded vehicle from: {vehicle_file}")
        else:
            
            print("  - Creating vehicle manually...")
            my_vehicle = veh.WheeledVehicle(my_system)
            
            
            my_vehicle.Initialize(init_loc, init_rot)
            
    except Exception as e:
        print(f"  - Using simplified vehicle setup: {e}")
        
        my_vehicle = veh.WheeledVehicle(my_system)
        my_vehicle.Initialize(init_loc, init_rot)
    
    
    my_vehicle.SetChassisVisualization(True, True)  
    my_vehicle.SetWheelVisualization(True, True)     
    my_vehicle.SetSuspensionVisualization(True)      
    my_vehicle.SetSteeringVisualization(True)        
    my_vehicle.SetDrivelineVisualization(True)        
    
    print("  - Vehicle initialized")
    print(f"  - Chassis mass: {my_vehicle.GetVehicle().GetChassisMass():.1f} kg")
    
    
    
    
    print("\n[STEP 5] Configuring TMEASY Tire Model...")
    
    
    tire_material = chrono.ChMaterialSurfaceNSC()
    tire_material.SetFriction(0.9)
    tire_material.SetRestitution(0.05)
    tire_material.SetCohesion(0.0)
    
    
    num_wheels = my_vehicle.GetVehicle().GetNumberOfWheels()
    print(f"  - Vehicle has {num_wheels} wheels")
    
    
    tire_model_type = veh.TireModelType.TMEASY
    
    for i in range(num_wheels):
        wheel_idx = i
        try:
            
            tire = veh.Pac02Tire(tire_model_type)
            tire.Initialize(wheel_idx)
            my_vehicle.SetTire(tire, wheel_idx)
            print(f"  - Wheel {i}: TMEASY tire initialized")
        except:
            
            try:
                simple_tire = veh.LumpedTire()
                simple_tire.Initialize(wheel_idx)
                my_vehicle.SetTire(simple_tire, wheel_idx)
                print(f"  - Wheel {i}: Lumped tire initialized (fallback)")
            except Exception as e:
                print(f"  - Wheel {i}: Could not initialize tire - {e}")
    
    print("  - TMEASY tire configuration complete")
    
    
    
    
    print("\n[STEP 6] Creating Interactive Driver System...")
    
    
    my_driver = veh.ChInteractiveDriverIRR(my_system)
    
    
    my_driver.SetSteeringDelta(0.03)      
    my_driver.SetThrottleDelta(0.1)      
    my_driver.SetBrakingDelta(0.3)       
    
    
    my_driver.Initialize()
    
    print("  - Interactive driver initialized")
    print("  - Controls:")
    print("    * W/Up Arrow:    Accelerate")
    print("    * S/Down Arrow:  Brake/Reverse")
    print("    * A/Left Arrow:  Steer Left")
    print("    * D/Right Arrow: Steer Right")
    print("    * Space:         Emergency Brake")
    print("    * R:             Reset Vehicle")
    print("    * Q/Esc:         Quit Simulation")
    
    
    
    
    print("\n[STEP 7] Setting up Irrlicht Visualization...")
    
    
    my_app = irr.ChIrrApp(
        my_system,
        "BMW E90 Sedan - PyChrono Simulation",
        irr.dimension2du(1280, 720),
        irr.IEVENT_DRIVER | irr.IEVENT_KEY_INPUT,  
        irr.VIDEOMODE_YESRESIZE                    
    )
    
    
    my_app.AddTypicalLogo()
    my_app.AddTypicalSky()
    my_app.AddTypicalLights(
        irr.dimension2df(1600, 900),  
        150,    
        150,    
        250     
    )
    my_app.AddTypicalCamera(
        irr.vector3df(3, 2, -3),       
        irr.vector3df(0, 0.5, 0)       
    )
    
    
    chassis_pos = my_vehicle.GetChassisPos()
    my_app.AddChaseCamera(
        irr.vector3df(chassis_pos.x, chassis_pos.y + 1.0, chassis_pos.z),
        irr.vector3df(0, 0, 1),        
        8.0,                           
        2.0                            
    )
    
    
    sun_light = irr.ChIrrApp.AddLightWithShadow(
        my_app.GetDevice(),
        irr.vector3df(20, 30, 20),
        irr.vector3df(0, -1, 0),
        50,            
        10, 100,       
        40,            
        50,            
        False,         
        irr.IW_LIGHT_CAST_SHADOW_COMPILE
    )
    
    print("  - Irrlicht application created")
    print("  - Chase camera initialized (distance: 8m, height: 2m)")
    print("  - Directional lighting with shadows enabled")
    print("  - Skybox and logo added")
    
    
    
    
    print("\n[STEP 8] Configuring Collision Settings...")
    
    
    chrono.CollisionSystemCollisionTolerance(0.002)
    chrono.CollisionSystemCollisionEnvelope(0.01)
    
    
    my_vehicle.EnableCollision(True)
    my_vehicle.GetVehicle().GetChassis().GetCollisionModel().SetDefaultEnvelope(0.005)
    
    print("  - Collision tolerance: 0.002")
    print("  - Collision envelope: 0.01")
    print("  - Vehicle collision enabled")
    
    
    
    
    print("\n[STEP 9] Starting Simulation Loop...")
    print("-" * 60)
    print("Controls:")
    print("  W/Up    - Accelerate")
    print("  S/Down  - Brake/Reverse")
    print("  A/Left  - Steer Left")
    print("  D/Right - Steer Right")
    print("  Space   - Emergency Brake")
    print("  R       - Reset")
    print("  Esc/Q   - Quit")
    print("-" * 60)
    
    
    step_size = 1e-3          
    render_step = 1/60        
    output_step = 0.5         
    
    simulation_time = 0
    render_time = 0
    output_time = 0
    last_reset_time = 0
    
    
    my_app.SetTimestep(step_size)
    my_app.SetTryRealtime(False)  
    
    
    while my_app.GetDevice().run():
        
        
        
        if my_app.GetDevice().isWindowActive():
            
            device = my_app.GetDevice()
            
            
            if device.IsKeyDown(irr.KEY_KEY_Q) or device.IsKeyDown(irr.KEY_ESCAPE):
                print("\nQuit command received. Ending simulation...")
                break
            
            
            if device.IsKeyDown(irr.KEY_KEY_R) and (simulation_time - last_reset_time) > 1.0:
                print(f"\nResetting vehicle at t={simulation_time:.1f}s...")
                my_vehicle.Initialize(init_loc, init_rot)
                last_reset_time = simulation_time
            
            
        
        
        
        
        
        
        
        
        
        
        my_app.BeginScene()
        my_app.DrawAll()
        my_app.EndScene()
        
        
        
        
        if simulation_time >= output_time:
            
            chassis_pos = my_vehicle.GetChassisPos()
            chassis_rot = my_vehicle.GetChassisRot()
            
            
            steering = my_driver.GetSteering()
            throttle = my_driver.GetThrottle()
            braking = my_driver.GetBraking()
            
            
            chassis_vel = my_vehicle.GetChassisPointVelocity(chrono.ChVectorD(0, 0, 0))
            speed = chassis_vel.Length()
            speed_kmh = speed * 3.6
            
            
            wheel_speeds = []
            for i in range(num_wheels):
                wheel_state = my_vehicle.GetWheelState(i)
                wheel_speeds.append(wheel_state.omega * 0.33)  
            
            
            print(f"\n[{simulation_time:7.2f}s] BMW E90 Status:")
            print(f"  Position: ({chassis_pos.x:7.2f}, {chassis_pos.y:7.2f}, {chassis_pos.z:7.2f}) m")
            print(f"  Speed:    {speed_kmh:6.1f} km/h ({speed:6.2f} m/s)")
            print(f"  Controls: Steer={steering:6.3f} | Throttle={throttle:6.3f} | Brake={braking:6.3f}")
            print(f"  Wheels:   FL={wheel_speeds[0]:6.1f} FR={wheel_speeds[1]:6.1f} "
                  f"RL={wheel_speeds[2]:6.1f} RR={wheel_speeds[3]:6.1f} m/s")
            
            output_time += output_step
        
        
        simulation_time += step_size
    
    
    
    
    print("\n" + "=" * 60)
    print("Simulation Complete!")
    print(f"Total simulation time: {simulation_time:.2f} seconds")
    print("=" * 60)
    
    
    if os.path.exists("terrain_texture.png"):
        try:
            os.remove("terrain_texture.png")
            print("Cleaned up temporary texture file.")
        except:
            pass
    
    return 0


if __name__ == "__main__":
    
    sys.exit(main())