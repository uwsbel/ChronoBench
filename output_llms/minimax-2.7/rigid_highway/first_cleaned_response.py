import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import os


def create_hmmwv_simulation():
    
    
    
    
    
    
    
    chrono.SetDataPath("../../../data/")
    
    
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
    
    
    system.SetMaxiterSpeedSolver(100)
    system.SetMaxiterStabSolver(20)
    system.SetSolverType(chrono.ChSolver.Type.SOR)
    system.SetTimestepperType(chrono.ChTimestepper.Type.HHT)
    
    
    
    
    
    print("Creating terrain system...")
    
    
    terrain = chrono.ChBody()
    terrain.SetName("Terrain")
    terrain.SetPos(chrono.ChVectorD(0, 0, 0))
    terrain.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))
    terrain.SetBodyFixed(True)
    terrain.SetCollide(True)
    
    
    collision_mesh = chrono.ChTriangleMeshConnected()
    collision_mesh_path = chrono.GetDataFile("terrain/Highway_col.obj")
    collision_mesh.LoadWavefrontMesh(collision_mesh_path, True, True)
    
    
    collision_shape = chrono.ChTriangleMeshShape()
    collision_shape.SetMesh(collision_mesh)
    collision_shape.SetName("Highway_collision_mesh")
    
    
    terrain_mat = chrono.ChMaterialSurfaceNSC()
    terrain_mat.SetFriction(0.8)
    terrain_mat.SetRestitution(0.1)
    
    terrain.AddCollisionShape(collision_shape, chrono.ChFrameD())
    terrain.GetCollisionModel().BuildModel()
    
    
    visual_mesh = chrono.ChTriangleMeshConnected()
    visual_mesh_path = chrono.GetDataFile("terrain/Highway_vis.obj")
    visual_mesh.LoadWavefrontMesh(visual_mesh_path, True, True)
    
    
    visual_shape = chrono.ChVisualShapeTriangleMesh(visual_mesh, False, 1.0)
    visual_shape.SetName("Highway_visual_mesh")
    terrain.AddVisualShape(visual_shape, chrono.ChFrameD())
    
    
    system.Add(terrain)
    
    
    
    
    
    print("Creating HMMWV vehicle...")
    
    
    vehicle_location = chrono.ChVectorD(0, 0.8, -5)
    vehicle_orientation = chrono.ChQuaternionD(1, 0, 0, 0)
    
    
    
    tire_radius = 0.5
    wheel_separation = 1.75  
    axle_distances = [1.6, 1.6]  
    
    
    chassis = chrono.ChBodyAuxRef()
    chassis.SetName("Chassis")
    chassis.SetPos(vehicle_location)
    chassis.SetRot(vehicle_orientation)
    chassis.SetMass(2000)  
    chassis.SetInertiaXX(chrono.ChVectorD(800, 800, 500))
    chassis.SetBodyFixed(False)
    chassis.SetCollide(True)
    
    
    chassis_mesh = chrono.ChTriangleMeshConnected()
    chassis_mesh_path = chrono.GetDataFile("hmmwv/hmmwv_chassis.obj")
    if os.path.exists(chassis_mesh_path):
        chassis_mesh.LoadWavefrontMesh(chassis_mesh_path, False, False)
        chassis_visual = chrono.ChVisualShapeTriangleMesh(chassis_mesh, False, 1.0)
        chassis.AddVisualShape(chassis_visual, chrono.ChFrameD())
    
    system.Add(chassis)
    
    
    wheelbase = 3.3  
    track_width = wheel_separation
    
    
    front_left_pos = vehicle_location + chrono.ChVectorD(0, -0.3, wheelbase/2) + \
                     chrono.ChVectorD(-track_width/2, 0, 0)
    front_right_pos = vehicle_location + chrono.ChVectorD(0, -0.3, wheelbase/2) + \
                      chrono.ChVectorD(track_width/2, 0, 0)
    rear_left_pos = vehicle_location + chrono.ChVectorD(0, -0.3, -wheelbase/2) + \
                    chrono.ChVectorD(-track_width/2, 0, 0)
    rear_right_pos = vehicle_location + chrono.ChVectorD(0, -0.3, -wheelbase/2) + \
                     chrono.ChVectorD(track_width/2, 0, 0)
    
    
    wheels = []
    wheel_data = [
        ("FL", front_left_pos, True),
        ("FR", front_right_pos, True),
        ("RL", rear_left_pos, False),
        ("RR", rear_right_pos, False)
    ]
    
    for name, pos, is_front in wheel_data:
        
        wheel = chrono.ChBody()
        wheel.SetName(f"Wheel_{name}")
        wheel.SetPos(pos)
        wheel.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))
        wheel.SetMass(50)
        wheel.SetInertiaXX(chrono.ChVectorD(2, 2, 2))
        wheel.SetCollide(True)
        
        
        wheel_collision = chrono.ChCylinderShape(tire_radius, 0.3)
        wheel.AddCollisionShape(wheel_collision, chrono.ChFrameD())
        
        
        wheel_mesh = chrono.ChTriangleMeshConnected()
        wheel_mesh_path = chrono.GetDataFile("hmmwv/hmmwv_wheel.obj")
        if os.path.exists(wheel_mesh_path):
            wheel_mesh.LoadWavefrontMesh(wheel_mesh_path, False, False)
            wheel_visual = chrono.ChVisualShapeTriangleMesh(wheel_mesh, False, 1.0)
            wheel.AddVisualShape(wheel_visual, chrono.ChFrameD())
        
        system.Add(wheel)
        wheels.append({"body": wheel, "name": name, "pos": pos, "is_front": is_front})
    
    
    tire_params = veh.TMeasyParameters()
    tire_params.radius = tire_radius
    tire_params.width = 0.35
    tire_params.vertical_stiffness = 350000
    tire_params.vertical_damping = 15000
    tire_params.longitudinal_stiffness = 350000
    tire_params.lateral_stiffness = 85000
    
    tires = []
    for wheel_info in wheels:
        tire = veh.TMeasyTire(wheel_info["name"], tire_params)
        tire.Initialize(wheel_info["body"])
        tires.append(tire)
    
    
    vehicle = veh.WheeledVehicle(system, chassis, wheels, tires)
    vehicle.Initialize()
    
    
    
    
    
    print("Creating powertrain system...")
    
    
    powertrain = veh.SimplePowertrain()
    powertrain.Initialize(chassis)
    
    
    driveline = veh.SimpleDriveline4WD()
    driveline.Initialize(axle_distances, wheelbase)
    
    
    
    
    
    
    tire_mat = chrono.ChMaterialSurfaceNSC()
    tire_mat.SetFriction(0.8)
    tire_mat.SetRestitution(0.3)
    tire_mat.SetCompliance(0.0)
    tire_mat.SetCohesion(0.0)
    
    
    
    
    
    print("Setting up Irrlicht visualization...")
    
    
    application = chronoirr.ChIrrApp(
        system,
        "HMMWV Simulation on Custom Terrain",
        chronoirr.dimension2du(1280, 720),
        chronoirr.ShualdTypes.EFW_BORDERLESS
    )
    
    application.AddTypicalLogo()
    application.AddTypicalSky()
    application.AddTypicalCamera(
        chronoirr.vector3df(8, 6, -8),  
        chronoirr.vector3df(0, 0, 0)     
    )
    application.AddTypicalLights()
    application.AddLightWithShadow(
        chronoirr.vector3df(20, 30, -20),
        chronoirr.vector3df(0, 0, 0),
        50, 10, 50,
        40, 512
    )
    
    
    application.AddCustomTool(
        terrain,
        chronoirr.dimension2du(400, 300),
        True,
        False,
        chronoirr.volumetric公立(
            chronoirr.SColorf(0.5, 0.5, 0.5),
            chronoirr.SColorf(0.2, 0.2, 0.2),
            0.1
        )
    )
    
    
    application.AddVehicle(
        vehicle,
        chronoirr.ChVisualizationType.MESH,
        False,  
        False,  
        False   
    )
    
    
    
    
    
    print("Creating interactive driver system...")
    
    driver = veh.ChInteractiveDriver()
    driver.Initialize(system, vehicle)
    driver.SetSteeringDelta(0.04)
    driver.SetThrottleDelta(0.04)
    driver.SetBrakingDelta(0.04)
    
    
    driver.RegisterKeyAction(veh.DriverInputs.STEERING, 
                            chronoirr.EKEY_CODE.EKEY_UP)
    driver.RegisterKeyAction(veh.DriverInputs.STEERING, 
                            chronoirr.EKEY_CODE.EKEY_DOWN)
    driver.RegisterKeyAction(veh.DriverInputs.THROTTLE, 
                            chronoirr.EKEY_CODE.EKEY_KEY_W)
    driver.RegisterKeyAction(veh.DriverInputs.THROTTLE, 
                            chronoirr.EKEY_CODE.EKEY_KEY_S)
    driver.RegisterKeyAction(veh.DriverInputs.BRAKING, 
                            chronoirr.EKEY_CODE.EKEY_KEY_A)
    driver.RegisterKeyAction(veh.DriverInputs.BRAKING, 
                            chronoirr.EKEY_CODE.EKEY_KEY_D)
    
    
    
    
    
    
    application.AddHUDMessage(
        "HMMWV Simulation - PyChrono\n"
        "Controls: WASD or Arrow Keys\n"
        "W/Up: Throttle | S/Down: Brake\n"
        "A/Left: Steer Left | D/Right: Steer Right",
        chronoirr.vector3df(0.02, 0.85, 0)
    )
    
    
    
    
    
    return system, vehicle, application, driver


def run_simulation(system, vehicle, application, driver):
    
    
    
    FPS = 50
    dt = 1.0 / FPS
    end_time = 100.0  
    
    print(f"Starting simulation at {FPS} FPS...")
    print("Press ESC or close window to exit\n")
    
    
    simulation_time = 0.0
    frame_count = 0
    
    
    while application.GetDevice().run():
        
        if application.GetDevice().isWindowActive() == False:
            break
        
        
        application.BeginScene()
        
        
        application.GetVideoDriver().beginScene(
            True,  
            True,  
            chronoirr.SColor(255, 100, 100, 100)  
        )
        
        
        driver.Synchronize(simulation_time)
        
        
        vehicle.Synchronize(simulation_time)
        
        
        system.DoStepDynamics(dt)
        
        
        application.DrawAll()
        
        
        state = vehicle.GetState()
        steering = driver.GetInput().steering
        throttle = driver.GetInput().throttle
        braking = driver.GetInput().braking
        
        hud_text = (
            f"Time: {simulation_time:.2f}s | FPS: {FPS}\n"
            f"Speed: {state.speed * 3.6:.1f} km/h\n"
            f"Steering: {steering:.2f} | "
            f"Throttle: {throttle:.2f} | "
            f"Brake: {braking:.2f}"
        )
        application.AddHUDMessage(
            hud_text,
            chronoirr.vector3df(0.02, 0.02, 0),
            chronoirr.SColor(255, 255, 255, 0)
        )
        
        
        application.GetVideoDriver().endScene()
        
        
        simulation_time += dt
        frame_count += 1
        
        
        if simulation_time >= end_time:
            print(f"Simulation completed at t = {end_time}s")
            break
    
    print(f"Total frames: {frame_count}")
    return frame_count


def main():
    
    
    print("=" * 60)
    print("PyChrono HMMWV Simulation")
    print("=" * 60)
    print()
    
    try:
        
        system, vehicle, application, driver = create_hmmwv_simulation()
        
        print("\nSimulation environment initialized successfully!")
        print("Starting main simulation loop...\n")
        
        
        frames = run_simulation(system, vehicle, application, driver)
        
        print(f"\nSimulation completed successfully!")
        print(f"Total frames rendered: {frames}")
        
    except Exception as e:
        print(f"\nERROR: Simulation failed with exception:")
        print(f"  {type(e).__name__}: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    return 0


if __name__ == "__main__":
    exit(main())