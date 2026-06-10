import numpy as np
import math






import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr


veh.SetDataPath(veh.GetDefaultDataPath())


system = chrono.ChSystemSMC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


system.SetSolverType(chrono.ChSolver.Type_SOR)
system.SetMaxItersSolverSpeed(50)
system.SetMaxItersSolverStab(50)
system.SetTolForce(1e-4)
system.Set timestep(0.002)  


contact_method = veh.ChContactMethod_SMC
friction_model = chrono.ChContactFrictionModel_MicroSlip

print("=" * 60)
print("PyChrono Kraz Vehicle Simulation Initialized")
print("=" * 60)










terrain_length = 200.0  
terrain_width = 50.0    
terrain_thickness = 1.0  


ground_material = chrono.ChMaterialSurfaceSMC()
ground_material.SetFriction(0.8)       
ground_material.SetRestitution(0.1)   
ground_material.SetAdhesion(0.0)       


ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.SetPos(chrono.ChVectorD(0, -terrain_thickness/2, 0))
ground.SetMaterialSurface(chrono.ChMaterialSurfaceSMC(ground_material))


ground_shape = chrono.ChCollisionShapeBox(terrain_length/2, terrain_thickness/2, terrain_width/2)
ground.AddCollisionShape(ground_shape, chrono.ChFrameD())
ground.SetCollide(True)

system.AddBody(ground)


ground_visual = chrono.ChVisualShapeBox(terrain_length, terrain_thickness, terrain_width)
ground_visual.SetTexture(veh.GetDataPath() + "vehicle/textures/tile4.jpg")
ground.AddVisualShape(ground_visual, chrono.ChFrameD())

print(f"Terrain created: {terrain_length}m x {terrain_width}m")
print(f"  - Friction coefficient: 0.8")
print(f"  - Restitution coefficient: 0.1")






vehicle_start_position = chrono.ChVectorD(-50, 1.5, 0)
vehicle_start_orientation = chrono.Q_ROTATE_Y_TO_Z


print("\nInitializing Kraz vehicle...")
kraz = veh.KRAZ(system, veh.CollisionType_LINE, contact_method)


kraz.SetInitPosition(veh.ChCoordsysD(vehicle_start_position, vehicle_start_orientation))
kraz.SetLightsVisualization(True)
kraz.SetChassisCollision(False)


kraz.Initialize()


chassis = kraz.GetChassis()
chassis_body = chassis.GetBody()

print("Kraz vehicle initialized successfully!")
print(f"  - Vehicle mass: {chassis_body.GetMass():.2f} kg")
print(f"  - Number of wheels: {kraz.GetNumberAxles() * 2}")
print(f"  - Initial position: ({vehicle_start_position.x}, {vehicle_start_position.y}, {vehicle_start_position.z})")






wheel_material = chrono.ChMaterialSurfaceSMC()
wheel_material.SetFriction(0.9)        
wheel_material.SetRestitution(0.05)    
wheel_material.SetYoungModulus(1.0e7)  
wheel_material.SetPoissonRatio(0.3)    


for axle in kraz.GetAxles():
    for wheel in axle.GetWheels():
        wheel.GetWheelBody().SetMaterialSurface(chrono.ChMaterialSurfaceSMC(wheel_material))

print("Wheel materials configured")










initial_speed = 15.0  


for axle in kraz.GetAxles():
    for wheel in axle.GetWheels():
        wheel.GetWheelBody().SetPos_dt(chrono.ChVectorD(initial_speed, 0, 0))


chassis_body.SetPos_dt(chrono.ChVectorD(initial_speed, 0, 0))

print(f"\nInitial conditions set:")
print(f"  - Initial speed: {initial_speed} m/s ({initial_speed * 3.6:.1f} km/h)")






driver = veh.ChDriver(kraz.GetVehicle())


driver.SetSteeringDelta(0.05)      
driver.SetThrottleDelta(0.1)        
driver.SetBrakingDelta(0.2)         


driver.Initialize()

print("Driver system initialized")






steering_controller = veh.ChSteeringController()


steering_controller.SetGains(
    proportional_gain=0.5,    
    integral_gain=0.01,        
    derivative_gain=0.1       
)


steering_controller.SetLookAheadDistance(5.0)  
steering_controller.SetMaxSteeringAngle(0.5)   

print("Steering controller configured")






speed_controller = veh.ChSpeedController()


desired_speed = 20.0  

speed_controller.SetGains(
    proportional_gain=1.0,
    integral_gain=0.05,
    derivative_gain=0.2
)
speed_controller.SetDesiredSpeed(desired_speed)

print(f"Speed controller configured (desired speed: {desired_speed} m/s)")
print(f"\nControl parameters:")
print(f"  - Steering delta: {driver.GetSteeringDelta():.3f} rad/s")
print(f"  - Throttle delta: {driver.GetThrottleDelta():.3f} 1/s")
print(f"  - Braking delta: {driver.GetBrakingDelta():.3f} 1/s")









print("\nInitializing Irrlicht visualization...")


vis = irr.CChIrrApp(
    system,
    "Kraz Vehicle Simulation",
    irr.dimension2du(1600, 900),  
    irr.E_WINDOW_ORIENTATION_LANDSCAPE
)


vis.AddTypicalLogo(veh.GetDataPath() + "logo_pychrono_alpha.png")
vis.AddTypicalSky()
vis.AddTypicalLights(
    irr.dimension2df(0.5, 0.5),  
    irr.dimension2df(0.5, 0.5)   
)
vis.AddTypicalCamera(
    irr.vector3df(10, 8, -15),     
    irr.vector3df(0, 2, 0)        
)






camera = vis.GetDevice().getSceneManager().getActiveCamera()
camera.setFOV(1.2)  


camera.setNearClip(0.1)
camera.setFarClip(1000.0)


camera.setInputReceiverEnabled(True)

print("Irrlicht visualization initialized")
print(f"  - Window size: 1600 x 900")
print(f"  - Camera FOV: 1.2 rad")
print(f"  - Clip planes: 0.1 - 1000.0 m")






grid = irr.CChIrrWizard().add_Grid(
    vis.GetDevice(),
    10.0,              
    2.0,               
    50,                
    50,                
    irr.video.SColor(100, 80, 80, 80),  
    True               
)


axes = irr.CChIrrWizard().add_LineAxes(
    vis.GetDevice(),
    10.0,              
    irr.vector3df(0, 0, 0),  
    5.0,               
    2.0                
)






sun_light = vis.GetDevice().getSceneManager().addLightSceneNode()
sun_light.setPosition(irr.vector3df(100, 100, 50))
sun_light.setLightType(irr.E_LIGHT_TYPE.ELT_DIRECTIONAL)
sun_light.setAmbient(irr.video.SColorf(0.3, 0.3, 0.3, 1.0))
sun_light.setDiffuseColor(irr.video.SColorf(1.0, 0.95, 0.8, 1.0))
sun_light.setSpecularColor(irr.video.SColorf(0.5, 0.5, 0.5, 1.0))


fill_light = vis.GetDevice().getSceneManager().addLightSceneNode()
fill_light.setPosition(irr.vector3df(-50, 30, -30))
fill_light.setLightType(irr.E_LIGHT_TYPE.ELT_DIRECTIONAL)
fill_light.setAmbient(irr.video.SColorf(0.15, 0.15, 0.2, 1.0))
fill_light.setDiffuseColor(irr.video.SColorf(0.6, 0.7, 1.0, 1.0))

print("Lighting configured (sun + fill light)")






kraz.GetVehicle().GetSystem().Add(chassis_body)
kraz.AddVisualizationAssets(vis.GetSceneManager())

print("Vehicle visualization added")






shadow_enabled = vis.GetDevice().getSceneManager().addShadowLight(
    irr.vector3df(100, 100, 50),
    irr.video.SColorf(0.5, 0.5, 0.5, 1.0),
    200.0,    
    20.0,     
    100.0,    
    1024      
)

print("Shadow lighting enabled")





print("\n" + "=" * 60)
print("STARTING SIMULATION")
print("=" * 60)


simulation_step = 0.002           
real_time_factor = 1.0            
simulation_duration = 30.0        
output_interval = 1.0             


current_time = 0.0
last_output_time = 0.0
frame_count = 0


time_history = []
speed_history = []
steering_history = []
throttle_history = []
brake_history = []


vis.SetTimestep(simulation_step)
vis.Set罐管理模式(irr.E_UI_SCROLL_BAR.ESBM_VERTICAL)
vis.SetupProxy(vis.GetRenderDevice())


while (vis.GetDevice().run() and current_time < simulation_duration):
    
    
    step_start_time = vis.GetDevice().getTimer().getRealTime()
    
    
    
    
    
    
    
    
    
    steering_input = 0.1 * math.sin(current_time * 0.2)
    
    
    current_speed = chassis_body.GetPos_dt().Length()
    speed_error = desired_speed - current_speed
    throttle_input = min(1.0, max(0.0, speed_error * 0.1))
    
    
    if current_speed > desired_speed * 1.1:
        brake_input = min(1.0, (current_speed - desired_speed) * 0.1)
    else:
        brake_input = 0.0
    
    
    driver.SetSteering(steering_input)
    driver.SetThrottle(throttle_input)
    driver.SetBraking(brake_input)
    
    
    
    
    
    
    kraz.Synchronize(current_time, driver.GetInputs())
    
    
    ground.Synchronize(current_time)
    
    
    driver.Synchronize(current_time)
    
    
    
    
    
    
    kraz.Advance(simulation_step)
    
    
    ground.Advance(simulation_step)
    
    
    driver.Advance(simulation_step)
    
    
    system.DoStepDynamics(simulation_step)
    
    
    
    
    
    
    vis.BeginScene()
    vis.DrawAll()
    
    
    draw_info = irr.CChIrrAppDrawTools(vis.GetDevice())
    
    
    info_text = f
    
    
    draw_info.DrawAll(info_text, irr.vector2d_int32(12, 12), irr.video.SColor(150, 0, 0, 0))
    draw_info.DrawAll(info_text, irr.vector2d_int32(10, 10), irr.video.SColor(255, 255, 255, 255))
    
    
    gauge_position = irr.vector2d_int32(vis.GetDevice().getVideoDriver().getScreenSize().Width - 150, 50)
    draw_info.DrawSpeedGauge(
        "SPEED",
        current_speed * 3.6,     
        0,                       
        120,                     
        gauge_position,
        100,                     
        irr.video.SColor(255, 0, 255, 0),   
        irr.video.SColor(255, 255, 0, 0)    
    )
    
    
    gauge_position2 = irr.vector2d_int32(50, 50)
    draw_info.DrawThrottleBrakeGauges(
        throttle_input,
        brake_input,
        gauge_position2,
        50,
        irr.video.SColor(255, 0, 255, 0)
    )
    
    vis.EndScene()
    
    
    
    
    
    
    if current_time - last_output_time >= output_interval:
        time_history.append(current_time)
        speed_history.append(current_speed)
        steering_history.append(steering_input)
        throttle_history.append(throttle_input)
        brake_history.append(brake_input)
        last_output_time = current_time
        
        
        print(f"[{current_time:6.2f}s] Speed: {current_speed * 3.6:6.1f} km/h | "
              f"Throttle: {throttle_input * 100:5.1f}% | "
              f"Brake: {brake_input * 100:5.1f}% | "
              f"Steering: {steering_input * 100:5.1f}%")
    
    
    current_time += simulation_step
    frame_count += 1
    
    
    
    elapsed = vis.GetDevice().getTimer().getRealTime() - step_start_time
    sleep_time = int((simulation_step * 1000 / real_time_factor - elapsed) * 1000)
    if sleep_time > 0:
        import time
        time.sleep(sleep_time / 1000000.0)





print("\n" + "=" * 60)
print("SIMULATION COMPLETED")
print("=" * 60)


print(f"\nSimulation Statistics:")
print(f"  - Total simulation time: {current_time:.2f} seconds")
print(f"  - Total frames: {frame_count}")
print(f"  - Average FPS: {frame_count / current_time:.1f}")
print(f"  - Average speed: {np.mean(speed_history) * 3.6:.1f} km/h")
print(f"  - Max speed: {np.max(speed_history) * 3.6:.1f} km/h")
print(f"  - Min speed: {np.min(speed_history) * 3.6:.1f} km/h")


import json

data_output = {
    "simulation_parameters": {
        "timestep": simulation_step,
        "duration": simulation_duration,
        "desired_speed": desired_speed,
        "initial_speed": initial_speed
    },
    "time_history": time_history,
    "speed_history": [s * 3.6 for s in speed_history],  
    "steering_history": steering_history,
    "throttle_history": throttle_history,
    "brake_history": brake_history,
    "statistics": {
        "average_speed_kmh": float(np.mean(speed_history) * 3.6),
        "max_speed_kmh": float(np.max(speed_history) * 3.6),
        "min_speed_kmh": float(np.min(speed_history) * 3.6),
        "total_distance_m": float(np.sum(speed_history) * simulation_step)
    }
}


with open("simulation_data.json", "w") as f:
    json.dump(data_output, f, indent=2)

print(f"\nData saved to: simulation_data.json")
print(f"Total distance traveled: {data_output['statistics']['total_distance_m']:.1f} meters")


vis.GetDevice().closeDevice()

print("\nSimulation cleanup complete.")
print("=" * 60)