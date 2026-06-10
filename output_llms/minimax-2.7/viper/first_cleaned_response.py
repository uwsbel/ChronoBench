import numpy as np
import math






import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr


chrono.SetChronoDataPath(veh.GetChronoDataPath())


output_dir = "VIROVER_SIMULATION_OUTPUT"
import os
if not os.path.exists(output_dir):
    os.makedirs(output_dir)






system = chrono.ChSystemNSC()
system.SetVerbosity(chrono.Verbosity_Verbose)


system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


solver = chrono.ChSolverBB()
solver.SetMaxIterations(100)
solver.SetTolerance(1e-6)
system.SetSolver(solver)


step_size = 1e-3  
t_end = 20.0      






ground_mat = chrono.ChMaterialSurfaceNSC()
ground_mat.SetFriction(0.9)
ground_mat.SetRestitution(0.1)


ground = chrono.ChBody()
ground.SetBodyFixed(True)  
ground.SetPos(chrono.ChVectorD(0, -0.5, 0))
ground.SetMaterialSurface(ground_mat)


ground_shape = chrono.ChCollisionShape()
ground_shape.AddBox(half_extents=chrono.ChVectorD(50, 0.5, 50))  
ground.AddCollisionShape(ground_shape, chrono.ChFrameD())


ground_visual = chrono.ChVisualShapeBox(half_extents=chrono.ChVectorD(50, 0.5, 50))
ground.AddVisualShape(ground_visual)

system.AddBody(ground)






print("Initializing Viper rover...")


viper = veh.Viper rover(vehicle_system=None, chassis_pos=chrono.ChVectorD(0, 0.5, 0))


viper_wheelbase = 0.94       
viper_track = 0.80           
viper_wheel_radius = 0.16    
viper_wheel_width = 0.08     



viper_rocker = veh.ViperRocker(
    system,
    pos=chrono.ChVectorD(0, 0.5, 0),
    chassis_mat=ground_mat,
    wheel_mat=ground_mat
)


viper_rocker.SetChassisColor(chrono.ChColor(0.8, 0.2, 0.2))  
viper_rocker.SetWheelColor(chrono.ChColor(0.1, 0.1, 0.1))   







driver = veh.ChPathFollowerDriver(
    viper_rocker.GetVehicle(),
    veh.GetDataFile("paths/curve_straight.txt"),
    "path_1",
    2.0,    
    2.0     
)


manual_driver = veh.ChDriver(viper_rocker.GetVehicle())


throttle = 0.0
steering = 0.0
braking = 0.0






print("Creating Irrlicht visualization...")
app = irr.ChIrrApp(
    system,
    "Viper Rover Simulation",
    irr.dimension2du(1280, 720),  
    irr.EWI_PERSPECTIVE           
)


app.AddTypicalCamera(
    irr.vector3df(5, 4, -5),      
    irr.vector3df(0, 0, 0)        
)


app.AddTypicalLight(
    irr.vector3df(10, 20, 10),    
    irr.SColorf(1.0, 1.0, 1.0)   
)


app.AddLightWithShadow(
    irr.vector3df(15, 20, -10),
    irr.vector3df(0, 0, 0),
    30,                            
    1, 40,                         
    irr.SColorf(1.0, 1.0, 0.9)
)


app.SetWindowTitle("PyChrono - Viper Rover Simulation")
app.SetShowDemoInfo(True)         
app.SetShowProfiler(True)         





print("Starting simulation loop...")


time = 0.0
frame = 0
last_time = 0.0
steer_direction = 1  


app.SetTimestep(step_size)
app.Start()  


while app.GetDevice().run():
    
    time = system.GetChTime()
    
    
    if time < 5.0:
        
        steering = 0.0
        throttle = 0.5
    elif time < 10.0:
        
        steering = steer_direction * min((time - 5.0) / 2.0, 0.4)
        throttle = 0.3
    elif time < 15.0:
        
        steering = steer_direction * 0.4
        throttle = 0.4
    elif time < 20.0:
        
        steering = steer_direction * (0.4 - (time - 15.0) / 2.5 * 0.4)
        throttle = 0.3
    else:
        
        steering = 0.0
        throttle = 0.0
        braking = 0.5
    
    
    manual_driver.SetSteering(steering)
    manual_driver.SetThrottle(throttle)
    manual_driver.SetBraking(braking)
    manual_driver.Synchronize(time)
    
    
    viper_rocker.Synchronize(time, manual_driver)
    viper_rocker.Update()
    
    
    system.DoStepDynamics(step_size)
    
    
    app.BeginScene()
    app.DrawAll()
    app.EndScene()
    
    
    if time - last_time >= 2.0:
        print(f"Time: {time:.2f}s | Steering: {steering:.3f} | "
              f"Throttle: {throttle:.2f} | "
              f"Position: {viper_rocker.GetVehicle().GetChassis().GetPos()}")
        last_time = time
    
    frame += 1





print("\nSimulation completed!")
print(f"Total frames: {frame}")
print(f"Final time: {time:.2f}s")


trajectory_file = os.path.join(output_dir, "viper_trajectory.txt")
with open(trajectory_file, 'w') as f:
    f.write("
    f.write("
    
    f.write(f"{time:.4f}\t0.0\t0.0\t0.0\t0.0\n")

print(f"Results saved to: {output_dir}")