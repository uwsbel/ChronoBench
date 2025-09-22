import pychrono as chrono
import pychrono.robot as viper
import pychrono.irrlicht as chronoirr
import math






system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))  


system.SetSolverMaxIterations(50)
system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)






ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, 0, -0.5))
ground.SetBodyFixed(True)
ground.SetName("ground")


ground_material = chrono.ChMaterialSurfaceNSC()
ground_material.SetFriction(0.8)
ground_material.SetRestitution(0.1)
ground.SetMaterialSurface(ground_material)


system.Add(ground)






rover = viper.ChViper(system)


init_pos = chrono.ChVectorD(0, 0, 0.2)
init_rot = chrono.ChQuaternionD(1, 0, 0, 0)
rover.Initialize(chrono.ChFrameD(init_pos, init_rot))


driver = viper.ChViperDriver()


driver.SetSteering(0.0)      
driver.SetDriving(0.3)       






vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Viper Rover Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(3, 3, 1.5), chrono.ChVectorD(0, 0, 0.5))
vis.AddTypicalLights()


vis.AddLight(chrono.ChVectorD(10, 10, 10), 200, chrono.ChColor(1, 1, 1))
vis.AddLight(chrono.ChVectorD(-10, 10, 10), 200, chrono.ChColor(0.7, 0.7, 0.8))


vis.EnableShadows()






time_step = 1e-3
time_end = 20.0


max_steering_angle = 0.6  
steering_period = 8.0     


render_step = 1.0/50  
render_frame = 0


time = 0

print("Starting Viper rover simulation...")
print(f"Simulation will run for {time_end} seconds")
print("Watch as the rover's steering gradually changes over time!")





while vis.Run() and time < time_end:
    
    
    
    steering_angle = max_steering_angle * math.sin(2 * math.pi * time / steering_period)
    
    
    driver.SetSteering(steering_angle)
    driver.SetDriving(0.3)  
    
    
    driver.Synchronize(time)
    rover.Update()
    
    
    system.DoStepDynamics(time_step)
    
    
    if time >= render_frame * render_step:
        
        rover_pos = rover.GetChassis().GetPos()
        camera_pos = rover_pos + chrono.ChVectorD(3, 3, 1.5)
        target_pos = rover_pos + chrono.ChVectorD(0, 0, 0.5)
        
        vis.GetActiveCamera().setPosition(chronoirr.vector3df(
            camera_pos.x, camera_pos.y, camera_pos.z))
        vis.GetActiveCamera().setTarget(chronoirr.vector3df(
            target_pos.x, target_pos.y, target_pos.z))
        
        
        vis.BeginScene()
        vis.Render()
        
        
        vis.GetGUIEnvironment().getBuiltInFont().draw(
            f"Time: {time:.2f}s",
            chronoirr.recti(10, 10, 200, 30),
            chronoirr.SColor(255, 255, 255, 255))
        
        vis.GetGUIEnvironment().getBuiltInFont().draw(
            f"Steering: {steering_angle:.3f} rad",
            chronoirr.recti(10, 35, 250, 55),
            chronoirr.SColor(255, 255, 255, 255))
        
        vis.GetGUIEnvironment().getBuiltInFont().draw(
            f"Rover Pos: ({rover_pos.x:.2f}, {rover_pos.y:.2f}, {rover_pos.z:.2f})",
            chronoirr.recti(10, 60, 350, 80),
            chronoirr.SColor(255, 255, 255, 255))
        
        vis.EndScene()
        render_frame += 1
    
    
    time += time_step
    
    
    if int(time) % 5 == 0 and time > 0:
        progress = (time / time_end) * 100
        if progress <= 100:
            print(f"Simulation progress: {progress:.1f}%")

print("Simulation completed!")
print("Closing visualization...")


vis.GetDevice().closeDevice()