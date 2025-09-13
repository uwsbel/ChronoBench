import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


terrain = veh.RigidTerrain(system)
terrain_patch = terrain.AddPatch(
    chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT),
    chrono.ChVectorD(100, 1, 100)  
)
terrain_patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 100, 100)
terrain.Initialize()


rover = veh.Viper(system)
rover.SetDriverPos(chrono.ChVectorD(0, 0, 0))  
rover.Initialize(veh.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0), chrono.Q_from_AngZ(0)))


driver = veh.ViperDCMotorControl()
driver.SetGains(0.5, 0, 0)  
rover.SetDriver(driver)


vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Viper Rover Steering Demo")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(2, 1.5, 2), chrono.ChVectorD(0, 0.5, 0))  
vis.AddTypicalLights(chrono.ChVectorD(50, 50, 50), chrono.ChVectorD(-50, -50, 50))  


step_size = 0.005
render_step = 1.0 / 50  
sim_time = 0
steering_duration = 10.0  
max_steering = 0.3       


while vis.Run():
    
    steering_angle = max_steering * chrono.ChSineStep(sim_time, 1.0, 0.0, steering_duration, 0.0)
    driver.SetSteering(steering_angle)
    
    
    rover.Update(sim_time, step_size, steering_angle, 0.5)  
    
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(step_size)
    
    
    sim_time += step_size
    
    
    if sim_time >= steering_duration:
        break

print("Simulation completed successfully.")