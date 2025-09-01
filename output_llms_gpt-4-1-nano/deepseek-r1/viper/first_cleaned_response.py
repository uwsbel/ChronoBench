import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


terrain = veh.RigidTerrain(system, chrono.ChContactMaterialNSC())
patch = terrain.AddPatch(chrono.ChVectorD(0, 0, 0), 
                        chrono.ChVectorD(0, 1, 0), 
                        100, 100)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
terrain.Initialize()


rover = veh.Viper(system)
rover.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0), 
                                   chrono.Q_from_AngZ(0)))
rover.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
rover.SetWheelVisualizationType(veh.VisualizationType_PRIMITIVES)


driver = veh.ViperDCMotorControl()
driver.SetGains(0.5, 0.0, 0.0)
rover.SetDriver(driver)


vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Viper Rover Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(3, 2, 3), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()


step_size = 0.005
render_step = 1.0 / 50  
sim_time = 0
steering_duration = 5.0  
start_steering_time = 2.0
max_steering = 0.5


while vis.Run():
    time = system.GetChTime()
    
    
    if time > start_steering_time:
        t_steer = time - start_steering_time
        steering = min(t_steer / steering_duration, 1.0) * max_steering
        driver.SetSteering(steering)
    
    
    driver.SetTargetSpeed(1.0)  
    driver.Update(step_size)
    
    
    if time % render_step < step_size:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
    
    
    rover.Update()
    system.DoStepDynamics(step_size)
    sim_time += step_size

print("Simulation completed.")