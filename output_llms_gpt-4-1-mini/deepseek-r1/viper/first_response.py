import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

# 1. Initialize Chrono system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# 2. Create rigid terrain
terrain = veh.RigidTerrain(system, chrono.ChContactMaterialNSC())
patch = terrain.AddPatch(chrono.ChVectorD(0, 0, 0), 
                        chrono.ChVectorD(0, 1, 0), 
                        100, 100)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
terrain.Initialize()

# 3. Initialize Viper rover
rover = veh.Viper(system)
rover.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0), 
                                   chrono.Q_from_AngZ(0)))
rover.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
rover.SetWheelVisualizationType(veh.VisualizationType_PRIMITIVES)

# 4. Create and initialize driver
driver = veh.ViperDCMotorControl()
driver.SetGains(0.5, 0.0, 0.0)
rover.SetDriver(driver)

# 5. Set up Irrlicht visualization
vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Viper Rover Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(3, 2, 3), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()

# 6. Simulation parameters
step_size = 0.005
render_step = 1.0 / 50  # 50 Hz rendering
sim_time = 0
steering_duration = 5.0  # Time to reach max steering
start_steering_time = 2.0
max_steering = 0.5

# 7. Main simulation loop
while vis.Run():
    time = system.GetChTime()
    
    # Update steering gradually
    if time > start_steering_time:
        t_steer = time - start_steering_time
        steering = min(t_steer / steering_duration, 1.0) * max_steering
        driver.SetSteering(steering)
    
    # Update driver inputs
    driver.SetTargetSpeed(1.0)  # Constant forward speed
    driver.Update(step_size)
    
    # Render scene
    if time % render_step < step_size:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
    
    # Advance simulation
    rover.Update()
    system.DoStepDynamics(step_size)
    sim_time += step_size

print("Simulation completed.")