import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

# 1. Initialize Chrono system and core components
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Gravity

# 2. Create rigid terrain
terrain = veh.RigidTerrain(system)
terrain_patch = terrain.AddPatch(
    chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT),
    chrono.ChVectorD(100, 1, 100)  # Dimensions
)
terrain_patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 100, 100)
terrain.Initialize()

# 3. Initialize Viper rover
rover = veh.Viper(system)
rover.SetDriverPos(chrono.ChVectorD(0, 0, 0))  # Initial position
rover.Initialize(veh.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0), chrono.Q_from_AngZ(0)))

# 4. Configure driver system with steering control
driver = veh.ViperDCMotorControl()
driver.SetGains(0.5, 0, 0)  # PID gains for steering
rover.SetDriver(driver)

# 5. Create Irrlicht visualization
vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Viper Rover Steering Demo")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(2, 1.5, 2), chrono.ChVectorD(0, 0.5, 0))  # Position & target
vis.AddTypicalLights(chrono.ChVectorD(50, 50, 50), chrono.ChVectorD(-50, -50, 50))  # Key lights

# 6. Simulation parameters
step_size = 0.005
render_step = 1.0 / 50  # 50 FPS
sim_time = 0
steering_duration = 10.0  # Total steering period
max_steering = 0.3       # Max steering angle (radians)

# 7. Simulation loop
while vis.Run():
    # Update steering based on simulation time
    steering_angle = max_steering * chrono.ChSineStep(sim_time, 1.0, 0.0, steering_duration, 0.0)
    driver.SetSteering(steering_angle)
    
    # Update rover and driver
    rover.Update(sim_time, step_size, steering_angle, 0.5)  # Constant speed 0.5 m/s
    
    # Advance simulation
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(step_size)
    
    # Increment simulation time
    sim_time += step_size
    
    # Exit after completing steering cycle
    if sim_time >= steering_duration:
        break

print("Simulation completed successfully.")