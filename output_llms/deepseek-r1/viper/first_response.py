import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

# 1. Initialize Chrono system with gravity
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))

# 2. Create rigid terrain
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChMaterialSurfaceNSC()
patch = terrain.AddPatch(patch_mat, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 1), 100, 100)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
terrain.Initialize()

# 3. Initialize Viper rover
viper = veh.Viper(system)
viper.SetDriverPos(chrono.ChVectorD(0, 0, 1))
viper.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, -0.2, 0.5), chrono.QUNIT))

# 4. Create and initialize driver
driver = veh.ViperDCMotorControl()
driver.SetGains(0.4, 0, 0)
viper.SetDriver(driver)

# 5. Setup Irrlicht visualization
vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Viper Rover Simulation")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(3, 3, 1), chrono.ChVectorD(0, 0, 0.5))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVectorD(5, 0, 5), chrono.ChVectorD(0, 0, 0.5), 10, 2, 10, 40, 512)

# 6. Simulation parameters
step_size = 0.005
t_end = 30
t_steer_start = 5
t_steer_end = 25
steering_max = 0.4  # radians (~23 degrees)
current_time = 0

# 7. Main simulation loop
while vis.Run() and current_time < t_end:
    # Update steering based on time
    if t_steer_start <= current_time <= t_steer_end:
        t_normalized = (current_time - t_steer_start) / (t_steer_end - t_steer_start)
        steering = steering_max * chrono.ChSineStep(t_normalized, 0.5)  # Smooth ramp
        driver.SetSteering(steering)
    
    # Update systems
    viper.Update()
    terrain.Update()
    
    # Advance simulation
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(step_size)
    current_time += step_size