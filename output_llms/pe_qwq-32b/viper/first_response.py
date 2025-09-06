import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import math

# 1. Initialize simulation environment
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Gravity vector

# 2. Create rigid terrain
terrain_mat = chrono.ChMaterialSurfaceNSC()
terrain_mat.SetFriction(0.9)
terrain_mat.SetRestitution(0.0)

terrain = veh.RigidTerrain(system)
patch = terrain.AddPatch(terrain_mat, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0)), 100, 100)
terrain.Initialize()

# 3. Initialize Viper rover
viper = veh.Viper(vehicle_system=system)
init_pos = chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0), chrono.QUNIT)
viper.SetInitPosition(init_pos)
viper.Initialize()

# Create and configure driver
driver = veh.RoverDriver()  # Assuming valid driver class exists
viper.SetDriver(driver)

# 4. Visualization setup
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Viper Rover Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.SetCamera(chronoirr.ChVectorD(0, 5, 20))  # Position camera
vis.SetCameraRotation(chronoirr.ChQuaternionD(1,0,0,0))  # Look towards origin
vis.AddTypicalLights()

# Simulation parameters
time_step = 0.005
max_steering = 0.5  # Maximum steering angle in radians
steering_period = 10.0  # Time period for steering change

# 5. Simulation loop
while vis.Run():
    current_time = system.GetChTime()
    
    # Gradually change steering over specified period
    if current_time < steering_period:
        steering = max_steering * (current_time / steering_period)
    else:
        steering = max_steering
    
    # Update driver inputs
    driver.SetSteering(steering)
    driver.SetThrottle(0.5)  # Maintain constant throttle
    
    # Advance simulation
    system.DoStepDynamics(time_step)
    
    # Update visualization
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

vis.Close()