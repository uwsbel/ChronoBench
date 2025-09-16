import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

# Initialize Chrono system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))

# Create terrain (flat ground)
terrain = veh.FlatTerrain(system)
terrain.SetColor(chrono.ChColor(0.8, 0.8, 0.8))
terrain.SetTexture(chrono.GetChronoDataFile('textures/concrete.jpg'), 200, 200)

# Initialize Viper rover
viper = veh.Viper()
viper.Initialize(terrain, chrono.ChVectorD(0, 0, 0.5), True)  # Position above ground

# Create driver
driver = chrono.ChDriver()
viper.SetDriver(driver)

# Setup Irrlicht visualization
vis = irr.ChIrrApp(system, 'Viper Rover Simulation', irr.dimension2du(800, 600))
vis.SetCamera(chrono.ChVectorD(5, 5, 5))  # Initial camera position
vis.SetCameraLookatPoint(viper.GetChassis().GetPos())
vis.AddAllTypicalLights()
vis.AddLightWithShadow(chrono.ChVectorD(0, 0, 5), chrono.ChVectorD(0, 0, 0), 15, 100, 512)
vis.AssetBind()
vis.AssetUpdate()
vis.SetTimestep(0.01)  # Simulation time step

# Simulation parameters
steer_target = 0.5  # Target steering angle (radians)
steer_duration = 5.0  # Time period for steering change
realtime_factor = 0.01  # Adjust for real-time scaling

# Main simulation loop
while vis.Run():
    current_time = system.GetChTime()
    
    # Gradually change steering over specified time
    if current_time < steer_duration:
        steer = (current_time / steer_duration) * steer_target
    else:
        steer = steer_target
    driver.SetSteering(steer)
    
    # Advance simulation
    system.DoStepDynamics(vis.GetDt())
    
    # Update camera to follow the rover
    chassis_pos = viper.GetChassis().GetPos()
    vis.SetCameraPosition(chassis_pos + chrono.ChVectorD(5, 5, 5))
    vis.SetCameraLookatPoint(chassis_pos)
    
    # Render scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    # Exit on 'Esc' key
    if vis.GetKeyboard().keyPress(irr.KEY_KEY_ESCAPE):
        break

vis.GetSystem().SaveSystemState()