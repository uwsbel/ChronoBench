import sys
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

# Create the HMMWV vehicle
vehicle = veh.CreateHMMWV()
vehicle.SetInitPosition(chrono.ChVectorD(0, 0, 1.0))  # Position above terrain
vehicle.SetInitFwdVel(5.0)  # Initial forward velocity

# Define soil parameters for SCM terrain
shear_modulus = 1e7  # Pa
damping_modulus = 1e5  # Pa
friction = 0.8

# Create a flat heightmap surface
heightmap = chrono.ChHeightmapSurface()
heightmap.SetSize(100.0, 100.0)  # Terrain size (meters)
heightmap.SetResolution(100, 100)  # Number of points in x and y
heightmap_data = [0.0] * (100 * 100)  # Flat terrain (all zeros)
heightmap.SetHeightData(heightmap_data, 100, 100, 1.0, 1.0)  # 1m spacing between points

# Initialize SCM terrain
terrain = veh.CreateSCMTerrain(vehicle.GetSystem(), shear_modulus, damping_modulus, friction, heightmap)

# Initialize driver system
driver = vehicle.GetDriver()
driver.SetThrottle(0.8)  # Constant throttle (80%)
driver.SetSteering(0.0)  # Straight direction

# Visualization setup
visualizator = irr.ChIrrApp(vehicle.GetSystem(), 'HMMWV on SCM Terrain', irr.dimension2du(1280, 720))
visualizator.AddTypicalLights()
visualizator.AddTypicalCamera(chrono.ChVectorD(0, 0, 3), chrono.ChVectorD(0, 0, 0))  # Camera position and lookat
visualizator.SetCameraVertical(chrono.ChVectorD(0, 1, 0))  # Camera up direction
visualizator.SetSymbolscale(0.01)
visualizator.SetChaseCamera(vehicle.GetChassis(), 6.0, 0.5)  # Chase camera parameters

# Initialize visualization
visualizator.Initialize()
visualizator.AddAllVisualization()

# Simulation settings
step_size = 0.01  # Time step
visualizator.SetTimestep(step_size)

# Simulation loop
while visualizator.Run():
    visualizator.BeginScene()
    visualizator.DrawAll()
    
    # Update driver inputs (current time synchronization)
    current_time = vehicle.GetSystem().GetChTime()
    driver.Synchronize(current_time)
    
    # Advance simulation
    vehicle.GetSystem().DoStepDynamics(step_size)
    
    # Update visualization
    visualizator.Advance()
    
    visualizator.EndScene()