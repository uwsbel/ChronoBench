import os
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

# 1. Initialize Chrono environment
system = chrono.ChSystemSMC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravity(chrono.ChVectorD(0, 0, -9.81))

# 2. Create custom mesh terrain
terrain = veh.TerrainMesh(system)
terrain.SetMeshFilename(chrono.GetChronoDataFile("models/terrain/Highway_col.obj"))  # Collision mesh
terrain.SetVisualMeshFilename(chrono.GetChronoDataFile("models/terrain/Highway_vis.obj"))  # Visual mesh
terrain.SetContactFrictionCoefficient(0.8)  # Adjust friction as needed
terrain.Initialize()

# 3. Initialize HMMWV vehicle
hmmwv = veh.HMMWV_Full(system)
hmmwv.SetChassisFixed(False)  # Allow chassis movement
hmmwv.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.5), chrono.QUNIT))
hmmwv.SetTireType(veh.TireModelType_TMEASY)  # Use TMEASY tire model
hmmwv.SetTireStepSize(0.001)  # Simulation step for tires
hmmwv.Initialize()

# Set visualization modes
hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

# 4. Create interactive driver
driver = veh.ChIrrGuiDriver(irr.irrlicht)  # Will be linked to Irrlicht later

# 5. Set up Irrlicht visualization
vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("HMMWV on Custom Terrain")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(6, 6, 1.5), chrono.ChVectorD(0, 0, 0.5))
vis.AddTypicalLights()

# Link driver to visualization
driver.SetInputMode(veh.InputMode_KEYBOARD)  # Enable keyboard controls
driver.Initialize()

# 6. Simulation loop setup
step_size = 0.002
frame_interval = 0.02  # Target 50 FPS (1/50 = 0.02 seconds)
realtime_timer = chrono.ChRealtimeStepTimer()
frame_number = 0

while vis.Run():
    time = system.GetChTime()
    
    # Render scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    # Process driver inputs
    driver_inputs = driver.GetInputs()
    hmmwv.GetVehicle().Synchronize(time, driver_inputs, terrain)
    
    # Advance simulation
    driver.Advance(step_size)
    system.DoStepDynamics(step_size)
    
    # Maintain real-time pacing
    realtime_timer.Spin(step_size)
    
    # Control frame rate
    if frame_number % int(frame_interval / step_size) == 0:
        vis.Render()
        frame_number = 0
    frame_number += 1