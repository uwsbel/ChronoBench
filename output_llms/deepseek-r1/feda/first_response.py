import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math

# 1. Initialize PyChrono environment
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))

# 2. Create and initialize the FEDA vehicle
vehicle = veh.FEDA_Vehicle(False, veh.RigidTerrain.ContactMethod_NSC)
vehicle.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1), chrono.Q_from_AngZ(0)))
vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_MESH)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)

# 3. Create and configure rigid terrain
terrain = veh.RigidTerrain(system)
patch = terrain.AddPatch(chrono.ChCoordsysD(), 100, 100)
patch.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 100, 100)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# 4. Initialize Irrlicht visualization
vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("FEDA Vehicle Simulation")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(3, 3, 1), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()

# 5. Set up interactive driver system
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0  # seconds to steer from neutral to full lock
driver.SetSteeringDelta(1.0/(50.0 * steering_time))
driver.SetThrottleDelta(0.5/50.0)
driver.SetBrakingDelta(0.5/50.0)
driver.Initialize()

# 6. Simulation loop parameters
step_size = 0.02
render_steps = math.ceil(1.0 / (50 * step_size))
realtime_timer = chrono.ChRealtimeStepTimer()

# 7. Main simulation loop
while vis.Run():
    time = system.GetChTime()
    
    # Update vehicle and driver inputs
    vehicle.Synchronize(time, driver.GetInputs(), terrain)
    terrain.Synchronize(time)
    driver.Synchronize(time)
    
    # Advance simulation
    system.DoStepDynamics(step_size)
    
    # Update visualization
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    # Maintain real-time synchronization
    realtime_timer.Spin(step_size)

# Cleanup
del vis
del system