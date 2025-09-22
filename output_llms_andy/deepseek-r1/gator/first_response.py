import os
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

# Set paths to Chrono data and vehicle data
chrono.SetChronoDataPath(os.path.join(os.environ.get('CHRONO_DATA_DIR', ''), 'data/'))
veh.SetDataPath(os.path.join(os.environ.get('CHRONO_DATA_DIR', ''), 'vehicle/'))

# Initialize simulation system
system = chrono.ChSystemSMC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create and initialize Gator vehicle
gator = veh.Gator(system)
gator.SetContactMethod(chrono.ChContactMethod_SMC)
gator.SetChassisFixed(False)
gator.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0), chrono.QUNIT))
gator.SetTireType(veh.TireModelType_TMEASY)
gator.Initialize()

# Set visualization modes
gator.SetChassisVisualizationType(veh.VisualizationType_MESH)
gator.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
gator.SetSteeringVisualizationType(veh.VisualizationType_MESH)
gator.SetWheelVisualizationType(veh.VisualizationType_MESH)
gator.SetTireVisualizationType(veh.VisualizationType_MESH)

# Create rigid terrain
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChMaterialSurfaceSMC()
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysD(), 100, 100)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
terrain.Initialize()

# Create Irrlicht visualization
vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Gator Vehicle Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(6, 3, 6), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()

# Create interactive driver
driver = veh.ChInteractiveDriverIRR(vis)
steering_controller = veh.ChSteeringControllerClosedLoop()
driver.SetSteeringController(steering_controller)
driver.SetGains(0.5, 0, 0)
driver.Initialize()

# Simulation parameters
step_size = 0.01
render_step = 1.0 / 50  # 50 FPS
time = 0.0
realtime_timer = chrono.ChRealtimeStepTimer()

# Simulation loop
while vis.Run():
    time = system.GetChTime()
    
    # Update driver inputs
    driver.Synchronize(time)
    
    # Update vehicle
    gator.Synchronize(time, driver.GetInputs(), terrain)
    
    # Advance dynamics
    system.DoStepDynamics(step_size)
    
    # Update visualization
    vis.BeginScene(True, True, irr.SColor(255, 140, 161, 192))
    vis.Render()
    vis.Synchronize(time, driver.GetInputs())
    vis.EndScene()
    
    # Maintain real-time simulation
    realtime_timer.Spin(step_size)

print("Simulation completed.")