import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# 1. Initialize the Chrono system and environment
system = chrono.ChSystemSMC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
chrono.SetChronoDataPath(chrono.GetChronoDataPath())

# 2. Create and initialize the FEDA vehicle
vehicle = veh.FEDA()
vehicle.SetContactMethod(chrono.ChContactMethod_SMC)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(
    chrono.ChVector3d(0, 0, 0.5),  # Initial position (0.5m above ground)
    chrono.Q_from_AngZ(0)           # Initial orientation (facing forward)
))
vehicle.SetTireType(veh.TireModelType_TMEASY)  # Tire model selection
vehicle.Initialize()

# Set visualization modes
vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_MESH)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)

# 3. Create and configure the rigid terrain
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialSMC()
patch_mat.SetFriction(0.8)  # High friction for off-road terrain
patch_mat.SetRestitution(0.01)

patch = terrain.AddPatch(patch_mat, 
                        chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
                        100, 50)  # 100m x 50m terrain patch
patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 100, 50)  # Custom texture
terrain.Initialize()

# 4. Set up the Irrlicht visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("FEDA Vehicle Simulation")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddTypicalLights()
vis.AddSkyBox()

# Set camera to follow vehicle from rear position
vis.AddCamera(chrono.ChVector3d(-5, 0, 2), chrono.ChVector3d(0, 0, 0.5))

# 5. Create interactive driver system
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(0.02)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.06)
driver.Initialize()

# 6. Simulation timing parameters
step_size = 0.002
render_step = 1.0 / 50  # 50 FPS rendering
frame_number = 0

# 7. Main simulation loop
while vis.Run():
    time = system.GetChTime()
    
    # Update vehicle systems
    driver.Synchronize(time)
    vehicle.Synchronize(time, driver.GetInputs(), terrain)
    terrain.Synchronize(time)
    
    # Advance simulation state
    system.DoStepDynamics(step_size)
    
    # Update visualization
    if frame_number % int(render_step / step_size) == 0:
        vis.BeginScene()
        vis.Render()
        vis.RenderFrame(vehicle.GetChassisBody().GetFrame_REF_to_abs())
        vis.EndScene()
        
        # Display vehicle stats
        pos = vehicle.GetPos()
        speed = vehicle.GetSpeed()
        print(f"Time: {time:.2f}s | Position: ({pos.x:.1f}, {pos.y:.1f}, {pos.z:.1f}) | Speed: {speed * 3.6:.1f} km/h")
    
    frame_number += 1