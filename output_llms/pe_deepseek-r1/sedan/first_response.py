import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import os

# =====================
# Simulation Parameters
# =====================
# Vehicle parameters
init_pos = chrono.ChVectorD(0, 0.5, 0)  # Initial position (above terrain)
init_rot = chrono.QUNIT  # Initial rotation (no rotation)

# Terrain parameters
terrain_length = 100  # meters
terrain_width = 8     # meters
terrain_height = 0.2  # meters
friction_coeff = 0.8  # Terrain friction coefficient

# Visualization parameters
window_width = 1280
window_height = 720
camera_pos = chrono.ChVectorD(0, 3, -6)  # Chase camera position

# =================
# System Setup
# =================
# Create Chrono system
system = chrono.ChSystemSMC()
system.SetGravity(chrono.ChVectorD(0, -9.81, 0))

# Set data path
chrono.SetChronoDataPath(os.path.join(os.environ.get('CHRONO_DATA_DIR', ''), 'data/'))
vehicle_data = veh.VehicleData(chrono.GetChronoDataFile("vehicle/BMW_E90/BMW_E90.json"))

# ==================
# Vehicle Creation
# ==================
# Create BMW E90 vehicle
bmw = veh.WheeledVehicle(system, vehicle_data)
bmw.Initialize(chrono.ChCoordsysD(init_pos, init_rot))
bmw.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
bmw.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
bmw.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
bmw.SetWheelVisualizationType(veh.VisualizationType_MESH)

# Create and initialize powertrain
powertrain = veh.SimplePowertrain(chrono.GetChronoDataFile("vehicle/BMW_E90/powertrain.json"))
bmw.InitializePowertrain(powertrain)

# Create and initialize tires (TMEASY model)
tire_fl = veh.TMeasyTire(chrono.GetChronoDataFile("vehicle/BMW_E90/tire.json"))
tire_fr = veh.TMeasyTire(chrono.GetChronoDataFile("vehicle/BMW_E90/tire.json"))
tire_rl = veh.TMeasyTire(chrono.GetChronoDataFile("vehicle/BMW_E90/tire.json"))
tire_rr = veh.TMeasyTire(chrono.GetChronoDataFile("vehicle/BMW_E90/tire.json"))

bmw.InitializeTire(tire_fl, bmw.GetAxle(0).m_wheels[0], veh.VisualizationType_MESH)
bmw.InitializeTire(tire_fr, bmw.GetAxle(0).m_wheels[1], veh.VisualizationType_MESH)
bmw.InitializeTire(tire_rl, bmw.GetAxle(1).m_wheels[0], veh.VisualizationType_MESH)
bmw.InitializeTire(tire_rr, bmw.GetAxle(1).m_wheels[1], veh.VisualizationType_MESH)

# ================
# Terrain Creation
# ================
# Create rigid terrain
terrain = veh.RigidTerrain(system, chrono.GetChronoDataFile("terrain/RigidPlane.json"))
patch_mat = chrono.ChContactMaterialSMC()
patch_mat.SetFriction(friction_coeff)
patch_mat.SetRestitution(0.01)

terrain.AddPatch(patch_mat, chrono.ChCoordsysD(), terrain_length, terrain_width)
terrain.SetColor(chrono.ChColor(0.5, 0.6, 0.2))  # Grass-like color
terrain.SetTexture(chrono.GetChronoDataFile("textures/grass.jpg"), terrain_length, terrain_width)
terrain.Initialize()

# ==================
# Driver System Setup
# ==================
# Interactive driver for keyboard control
driver = veh.ChInteractiveDriverIRR(system)
driver.SetSteeringDelta(0.02)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.06)

# ===================
# Visualization Setup
# ===================
# Create Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(window_width, window_height)
vis.SetWindowTitle("BMW E90 Driving Simulation")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(camera_pos)
vis.AddTypicalLights()

# Add chase camera (follows vehicle)
chase_cam = veh.ChChaseCamera(vis.GetCamera(), bmw.GetVehicle())
chase_cam.SetState(-6, 0.5, 0.5)
vis.SetCameraChase(chase_cam)

# ===================
# Simulation Loop
# ===================
# Simulation parameters
time_step = 0.001
render_step = 1.0 / 50  # FPS = 50

# Initialize real-time simulation
driver.Initialize()

while vis.Run():
    # Get simulation time
    time = system.GetChTime()
    
    # Update driver inputs
    driver.Synchronize(time)
    
    # Update vehicle systems
    bmw.Synchronize(time, driver.GetInputs(), terrain)
    terrain.Synchronize(time)
    
    # Advance simulation
    system.DoStepDynamics(time_step)
    
    # Update visualization
    if time >= 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
    
    # Render at specified FPS
    if time_step < render_step:
        chrono.ChRealtimeStepTimer.Spin(render_step)

print("Simulation completed.")