import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import math

# ======================
# Simulation Parameters
# ======================
TIME_STEP = 0.02  # 50 FPS (1/0.02 = 50)
SIM_TIME = 30.0    # Total simulation time in seconds

# Vehicle initialization parameters
INIT_POS = chrono.ChVector3d(0, 0.5, 0)  # Starting position (y=0.5m above ground)
INIT_ROT = chrono.QuatFromAngleY(math.pi/4)  # 45-degree rotation

# Terrain parameters
TERRAIN_LENGTH = 100.0  # meters
TERRAIN_WIDTH = 100.0   # meters

# ======================
# System Initialization
# ======================
system = chrono.ChSystemSMC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

# ======================
# Vehicle Setup
# ======================
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysd(INIT_POS, INIT_ROT))
hmmwv.SetTireType(veh.TireModelType_TMEASY)
hmmwv.SetTireStepSize(TIME_STEP)
hmmwv.Initialize()

# Set visualization modes
hmmwv.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetTireVisualizationType(veh.VisualizationType_PRIMITIVES)

# Get vehicle systems for later reference
vehicle = hmmwv.GetVehicle()
tires = [hmmwv.GetTire(i) for i in range(4)]

# ======================
# Terrain Setup
# ======================
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialSMC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 
                         TERRAIN_LENGTH, TERRAIN_WIDTH)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
terrain.Initialize()

# ======================
# Visualization Setup
# ======================
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("HMMWV Simulation")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 2, -10))
vis.AddTypicalLights()

# Attach vehicle-specific visualization
vis.AssetBindAll()
vis.AssetUpdateAll()

# ======================
# Driver System Setup
# ======================
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(0.02)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.06)
driver.Initialize()

# ======================
# Simulation Loop
# ======================
realtime_timer = chrono.ChRealtimeStepTimer()

while vis.Run() and vehicle.GetChTime() < SIM_TIME:
    time = vehicle.GetChTime()
    
    # Render scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    # Get driver inputs
    driver_inputs = driver.GetInputs()
    
    # Update vehicle systems
    driver.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    terrain.Synchronize(time)
    
    # Advance dynamics
    system.DoStepDynamics(TIME_STEP)
    
    # Update visualization
    vis.Synchronize(time, driver_inputs)
    vehicle.Advance(TIME_STEP)
    terrain.Advance(TIME_STEP)
    
    # Maintain real-time simulation
    realtime_timer.Spin(TIME_STEP)

print("Simulation completed successfully.")