import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.terrain as terr
import math

# --- Simulation setup ---
sys = chrono.ChSystemSMC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
time_step = 0.005

# --- Set the path to the Chrono data directory ---
chrono.SetChronoDataPath("./chrono_data/")  # Replace with your actual path

# --- Visualization setup ---
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('HMMWV on Complex Terrain')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(10, 10, -15))
vis.AddTypicalLights()

# --- Terrain setup ---
terrain = terr.RigidTerrain(sys)

# Flat patches
patch1 = terrain.AddPatch(veh.PatchMaterialData(0.5, 0.8, 0.2), chrono.ChCoordsysd(chrono.ChVector3d(-10, 0, 0), chrono.Q_from_Ang3(0, 0, 0)), 10, 10)
patch2 = terrain.AddPatch(veh.PatchMaterialData(0.8, 0.5, 0.2), chrono.ChCoordsysd(chrono.ChVector3d(0, 0, -10), chrono.Q_from_Ang3(0, 0, 0)), 10, 10)

# Mesh-based bump
mesh_data = chrono.GetChronoDataFile("terrain/bump.obj")
bump_patch = terrain.AddPatch(veh.PatchMaterialData(0.2, 0.8, 0.5), chrono.ChCoordsysd(chrono.ChVector3d(10, 0, 0), chrono.Q_from_Ang3(0, 0, 0)), 5, 5, mesh_data)

# Heightmap patch
heightmap_data = chrono.GetChronoDataFile("terrain/heightmap.png")
heightmap_patch = terrain.AddPatch(veh.PatchMaterialData(0.7, 0.3, 0.1), chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 10), chrono.Q_from_Ang3(0, 0, 0)), 10, 10, heightmap_data)

terrain.Initialize()

# --- HMMWV Vehicle setup ---
hmmwv = veh.HMMWV(sys)
hmmwv.SetVehicleModel(veh.VehicleModel.HMMWV)
hmmwv.SetEngineType(veh.EngineModel.FOUR_STROKE)
hmmwv.SetDrivetrainType(veh.DrivetrainModel.FOUR_WHEEL_DRIVE)
hmmwv.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 1, 0), chrono.Q_from_Ang3(0, 0, 0)))
hmmwv.SetChassisCollisionType(veh.ChassisCollisionType.BOX)
hmmwv.Initialize()

# Ensure mesh visualization for all vehicle components
for i in range(hmmwv.GetNumWheels()):
    wheel = hmmwv.GetWheel(i)
    wheel.SetVisualize(True)

hmmwv.GetChassis().SetVisualize(True)

# --- Driver System ---
driver = veh.HMMWV_SimpleDriver(hmmwv)
driver.SetSteeringDelta(0.01)
driver.SetThrottleDelta(0.01)
driver.SetBrakingDelta(0.01)

# --- Simulation loop ---
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Update driver input (example: constant throttle)
    driver.SetThrottle(0.5)
    driver.SetSteering(0.0)
    driver.SetBraking(0.0)

    sys.DoStepDynamics(time_step)