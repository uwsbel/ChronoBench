import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh

# Initialize the PyChrono system
sys = chrono.ChSystemSMC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Set the path to the Chrono data directory
chrono.SetChronoDataPath("./chrono_data/")  # Replace with your actual data path

# --- Vehicle Setup ---
# Create the ARTcar vehicle
car = veh.ARTcar()
car.SetContactMethod(veh.ChContactMethod.SMC)
car.SetChassisCollisionType(veh.ChChassis.CollisionType.BOX)
initLoc = chrono.ChVector3d(0, 0, 0)
initRot = chrono.Q_from_AngAxis(0, chrono.ChVector3d(0, 1, 0))  # No initial rotation
car.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
car.Initialize()
sys.Add(car.GetVehicle())

# --- Terrain Setup ---
# Create the RigidTerrain
terrain = veh.RigidTerrain(sys)
terrainLength = 100
terrainWidth = 100
terrainHeight = 10
patch = terrain.AddPatch(veh.ChMaterialSurfaceSMC(),
                         chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
                         terrainLength, terrainWidth)
patch.SetTexture(chrono.GetChronoDataFile("textures/terrain/grass.png"), 20, 20)
terrain.Initialize()
sys.Add(terrain)

# --- Driver System ---
# Create an interactive driver system
driver = veh.ARTcarDriver(car)
driver.SetSteeringDelta(20.0)
driver.SetThrottleDelta(0.1)
driver.SetBrakingDelta(0.1)

# --- Visualization Setup ---
# Create the Irrlicht visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('ARTcar Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(50, 10, -50))
vis.AddTypicalLights()

# --- Simulation Loop ---
# Simulation time step
time_step = 0.01

while vis.Run():
    vis.BeginScene(True, True, chrono.ChColor(0.2, 0.2, 0.3))
    vis.Render()
    vis.EndScene()

    # Update the vehicle and terrain
    sys.DoStepDynamics(time_step)

    # Get driver inputs (example: keyboard input)
    if vis.GetKeyboard().GetKeyDown(chrono.ChVirtualKeys.W):
        driver.SetThrottle(1.0)
    elif vis.GetKeyboard().GetKeyDown(chrono.ChVirtualKeys.S):
        driver.SetBraking(1.0)
    else:
        driver.SetThrottle(0.0)
        driver.SetBraking(0.0)

    if vis.GetKeyboard().GetKeyDown(chrono.ChVirtualKeys.A):
        driver.SetSteering(0.5)  # Steer left
    elif vis.GetKeyboard().GetKeyDown(chrono.ChVirtualKeys.D):
        driver.SetSteering(-0.5)  # Steer right
    else:
        driver.SetSteering(0.0)

    driver.Synchronize(time_step)