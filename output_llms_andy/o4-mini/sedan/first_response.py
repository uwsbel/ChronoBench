import os
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

# --------------------------------------------------------------------------
# 1) INITIALIZATION
# --------------------------------------------------------------------------

# 1) Set the Chrono data path (adjust this to your Chrono data folder)
chrono.SetChronoDataPath(os.getenv('CHRONO_DATA_DIR', '/usr/local/share/chrono/data/'))

# 2) Create the physics system (SMC contact)
system = chrono.ChSystemSMC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))

# 3) Create the vehicle:
#    Here we load a 4-wheeled “GenericSedan” from JSON as a stand-in for the BMW E90.
#    If you have your own E90 JSON files, just replace the paths below.
vehicle = veh.WheeledVehicle(
    system,
    chrono.GetChronoDataFile("vehicle/GenericSedan/Sedan_FullCoord.json"),
    veh.CollisionType_PRIMITIVES)

vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)

# 4) Initialize vehicle at the origin, with zero initial forward speed.
init_loc = chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.5), chrono.QUNIT)
vehicle.Initialize(init_loc)
vehicle.SetChassisFixed(False)

# 5) Create and attach TMeasy tire models (one per wheel)
tire_vis = veh.VisualizationType_MESH
tire_collision = veh.ChTire.TerrainType_SPHERE  # no tire-body collision
tire_mesh = chrono.GetChronoDataFile("vehicle/tire/hmmwv_tire_tmeasy/meshes/wheel.obj")

for i in range(vehicle.GetNumTires()):
    tire = veh.TMeasyTire()
    # Load default TMeasy parameters from JSON
    tire.SetTMeasyParams(chrono.GetChronoDataFile("vehicle/tire/TMeasyTire.json"))
    tire.SetVisualizationType(tire_vis)
    vehicle.InitializeTire(tire, i, tire_collision)

# 6) Create the rigid terrain
terrain = veh.RigidTerrain(system)
patch = terrain.AddPatch(
    chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT),
    chrono.ChVectorD(20, 20, 1))
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
patch.SetTexture(chrono.GetChronoDataFile("vehicle/terrain/textures/tile4.jpg"), 8, 8)
terrain.Initialize()

# 7) Create the interactive driver system
#    Maps: steering = A/D, throttle = W/X, brake = S, pause = P
driver = veh.ChInteractiveDriver(vehicle)
driver.SetDeadband(0.0)
driver.SetSteeringDelta(0.04)
driver.SetThrottleDelta(0.04)
driver.SetBrakingDelta(0.04)
driver.Initialize()

# 8) Create the Irrlicht application for real-time 3D visualization
app = veh.ChWheeledVehicleIrrApp(
    vehicle,                  # the vehicle
    "BMW E90 Sedan Demo",     # window title
    irr.dimension2du(1280, 720),
    False,                    # do not use full screen
    False)                    # shadows off by default

# Set up the chase camera
app.SetChaseCameraPosition(chrono.ChVectorD(0.0, -6.0, 2.0))
app.SetChaseCameraSmoothness(30.0)

# Add a skybox, lights, and logos
app.AddTypicalSky()
app.AddTypicalLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"),               # icon
                   chrono.GetChronoDataFile("logo_pychrono.png"),                     # title
                   chrono.GetChronoDataFile("logo_pychrono_logo.png"),                # logo
                   50, 50)                                                           # position

app.AddTypicalLights()
app.SetLightDirection(chrono.ChVector3D(-0.5, 0.5, -1))
app.AddCamera(irr.vector3df(0, -10, 5), irr.vector3df(0, 0, 1))

# Finalize Irrlicht initialization
app.AssetBindAll()
app.AssetUpdateAll()
app.SetTimestep(1e-3)

# --------------------------------------------------------------------------
# 2) SIMULATION LOOP
# --------------------------------------------------------------------------

time = 0.0
while app.Run():
    # 1) Begin scene
    app.BeginScene()
    app.DrawAll()

    # 2) Collect driver inputs
    steering = driver.GetSteering()
    throttle = driver.GetThrottle()
    braking = driver.GetBraking()

    # 3) Synchronize modules at current time
    time = system.GetChTime()
    driver.Synchronize(time)
    vehicle.Synchronize(time, driver.GetInputs(), terrain)
    terrain.Synchronize(time)
    app.Synchronize("", steering, throttle, braking)

    # 4) Advance simulation for one timestep for all modules
    driver.Advance(app.GetTimestep())
    vehicle.Advance(app.GetTimestep())
    terrain.Advance(app.GetTimestep())
    app.Advance(app.GetTimestep())

    # 5) End scene
    app.EndScene()

# --------------------------------------------------------------------------
# 3) END
# --------------------------------------------------------------------------

print("Simulation finished.")