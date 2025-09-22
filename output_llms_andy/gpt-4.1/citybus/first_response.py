import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math

# 1. Initialize the PyChrono environment
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(veh.GetDataPath())

# 2. Create the physical system
system = chrono.ChSystemSMC()

# 3. Create the vehicle (CityBus)
initLoc = chrono.ChVectorD(0, 0, 1.0)  # X, Y, Z
initRot = chrono.Q_from_AngY(math.radians(15))  # 15 degrees yaw
tire_model = veh.TireModelType.TMEASY

citybus = veh.CityBus()
citybus.SetContactMethod(chrono.ChContactMethod_SMC)
citybus.SetChassisFixed(False)
citybus.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot))
citybus.SetTireType(tire_model)
citybus.SetTireStepSize(1e-3)
citybus.Initialize()

# Visualization types
citybus.SetChassisVisualizationType(veh.VisualizationType.MESH)  # Mesh for chassis
citybus.SetSuspensionVisualizationType(veh.VisualizationType.PRIMITIVES)
citybus.SetSteeringVisualizationType(veh.VisualizationType.PRIMITIVES)
citybus.SetWheelVisualizationType(veh.VisualizationType.MESH)    # Mesh for wheels
citybus.SetTireVisualizationType(veh.VisualizationType.PRIMITIVES)

# 4. Create the terrain
terrain = veh.RigidTerrain(system)
patch = terrain.AddPatch(
    chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT),
    length=200.0, width=100.0
)
patch.SetContactFrictionCoefficient(0.9)
patch.SetRestitutionCoefficient(0.01)
patch.SetMaterialSurface(chrono.ChMaterialSurfaceSMC())
patch.SetTexture(chrono.GetChronoDataFile("terrain/textures/tile4.jpg"), 200, 100)
terrain.Initialize()

# 5. Create the Irrlicht application
app = veh.ChVehicleIrrApp(
    citybus, "PyChrono CityBus Demo", irr.dimension2du(1024, 768)
)
app.SetSkyBox()
app.AddTypicalLights()
app.AddTypicalLogo()
app.AddLightWithShadow(chrono.ChVectorF(50, 50, 100), chrono.ChVectorF(0, 0, 0), 250, 1, 60, 60, 1)
app.SetChaseCamera(
    chrono.ChVectorD(0.0, 0.0, 1.75),  # Camera position relative to vehicle
    6.0,  # chase distance
    0.5   # chase height
)
app.SetTimestep(0.02)  # 50 FPS

# 6. Create the interactive driver
driver = veh.ChIrrGuiDriver(app)
driver.SetSteeringDelta(0.04)  # how much to increment steering per keypress
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.1)
driver.Initialize()

# 7. Simulation loop
step_size = 0.02  # 50 FPS
render_steps = 1

while app.GetDevice().run():
    time = system.GetChTime()

    # Input from driver
    driver.Synchronize(time)
    # Synchronize vehicle and terrain
    citybus.Synchronize(time, driver.GetInputs(), terrain)
    terrain.Synchronize(time)

    # Advance simulation
    driver.Advance(step_size)
    citybus.Advance(step_size)
    terrain.Advance(step_size)

    # Render scene
    if app.IsRunning():
        app.BeginScene()
        app.DrawAll()
        app.EndScene()
    else:
        break