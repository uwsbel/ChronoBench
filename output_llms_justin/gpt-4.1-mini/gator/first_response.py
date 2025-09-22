import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import os

# -----------------------------------------------------------------------------
# Initialize the PyChrono environment and core components
# -----------------------------------------------------------------------------
chrono.SetChronoDataPath(chrono.GetChronoDataPath())

# Simulation parameters
contact_method = chrono.ChContactMethod_SMC
step_size = 0.01
render_step_size = 1.0 / 50  # 50 FPS

# -----------------------------------------------------------------------------
# Create the vehicle system (Gator)
# -----------------------------------------------------------------------------
init_loc = chrono.ChVectorD(0, 0, 0.5)  # initial position (x,y,z)
init_rot = chrono.Q_from_AngX(0)        # initial orientation (no rotation)

vehicle = veh.Gator()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysD(init_loc, init_rot))
vehicle.SetTireType(veh.VehicleTire.TMEASY)
vehicle.SetTireStepSize(step_size)
vehicle.Initialize()

# Enable mesh visualization for all vehicle parts
vehicle.SetChassisVisualizationType(veh.ChassisVisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.SuspensionVisualizationType_MESH)
vehicle.SetSteeringVisualizationType(veh.SteeringVisualizationType_MESH)
vehicle.SetWheelVisualizationType(veh.WheelVisualizationType_MESH)
vehicle.SetTireVisualizationType(veh.TireVisualizationType_MESH)

# -----------------------------------------------------------------------------
# Create the terrain
# -----------------------------------------------------------------------------
terrain = veh.RigidTerrain(vehicle.GetSystem())
terrain.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT))

# Flat patch with specified dimensions (e.g., 200m x 200m), box dimension is half-size on each axis
patch_length = 100.0
patch_width = 100.0
patch_height = 0.1

patch = terrain.AddPatch(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, -patch_height), chrono.QUNIT),
                         patch_length, patch_width)
patch.SetTexture(veh.GetDataFile("terrain/textures/grass.png"), 16, 16)
patch.SetMaterialSurface(chrono.ChMaterialSurfaceSMC())
patch.GetMaterialSurface().SetFriction(0.9f)
patch.GetMaterialSurface().SetRestitution(0.01f)
patch.GetMaterialSurface().SetYoungModulus(2e7)
patch.GetMaterialSurface().SetPoissonRatio(0.3)
patch.GetMaterialSurface().SetKn(2e5)
patch.GetMaterialSurface().SetGn(40)
patch.GetMaterialSurface().SetKt(2e5)
patch.GetMaterialSurface().SetGt(20)

terrain.InitializePatch()

# -----------------------------------------------------------------------------
# Create the interactive driver system (Irrlicht GUI)
# -----------------------------------------------------------------------------
driver = veh.ChIrrGuiDriver(vehicle)
driver.SetSteeringDelta(0.04)
driver.SetThrottleDelta(0.1)
driver.SetBrakingDelta(0.1)
driver.Initialize()

# -----------------------------------------------------------------------------
# Visualization setup
# -----------------------------------------------------------------------------
application = chronoirr.ChIrrApp(vehicle.GetSystem(), "Gator Vehicle Simulation",
                                 chronoirr.dimension2du(1280, 720))

application.AddTypicalLights()
application.AddTypicalSky()
application.AddTypicalCamera(chronoirr.vector3df(0, -10, 4), chronoirr.vector3df(0, 0, 1))
application.AssetBindAll()
application.AssetUpdateAll()

# -----------------------------------------------------------------------------
# Simulation loop
# -----------------------------------------------------------------------------
time = 0
render_frame = 0
while application.GetDevice().run():
    time = vehicle.GetSystem().GetChTime()

    driver_inputs = driver.GetInputs()
    vehicle.Synchronize(time, driver_inputs, terrain)
    terrain.Synchronize(time)
    driver.Synchronize(time)

    application.BeginScene()
    application.DrawAll()

    vehicle.Advance(step_size)
    terrain.Advance(step_size)
    driver.Advance(step_size)

    application.EndScene()

    render_frame += 1
    chrono.ChRealtimeStepTimer().TrySleep(step_size)