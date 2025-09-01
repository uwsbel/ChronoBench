# =============================================================================
# PyChrono FEDA Vehicle on Rigid Terrain with Irrlicht Visualization
# =============================================================================

import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

# -----------------------------------------------------------------------------
# 1) Chrono initialization
# -----------------------------------------------------------------------------
chrono.SetChronoDataPath(chrono.GetChronoDataFilePath())   # data path for textures, meshes, etc.
SCENE_TITLE = "PyChrono FEDA Demo"

# Simulation parameters
timestep = 1.0 / 50.0   # 50 FPS

# -----------------------------------------------------------------------------
# 2) Create the FEDA vehicle
# -----------------------------------------------------------------------------
# Use DVI contact method
contact_method = chrono.ChMaterialSurface.ContactMethod.DVI

# Instantiate vehicle
vehicle = veh.FedaVehicle(contact_method=contact_method)
# Set vehicle initial position and orientation (x,y,z) and no initial rotation
init_loc = chrono.ChVectorD(0, 0, 0.5)  
init_rot = chrono.QUNIT
vehicle.Initialize(chrono.ChCoordsysD(init_loc, init_rot))

# Select mesh visualization for all parts
vehicle.SetChassisVisualizationType(veh.VisualizationType.MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType.MESH)
vehicle.SetSteeringVisualizationType(veh.VisualizationType.MESH)
vehicle.SetWheelVisualizationType(veh.VisualizationType.MESH)
vehicle.SetTireVisualizationType(veh.VisualizationType.MESH)

# Use a TMeasy tire model
tire_left = veh.TMeasyTire("TMeasy", veh.VisualizationType.MESH)
tire_right = veh.TMeasyTire("TMeasy", veh.VisualizationType.MESH)
for axle in vehicle.GetAxles():
    axle.left_tire = tire_left
    axle.right_tire = tire_right

# -----------------------------------------------------------------------------
# 3) Create the rigid terrain
# -----------------------------------------------------------------------------
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0),
                                           chrono.QUNIT),
                         chrono.VectorD(200, 200, 1))
patch.SetMaterialSurface(veh.CCMaterials.CreateMaterial(contact_method))
patch.SetTexture(chrono.GetChronoDataFile("terrain/grass.jpg"), 200, 200)
terrain.Initialize()

# -----------------------------------------------------------------------------
# 4) Create the Irrlicht application for visualization
# -----------------------------------------------------------------------------
app = irr.ChIrrApp(vehicle.GetSystem(),
                   SCENE_TITLE,
                   irr.dimension2du(1280, 720))
# Easy lighting and camera
app.AddTypicalSky()
app.AddTypicalLogo()
app.AddTypicalLights()
app.AddTypicalCamera(irr.vector3df(0.0, 5.0, -10.0),   # camera position
                     irr.vector3df(0.0, 0.5, 0.0))     # camera target
app.SetTimestep(timestep)

# -----------------------------------------------------------------------------
# 5) Create the interactive driver
# -----------------------------------------------------------------------------
driver = veh.ChIrrGuiDriver(app)
driver.SetSteeringDelta(0.04)
driver.SetThrottleDelta(0.04)
driver.SetBrakingDelta(0.1)
driver.Initialize()

# -----------------------------------------------------------------------------
# 6) Initialize all modules
# -----------------------------------------------------------------------------
app.AssetBindAll()
app.AssetUpdateAll()

# -----------------------------------------------------------------------------
# 7) Simulation loop
# -----------------------------------------------------------------------------
time = 0.0
while app.GetDevice().run():
    # 7.1 Get driver inputs
    driver_inputs = driver.GetInputs()
    steering = driver_inputs.get_steering()
    throttle = driver_inputs.get_throttle()
    braking = driver_inputs.get_braking()

    # 7.2 Synchronize modules at current time
    time = vehicle.GetSystem().GetChTime()
    driver.Synchronize(time)
    vehicle.Synchronize(time, steering, throttle, braking, terrain)
    terrain.Synchronize(time)

    # 7.3 Advance dynamics
    driver.Advance(timestep)
    vehicle.Advance(timestep)
    terrain.Advance(timestep)

    # 7.4 Render scene
    app.BeginScene()
    app.DrawAll()
    app.EndScene()

# =============================================================================
# End of script
# =============================================================================