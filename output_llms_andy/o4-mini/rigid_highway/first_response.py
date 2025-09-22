import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# -----------------------------------------------------------------------------
# 1) GENERAL SETUP
# -----------------------------------------------------------------------------
chrono.SetChronoDataPath(chrono.GetChronoDataPath())  # or your custom path
chrono.SetChronoThread(0)  # single‐threaded; adjust as needed

# simulation parameters
time_step = 1.0 / 50.0   # 50 Hz
contact_method = chrono.ChContactMethod_SMC

# -----------------------------------------------------------------------------
# 2) CREATE THE VEHICLE
# -----------------------------------------------------------------------------
# Instantiate a full HMMWV vehicle model
vehicle = veh.HMMWV_Full(contact_method)

# Vehicle visualization and collision
vehicle.SetChassisCollisionType(veh.ChassisCollisionType_NONE)
vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0.2, 0),
                                           chrono.Q_from_AngY(0.0)))
vehicle.SetTireType(veh.TireModelType_TMEASY)
vehicle.SetTireStepSize(1e-3)

# Use mesh visualization for chassis and wheels
vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
# point these to your mesh files for chassis, rim, tire, etc., or rely on defaults
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)

# Initialize vehicle system
vehicle.Initialize()

# -----------------------------------------------------------------------------
# 3) CREATE MESH TERRAIN
# -----------------------------------------------------------------------------
terrain = veh.ChMeshTerrain(vehicle.GetSystem())

# Load a collision and a visual mesh (Wavefront OBJ)
terrain.LoadMesh('Highway_col.obj',        # collision mesh
                 'Highway_vis.obj')        # visual mesh

# Optional: set a uniform friction coefficient
terrain.SetMaterialSurface(0.9, 0.1, 0.02)

terrain.Initialize()

# -----------------------------------------------------------------------------
# 4) SET UP THE IRRLICHT APPLICATION
# -----------------------------------------------------------------------------
app = chronoirr.ChIrrApp(vehicle.GetSystem(),
                         "HMMWV Mesh Terrain",
                         chronoirr.dimension2du(1280, 720),
                         chronoirr.HorizontalDir_Z)

# Camera
app.AddTypicalCamera(chronoirr.vector3df(-10, 3,  0),
                     chronoirr.vector3df(  0, 0,  0))

# Lights and skybox
app.AddTypicalLights()
app.AddSkyBox()

# Set the step size and real‐time
app.SetTimestep(time_step)
app.SetTryRealtime(True)

# -----------------------------------------------------------------------------
# 5) CREATE AN INTERACTIVE DRIVER
# -----------------------------------------------------------------------------
driver = veh.ChIrrGuiDriver(app)
driver.SetThrottleDelta(0.06)
driver.SetBrakingDelta(0.1)
driver.SetSteeringDelta(0.04)
driver.Initialize()

# -----------------------------------------------------------------------------
# 6) BIND/UPDATE ALL ASSETS FOR VISUALIZATION
# -----------------------------------------------------------------------------
app.AssetBindAll()
app.AssetUpdateAll()

# -----------------------------------------------------------------------------
# 7) RUN THE SIMULATION LOOP
# -----------------------------------------------------------------------------
while app.GetDevice().run():
    t = vehicle.GetSystem().GetChTime()

    # Collect driver inputs
    steering = driver.GetSteering()
    throttle = driver.GetThrottle()
    braking  = driver.GetBraking()

    # Synchronize modules
    driver.Synchronize(t)
    terrain.Synchronize(t)
    vehicle.Synchronize(t, steering, throttle, braking, terrain)

    # Render
    app.BeginScene()
    app.DrawAll()
    app.EndScene()

    # Advance simulation for one timestep
    driver.Advance(time_step)
    terrain.Advance(time_step)
    vehicle.Advance(time_step)
    app.Advance(time_step)