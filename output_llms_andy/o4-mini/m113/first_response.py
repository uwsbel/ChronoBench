import os
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.vehicle.m113 as m113
import pychrono.irrlicht as irr

# -----------------------------------------------------------------------------
# 1) Initialize Chrono and set data paths
# -----------------------------------------------------------------------------
chrono.SetChronoDataPath(os.environ.get('CHRONO_DATA_DIR', ''))   
chrono.SetChronoEngineUpdateMode(chrono.ChEngineUpdateMode_FULL_UPDATE)

# -----------------------------------------------------------------------------
# 2) Create the M113 vehicle
# -----------------------------------------------------------------------------
# Choose SMC contact method
contact_method = chrono.ChMaterialSurfaceSMC

vehicle = m113.M113_Vehicle(contact_method)
vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetTrackShoeVisualizationType(veh.VisualizationType_MESH)
vehicle.SetBrakeType(m113.M113_BrakeType_SHAFTS)
vehicle.SetTrackShoeType(m113.M113_TrackShoeType_SINGLE_PIN)

# Initialize at (x=0, y=0.4, z=0), no rotation
init_loc = chrono.ChVectorD(0, 0.4, 0)
init_rot = chrono.QUNIT
vehicle.Initialize(chrono.ChCoordsysD(init_loc, init_rot))

# -----------------------------------------------------------------------------
# 3) Create a rigid terrain
# -----------------------------------------------------------------------------
terrain = veh.RigidTerrain(vehicle.GetSystem())

# Add a large flat patch (50×50 m, thickness = 1 m)
patch_mat = chrono.SMC_SurfaceSMC()
patch = terrain.AddPatch(chrono.ChCoordsysD(chrono.ChVectorD(0,0,0), chrono.QUNIT),
                         chrono.ChVectorD(50,50,1))
patch.SetMaterialSurface(patch_mat)

# Set friction, restitution, Young’s modulus, etc.
patch_mat.SetFriction(0.8)
patch_mat.SetRestitution(0.1)
patch_mat.SetYoungModulus(2e7)
patch_mat.SetPoissonRatio(0.3)

terrain.Initialize()

# -----------------------------------------------------------------------------
# 4) Create the Irrlicht visualization interface
# -----------------------------------------------------------------------------
# A helper application that draws vehicle + terrain and manages keyboard/mouse
app = veh.ChIrrApp(vehicle,                                       # the vehicle to render
                   "M113 Tracked Vehicle Demo",                  # window title
                   irr.dimension2du(1024, 768),                  # window size
                   False, False,                                  # no shadows, no vertical sync
                   True,                                          # enable wireframe?
                   chrono.GetChronoDataPath() + 'textures/')     # optional texture path

app.AddTypicalLogo()
app.AddTypicalSky()
app.AddTypicalLights()
# Third‐person chasing camera
app.AddTypicalCamera(irr.vector3df(0.0, 2.0, -6.0),
                     irr.vector3df(0.0, 0.4, 0.0))

# Bind assets (meshes, textures) and update all
app.AssetBindAll()
app.AssetUpdateAll()

# -----------------------------------------------------------------------------
# 5) Create a simple driver (Irrlicht GUI)
# -----------------------------------------------------------------------------
driver = veh.ChIrrGuiDriver(app)
# Basic key mappings: 
#   W/S - throttle/brake, 
#   A/D - left/right track
driver.SetThrottleDelta(0.1)
driver.SetBrakingDelta(0.2)
driver.SetSteeringDelta(0.0)  # no steering wheel on M113
driver.Initialize()

# -----------------------------------------------------------------------------
# 6) Simulation loop
# -----------------------------------------------------------------------------
timestep = 1.0 / 60.0
while app.GetDevice().run():
    time = vehicle.GetSystem().GetChTime()

    # 1. Collect driver inputs
    driver_inputs = driver.GetInputs()

    # 2. Synchronize all modules at current time
    vehicle.Synchronize(time, driver_inputs, terrain)
    terrain.Synchronize(time)
    app.Synchronize(time)

    # 3. Render scene
    app.BeginScene()
    app.DrawAll()
    app.EndScene()

    # 4. Advance dynamics by one timestep
    vehicle.Advance(timestep)
    terrain.Advance(timestep)
    driver.Advance(timestep)
    app.Advance(timestep)