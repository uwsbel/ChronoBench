""" HMMWV + SCM deformable terrain + Irrlicht real-time visualization
    ------------------------------------------------------------------
    This script shows how to set–up a complete Chrono::Vehicle scenario
    from Python:

    1.  HMMWV 4-wheel drive, full suspension model.
    2.  SCM deformable soil whose initial surface profile is read from
        an 8-bit grey–scale height-map.
    3.  Interactive (keyboard) driver.
    4.  Irrlicht window with lights and chase-camera.
    5.  Main loop that keeps every module (driver, vehicle, terrain,
        visual system) tightly synchronized while preserving real time.

    Tested with the official PyChrono binary (version ≥ 7.0).
"""
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

# ---------------------------------------------------------------------
# 1. Global simulation parameters
# ---------------------------------------------------------------------
step_size          = 2.5e-3        # integrator step (s)
tire_step_size     = 1.0e-3        # smaller step for the tire models
render_steps       = 5             # draw every n. integration steps
contact_method     = chrono.ChContactMethod_SMC
data_path          = chrono.GetChronoDataPath()               # helper
heightmap_file     = data_path + "terrain/heightmap.bmp"      # 8-bit
heightmap_sizeX    = 40.0         # [m]  size of height-map patch
heightmap_sizeY    = 40.0
heightmap_maxZ     = 0.50         # [m]  darkest pixel → 0   , lightest → maxZ
start_loc          = chrono.ChVectorD(0, 0, 1.0)              # initial
start_yaw          = 0.0

# ---------------------------------------------------------------------
# 2. Chrono system
# ---------------------------------------------------------------------
chrono.SetChronoDataPath(data_path)
sys = chrono.ChSystemSMC()               # SMC is mandatory for SCM soil
sys.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))

# ---------------------------------------------------------------------
# 3. Vehicle: HMMWV full model
# ---------------------------------------------------------------------
print("Creating the HMMWV vehicle ...")
vehicle = veh.HMMWV_Full(sys,
                         driveType   = veh.DriveType_4WD,
                         contactMethod=contact_method)

vehicle.SetInitPosition( chrono.ChCoordsysD(start_loc,
                          chrono.Q_from_AngZ(start_yaw)) )
vehicle.SetChassisFixed(False)
vehicle.SetTireType(veh.TireModelType_TMEASY)
vehicle.SetTireStepSize(tire_step_size)

# Nice looking visualization
vehicle.SetChassisVisualizationType(   veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(  veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(     veh.VisualizationType_MESH)
vehicle.Initialize()

# ---------------------------------------------------------------------
# 4. SCM deformable terrain
# ---------------------------------------------------------------------
print("Creating SCM deformable terrain ...")
terrain = veh.SCMDeformableTerrain(sys)

# Flat reference plane (Z = 0)
terrain.SetPlane( chrono.ChCoordsysD( chrono.ChVectorD(0,0,0),
                                      chrono.QUNIT ) )

# Grid resolution (cell size) and out-of-bounds cushion
terrain.Initialize(0.04)          # 4 cm nodes

# Bekker/Wong–Janosi soil parameters (medium soil)
terrain.SetSoilParameters( 12e5,     # Bekker Kphi [N/m^(n+1)]
                           0,        # Bekker Kc   [N/m^(n+2)]
                           1.1,      # Bekker n    [–]
                           1.3e4,    # Cohesion    [Pa]
                           30.0,     # Friction angle [deg]
                           0.02,     # Janosi shear [m]
                           4e7,      # Elastic stiffness (K) [N/m]
                           3e4)      # Damping (R)

# Load the height-map – the patch is centered at (0,0)
print("Loading height-map  …")
terrain.LoadHeightMap(heightmap_file,
                      chrono.ChVectorD(-heightmap_sizeX/2,
                                       -heightmap_sizeY/2, 0),
                      heightmap_sizeX, heightmap_sizeY,
                      heightmap_maxZ)

# (optional) visual texture on the soil
terrain.SetTexture( data_path + "terrain/textures/soil.jpg", 10, 10)

# ---------------------------------------------------------------------
# 5. Irrlicht application + GUI driver
# ---------------------------------------------------------------------
print("Starting Irrlicht interface …")
app = veh.ChVehicleIrrApp(vehicle,
                          "HMMWV on SCM deformable terrain",
                          irr.dimension2du(1280, 720))
app.SetSkyBox()
app.AddTypicalLights( chrono.ChVectorD( 30,  30, 100),
                      chrono.ChVectorD(-30, -30, 100),
                      250, 130)
app.AddTypicalLogo()
app.SetChaseCamera( chrono.ChVectorD(0,0.6,0),
                    4.0,        # chase distance
                    0.5)        # up-shift

app.SetTimestep(step_size)
app.Initialize()

# Interactive driver (WASD + SPACE)
driver = veh.ChIrrGuiDriver(app)
driver.SetSteeringDelta( 0.04 )
driver.SetThrottleDelta( 0.02 )
driver.SetBrakingDelta(  0.20 )
driver.SetGains( 0.50, 0.005 )      # filtering
driver.Initialize()

# ---------------------------------------------------------------------
# 6. Real-time simulation loop
# ---------------------------------------------------------------------
realtime_timer = chrono.ChRealtimeStepTimer()
step_number    = 0

print("Simulation loop started, press ESC to quit.")
while app.GetDevice().run():
    # Render scene
    if step_number % render_steps == 0:
        app.BeginScene(True, True, irr.SColor(255,140,161,192))
        app.DrawAll()
        app.EndScene()

    # Current time and inputs -----------------------------------------
    time    = sys.GetChTime()
    inputs  = driver.GetInputs()

    # Synchronize all modules -----------------------------------------
    driver.Synchronize (time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, inputs, terrain)
    app.Synchronize    ("HMMWV demo", inputs)

    # Advance state ----------------------------------------------------
    driver.Advance (step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    sys.DoStepDynamics(step_size)

    # Spin in real time ------------------------------------------------
    realtime_timer.Spin(step_size)
    step_number += 1