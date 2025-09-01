# citybus_sim.py

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh

# -----------------------------------------------------------------------------
# 1) INITIALIZE THE CHRONO ENVIRONMENT
# -----------------------------------------------------------------------------

# Use NSC (non-smooth contact) or SMC (smooth contact)
chrono.SetChronoDataPath("C:/Chrono/chrono_data/")  
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# -----------------------------------------------------------------------------
# 2) CREATE THE CITYBUS VEHICLE
# -----------------------------------------------------------------------------

# We assume the CityBus model is defined by a JSON specification that
# lives under data/vehicle/citybus/CityBus.json
bus = veh.WheeledVehicle(system, 
    veh.GetDataFile("vehicle/citybus/CityBus.json"), 
    chrono.ChContactMethod_NSC)

# Initialize at X=0, Y=0.5 m above ground, heading along +X
init_loc = chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0), chrono.QUNIT)
bus.Initialize(init_loc)

# Visualization: mesh for chassis and wheels, primitive for suspension
bus.SetChassisVisualizationType(veh.VisualizationType_MESH)
bus.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVE)
bus.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVE)
bus.SetWheelVisualizationType(veh.VisualizationType_MESH)

# Create and attach TMsEasy tires (one per wheel)
# We point each tire to its JSON spec under data/vehicle/citybus/tire
for axle in range(bus.GetNumberAxles()):
    for side in [0, 1]:
        tire_json = "vehicle/citybus/tire/tm_tire.json"
        tire = veh.TMeasyTire(tire_json, veh.ChTire.TMEASY)
        bus.InitializeTire(tire, axle, side)

# -----------------------------------------------------------------------------
# 3) CREATE RIGID TERRAIN WITH CUSTOM TEXTURE
# -----------------------------------------------------------------------------

terrain = veh.RigidTerrain(system)

# Define a big 200×200 m patch at the origin
patch = terrain.AddPatch(
    chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT),
    sizeX=200, sizeY=200)

# Set a dark asphalt‐like material
patch.SetMaterialSurface(veh.ChMaterialSurfaceNSC())
mat = patch.GetMaterialSurface()
mat.SetFriction(0.9)
mat.SetRestitution(0.1)

# Set your custom texture (replace with your own file if you like)
patch.SetTexture(
    veh.GetDataFile("terrain/textures/tile4.jpg"),
    scale_x=20, scale_y=20)

terrain.Initialize()

# -----------------------------------------------------------------------------
# 4) SET UP IRRLICHT APPLICATION FOR VISUALIZATION
# -----------------------------------------------------------------------------

app = chronoirr.ChIrrApp(
    system,
    "CityBus on Rigid Terrain",
    chronoirr.dimension2du(1280, 720))

app.AddTypicalLogo()
app.AddTypicalSky()
app.AddTypicalLights()
app.AddTypicalCamera(chronoirr.ChVectorD(-8, 4, -8))
app.SetChaseCameraPosition(chronoirr.ChVectorD(0, 3, -8))

# Bind/Update assets (must come after vehicle & terrain init)
app.AssetBindAll()
app.AssetUpdateAll()

# -----------------------------------------------------------------------------
# 5) CREATE AN INTERACTIVE GUI DRIVER
# -----------------------------------------------------------------------------

driver = veh.ChIrrGuiDriver(app)
driver.SetSteeringDelta(0.04)   # how fast steering changes per frame
driver.SetThrottleDelta(0.05)
driver.SetBrakingDelta(0.1)
driver.Initialize()

# -----------------------------------------------------------------------------
# 6) SIMULATION LOOP (50 Hz)
# -----------------------------------------------------------------------------

time_step = 1.0 / 50.0
max_time   = 100.0

while app.GetDevice().run() and system.GetChTime() < max_time:
    # Retrieve driver inputs
    steering, throttle, braking = driver.GetInput()
    
    # Synchronize modules at current time
    time = system.GetChTime()
    driver.Synchronize(time)
    bus.Synchronize(time, steering, throttle, braking, terrain)
    terrain.Synchronize(time)
    
    # Advance dynamics
    system.DoStepDynamics(time_step)
    
    # Render scene
    app.BeginScene()
    app.DrawAll()
    app.EndScene()

# Clean up
app.GetDevice().drop()