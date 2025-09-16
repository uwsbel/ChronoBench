import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import pychrono.driver as driver
import math

# =============================================================================
# Simulation parameters
# =============================================================================

# Simulation step size
step_size = 0.01

# Simulation end time
tend = 100

# =============================================================================
# Create the vehicle system
# =============================================================================

# Create the vehicle system
vehicle = veh.WheelVehicle(veh.GetDataFile('hmmwv/vehicle/HMMWV_Vehicle.json'))

# Set vehicle collision type
vehicle.SetChassisCollisionType(chrono.ChCollisionModelType_ELLIPSOIDS)
vehicle.SetTireCollisionType(chrono.ChCollisionModelType_MESH)

# Initialize the vehicle at the specified location
vehicle.Initialize(chrono.ChCoorSysd(chrono.ChVectorD(0, 0, 0.5), chrono.QUNIT))

# =============================================================================
# Create the terrain
# =============================================================================

# Create a rigid terrain
terrain = veh.RigidTerrain(vehicle.GetSystem())
terrain.SetContactFrictionCoefficient(0.9)
terrain.SetContactRestitutionCoefficient(0.1)
terrain.SetContactMaterialProperties(2e7, 0.3)
terrain.Initialize(chrono.ChVectorD(0, 0, -1), chrono.ChVectorD(0, 0, 1))

# =============================================================================
# Create the driver system
# =============================================================================

# Create a driver system
driver = veh.ChDriver()
driver.SetSteeringDelta(0.1)
driver.SetThrottleDelta(0.1)
driver.SetBrakingDelta(0.1)
driver.Initialize()

# =============================================================================
# Create the visualization system
# =============================================================================

# Create the Irrlicht application
app = irr.ChIrrApp(vehicle.GetSystem(), 'MAN 10t Truck Simulation', irr.dimension2d(1280, 720))

# Set up the camera
app.AddTypicalLogo()
app.AddTypicalSky()
app.AddTypicalLights()
app.AddTypicalCamera(irr.vector3df(0, 5, 2))
app.SetChaseCamera(vehicle.GetChassisBody(), 6.0, 0.5)

# Set up the terrain visualization
terrain_vis = veh.ChTerrainVisualizationRigid(app.GetSceneManager(), vehicle.GetSystem(), terrain)
terrain_vis.SetTexture(veh.GetDataFile('terrain/textures/grass.jpg'), 20, 20)

# =============================================================================
# Simulation loop
# =============================================================================

# Number of simulation steps
num_steps = int(math.ceil(tend / step_size))

# Simulation loop
app.SetTimestep(step_size)
app.AssetBindAll()
app.AssetUpdateAll()

while app.GetDevice().run():
    app.BeginScene()
    app.DrawAll()
    app.DoStep()
    app.EndScene()

    # Driver inputs
    app.GetDevice().getVideoDriver().beginScene()
    driver.Synchronize(step_size)
    driver.SetSteering(0.5 * math.sin(2 * math.pi * 0.1 * app.GetSystem().GetChTime()))
    driver.SetThrottle(0.5)
    driver.SetBraking(0.0)
    driver.Advance(step_size)
    app.GetDevice().getVideoDriver().endScene()

    # Update vehicle state
    vehicle.Synchronize(step_size)
    vehicle.Advance(step_size)

    # Check for simulation end
    if app.GetSystem().GetChTime() >= tend:
        break

app.AssetUpdateAll()