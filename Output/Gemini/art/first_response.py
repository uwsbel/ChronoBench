import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh

# ---------------------------------------------------------------------
#
#  Create the simulation system and add items
#

# Create the Chrono system
system = chrono.ChSystemNSC()

# Set the default outward/inward shape margins for collision detection,
# this is epecially important for very large or very small objects.
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.001)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.001)

# Create the ARTcar vehicle, set parameters, and initialize
vehicle = veh.ARTcar_Simple(system)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChInitPosition(chrono.ChVectorD(0, 1, 0.5)))
vehicle.SetInitOrientation(chrono.ChQuaternionD(1, 0, 0, 0))
vehicle.SetContactMethod(contact_method=veh.ChContactMethod_NSC)
vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetTireVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.Initialize()

# Create the terrain, set its size and texture
terrain = veh.RigidTerrain(system)
terrain_size = chrono.ChVectorD(80, 12, 3)
terrain.Initialize(chrono.ChVectorD(0, 0, 0), terrain_size)
terrain.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 20, 20)

# Create the driver system
driver = veh.ChDriverSimple(vehicle)
driver.Initialize()

# ---------------------------------------------------------------------
#
#  Create an Irrlicht application to visualize the system
#

myapplication = chronoirr.ChIrrApp(
    system, "PyChrono example: ARTcar", chronoirr.dimension2du(1024, 768)
)
myapplication.AddTypicalSky()
myapplication.AddTypicalLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
myapplication.AddTypicalCamera(
    chronoirr.vector3df(0, 1.5, -2), chronoirr.vector3df(0, 0, 0)
)
myapplication.AddLightWithShadow(
    chronoirr.vector3df(10, 20, 10),
    chronoirr.vector3df(0, 2.6, 0),
    30,
    10,
    40,
    60,
    512,
    chronoirr.SColorf(1, 1, 1),
)
myapplication.AssetBindAll()
myapplication.AssetUpdateAll()
myapplication.AddShadowAll()

# ---------------------------------------------------------------------
#
#  Run the simulation
#

myapplication.SetTimestep(0.01)

while myapplication.GetDevice().run():
    # Render scene
    myapplication.BeginScene()
    myapplication.DrawAll()
    myapplication.EndScene()

    # Get driver inputs
    driver_inputs = driver.GetInputs()

    # Update the vehicle systems
    vehicle.Update(system.GetChTime(), driver_inputs)

    # Advance simulation step
    system.DoStepDynamics(0.01)

    # Display simulation at 50 FPS
    myapplication.SetTryRealtime(True)
    myapplication.SetDesiredFPS(50)