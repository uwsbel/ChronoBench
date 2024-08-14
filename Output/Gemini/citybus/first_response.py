import pychrono as chrono
import pychrono.irrlicht as chronoirr

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

# Create the ground
terrain = chrono.ChBodyEasyBox(100, 1, 100, 1000, True, True)
terrain.SetBodyFixed(True)
terrain.SetPos(chrono.ChVectorD(0, -1, 0))
asset = chrono.ChTexture()
asset.SetTextureFilename(chrono.GetChronoDataFile('textures/concrete.jpg'))
terrain.GetAssets().push_back(asset)
system.Add(terrain)

# Create the vehicle
vehicle = chrono.ChVehicleModelData()
vehicle.LoadVehicleData(chrono.GetChronoDataFile('vehicle/citybus/vehicle_data.json'))

# Initialize the vehicle at the specified position
initLoc = chrono.ChVectorD(0, 1, 0)
initRot = chrono.ChQuaternionD(1, 0, 0, 0)

my_vehicle = chrono.ChVehicle(vehicle, system)
my_vehicle.SetInitPos(initLoc)
my_vehicle.SetInitRot(initRot)
my_vehicle.Initialize()

# Set tire model
my_vehicle.SetTireType('RIGID')

# Create and attach the visualization assets
mesh_vis_shape = chrono.ChVehicleVisualSystemIrrlicht()
mesh_vis_shape.SetWindowTitle('CityBus')
mesh_vis_shape.SetChaseCamera(chrono.ChVectorD(0, 3, -6), 6.0, 0.5)
mesh_vis_shape.Initialize()
mesh_vis_shape.AddTypicalLights()
mesh_vis_shape.AddSkyBox()
mesh_vis_shape.AttachVehicle(my_vehicle)

# ---------------------------------------------------------------------
#
#  Create an Irrlicht application to visualize the system
#

# Create the interactive driver system
driver = chrono.ChInteractiveDriverIrrlicht(mesh_vis_shape)

# Set the time step
my_vehicle.SetChassisFixed(False)
system.SetChTime(0)

driver.Initialize()

# Simulation loop
while mesh_vis_shape.Run():
    time = system.GetChTime()

    # Driver inputs
    driver_inputs = driver.GetInputs()

    # Update the vehicle
    my_vehicle.Update(time, driver_inputs)

    # Advance simulation step
    system.DoStepDynamics(1.0 / 50)

    # Render scene
    mesh_vis_shape.BeginScene()
    mesh_vis_shape.Render()
    mesh_vis_shape.EndScene()