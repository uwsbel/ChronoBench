import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

# -------------------------------------
# Initialize the PyChrono system
# -------------------------------------
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Set gravity

# -------------------------------------
# Create the BMW E90 Sedan vehicle
# -------------------------------------
# Load the vehicle model from a JSON file (ensure the file exists in the data path)
vehicle = veh.WheeledVehicle(system, veh.GetDataFile("vehicle/BMW_E90_Sedan.json"))

# Set initial position and orientation
vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0), chrono.QUNIT))

# Configure TMeasy tire model for all tires
tire_model = veh.TMeasyTire()
for axle in vehicle.GetAxles():
    for wheel in axle.GetWheels():
        wheel.SetTire(tire_model.Clone())

# Configure visualization types for vehicle components
vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)

# Initialize the vehicle
vehicle.Initialize()

# -------------------------------------
# Create the rigid terrain
# -------------------------------------
# Create a flat rigid terrain
ground = chrono.ChBodyEasyBox(100, 1, 100, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -1, 0))
ground.SetBodyFixed(True)
system.Add(ground)

# Add a customizable texture to the terrain
terrain_texture = chrono.ChTexture()
terrain_texture.SetTextureFilename(chrono.GetChronoDataFile("textures/concrete.jpg"))
ground.AddAsset(terrain_texture)

# -------------------------------------
# Initialize Irrlicht visualization
# -------------------------------------
application = irr.ChIrrApp(system, 'BMW E90 Sedan Simulation', irr.dimension2du(1024, 768))

# Add skybox for background
application.AddSkyBox()

# Add directional light
light = irr.ChLightDirectional()
light.SetDirection(chrono.ChVectorD(-1, -2, -1))
application.GetSceneManager().addLight(light)

# Set up a chase camera that follows the vehicle
application.SetChaseCamera(vehicle.GetChassisBody(), 6.0, 0.5)

# -------------------------------------
# Set up the interactive driver system
# -------------------------------------
driver = veh.ChInteractiveDriverIRR(application)
driver.SetVehicle(vehicle)
driver.SetSteeringDelta(0.05)
driver.SetThrottleDelta(0.05)
driver.SetBrakingDelta(0.05)

# -------------------------------------
# Simulation loop
# -------------------------------------
application.AssetBindAll()
application.AssetUpdateAll()

while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.EndScene()

    # Update driver inputs and simulate one time step
    driver.Synchronize(system.GetChTime())
    system.DoStepDynamics(chrono.ChTime(0.01))