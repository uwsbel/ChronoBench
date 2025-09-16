import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as chronovehicle

# ---------------------------------------------------------------------
#
#  Create the simulation system and add the vehicle
#
# ---------------------------------------------------------------------

system = chrono.ChSystemNSC()

# Set gravitational acceleration
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# Create the vehicle
vehicle = chronovehicle.ChVehicle("BMW E90", system)

# Set vehicle visualization parameters
vehicle.SetChassisVisualizationType(chronovehicle.VisualizationType_MESH)
vehicle.SetChassisCollide(True)
vehicle.SetSuspensionVisualizationType(chronovehicle.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(chronovehicle.VisualizationType_MESH)

# Load vehicle model and set initial position
vehicle.SetVehicleModel(chronovehicle.VehicleModelData("E90.json"))
vehicle.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 1.0, 0)))

# Add TMEASY tire model
tire_model = chronovehicle.TMeasyTire()
vehicle.SetTireModel(tire_model)

# ---------------------------------------------------------------------
#
#  Create the terrain
#
# ---------------------------------------------------------------------

terrain = chrono.ChRigidBody()
terrain.SetBodyFixed(True)
terrain.SetPos(chrono.ChVectorD(0, 0, 0))
terrain.SetCollide(True)

# Load terrain mesh and set collision shape
terrain.GetCollisionModel().ClearModel()
terrain.GetCollisionModel().AddMesh(chrono.ChTriangleMeshSoup(), False, False)
terrain.GetCollisionModel().BuildModel()

# Add ground texture and logo
terrain.AddAsset(chronoirr.ChIrrAssetType_TEXTURE, "ground_texture.jpg")
terrain.AddAsset(chronoirr.ChIrrAssetType_LOGO, "logo.png")

# ---------------------------------------------------------------------
#
#  Create the driver system
#
# ---------------------------------------------------------------------

driver = chronovehicle.ChInteractiveDriver(vehicle)

# Set driver controls
driver.SetSteeringControl(chrono.ChFunction_Const(0))
driver.SetThrottleControl(chrono.ChFunction_Const(0))
driver.SetBrakingControl(chrono.ChFunction_Const(0))

# ---------------------------------------------------------------------
#
#  Create the Irrlicht visualization system
#
# ---------------------------------------------------------------------

vis = chronoirr.ChIrrApp(system, "BMW E90 Simulation", chronoirr.dimension2du(1280, 720))
vis.AddTypicalSky()
vis.AddTypicalLights()
vis.SetChaseCamera(chrono.ChVectorD(0, 2.0, 5.0), chrono.ChVectorD(0, 1.0, 0))

# Add vehicle and terrain to the visualization
vis.AddAsset(vehicle, True)
vis.AddAsset(terrain, True)

# ---------------------------------------------------------------------
#
#  Simulation loop
#
# ---------------------------------------------------------------------

while vis.GetDevice().run():
    vis.BeginScene()
    vis.Render()
    system.DoStepDynamics(0.01)
    vis.EndScene()