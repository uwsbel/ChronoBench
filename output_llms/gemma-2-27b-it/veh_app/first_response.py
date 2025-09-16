import pychrono as chrono
import pychrono.postprocess as postprocess
import pychrono.irrlicht as chronoirr

# ---------------------------------------------------------------------
#
#  Create the simulation system and add items
#

# Create a Chrono system
system = chrono.ChSystemNSC()

# Set the simulation parameters
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# Create a material for the terrain
terrain_material = chrono.ChMaterialSurfaceNSC()
terrain_material.SetFriction(0.8)
terrain_material.SetRestitution(0.2)

# Create a terrain object
terrain = chrono.ChBodyEasyBox(
    200, 200, 1, 1000, True, True, terrain_material
)
terrain.SetPos(chrono.ChVectorD(0, -1, 0))
terrain.SetBodyFixed(True)
system.Add(terrain)

# Create the Gator vehicle
vehicle = chrono.ChVehicle(system)
vehicle.SetChassisBody(chrono.ChBodyEasyBox(2, 1, 0.5, 1000, True, True))
vehicle.SetChassisFixed(False)

# Add wheels
wheel_radius = 0.3
wheel_width = 0.1
wheel_pos = [
    chrono.ChVectorD(1, -0.5, 0),
    chrono.ChVectorD(-1, -0.5, 0),
    chrono.ChVectorD(1, 0.5, 0),
    chrono.ChVectorD(-1, 0.5, 0),
]
for i in range(4):
    wheel = chrono.ChWheel(vehicle.GetSystem())
    wheel.SetRadius(wheel_radius)
    wheel.SetWidth(wheel_width)
    wheel.SetFriction(2.0)
    wheel.SetSuspensionStiffness(1000)
    wheel.SetSuspensionDamping(100)
    wheel.SetSteering(True)
    wheel.SetPos(wheel_pos[i])
    vehicle.AddWheel(wheel)

# Set visualization types for vehicle parts
vehicle.GetChassisBody().SetBodyFixed(False)
vehicle.GetChassisBody().SetCollide(True)
vehicle.GetChassisBody().SetVisualizationType(chrono.ChVisualizationType_MESH)

for wheel in vehicle.GetWheels():
    wheel.SetVisualizationType(chrono.ChVisualizationType_PRIMITIVES)

# Add the vehicle to the system
system.Add(vehicle)

# ---------------------------------------------------------------------
#
#  Create the driver system
#

driver = chrono.ChDriverSystem(vehicle)

# ---------------------------------------------------------------------
#
#  Create the sensor manager
#

sensor_manager = postprocess.ChSensorManager(system)

# Add a point light
light = postprocess.ChLightPoint(
    chrono.ChVectorD(0, 5, 0), chrono.ChColor(1, 1, 1), 1000
)
sensor_manager.AddSensor(light)

# Add a camera attached to the vehicle chassis
camera = postprocess.ChCamera(
    chrono.ChVectorD(0, 2, 0), chrono.ChVectorD(0, 0, -1), chrono.ChVectorD(0, 1, 0)
)
camera.SetZoom(1.5)
sensor_manager.AddSensor(camera)

# ---------------------------------------------------------------------
#
#  Create the Irrlicht visualization
#

# Create an Irrlicht application
vis = chronoirr.ChIrrApp(system, sensor_manager)
vis.SetWindowSize(1200, 800)
vis.SetStep(0.01)
vis.AddTypicalSky()
vis.AddTypicalLights()
vis.AddCamera(camera)

# Set the view point
vis.SetCameraPosition(chrono.ChVectorD(0, 5, 10))
vis.SetCameraLookAt(chrono.ChVectorD(0, 0, 0))

# ---------------------------------------------------------------------
#
#  Simulation loop
#

while vis.GetDevice().run():
    vis.BeginScene()
    vis.Render()

    # Update the driver system
    driver.Update(vis.GetTimeStep())

    # Update the simulation
    system.DoStepDynamics(vis.GetTimeStep())

    vis.EndScene()