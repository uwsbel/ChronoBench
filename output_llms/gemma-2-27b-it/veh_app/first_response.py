import pychrono as chrono
import pychrono.postprocess as postprocess
import pychrono.irrlicht as chronoirr
import numpy as np

# ---------------------------------------------------------------------
#
#  Create the simulation system and add items
#
# ---------------------------------------------------------------------

mysystem = chrono.ChSystemNSC()

# Set gravitational acceleration
mysystem.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# Create the terrain
terrain = chrono.ChRigidBody()
terrain.SetBodyFixed(True)
col_terrain = chrono.ChCollisionShape()
col_terrain.SetGeometry(chrono.ChBoxShape(chrono.ChVectorD(100, 1, 100)))
terrain.AddCollisionShape(col_terrain)
terrain.SetPos(chrono.ChVectorD(0, -0.5, 0))
mysystem.Add(terrain)

# Create the Gator vehicle
gator = chrono.ChVehicle()
gator.SetSystem(mysystem)

# Create chassis
chassis = chrono.ChBody()
chassis.SetMass(1000)
chassis.SetInertiaXX(chrono.ChVectorD(100, 100, 100))
chassis.SetPos(chrono.ChVectorD(0, 1, 0))
gator.AddChassis(chassis)

# Create wheels
wheel_radius = 0.5
wheel_width = 0.2

# Front right wheel
right_front_wheel = chrono.ChWheel()
right_front_wheel.SetWheelGeometry(chrono.ChCylinderShape(wheel_radius, wheel_width))
right_front_wheel.SetPos(chrono.ChVectorD(1, 0.5, 1))
right_front_wheel.SetSteeringAngle(0)
gator.AddWheel(right_front_wheel)

# Front left wheel
left_front_wheel = chrono.ChWheel()
left_front_wheel.SetWheelGeometry(chrono.ChCylinderShape(wheel_radius, wheel_width))
left_front_wheel.SetPos(chrono.ChVectorD(-1, 0.5, 1))
left_front_wheel.SetSteeringAngle(0)
gator.AddWheel(left_front_wheel)

# Rear right wheel
right_rear_wheel = chrono.ChWheel()
right_rear_wheel.SetWheelGeometry(chrono.ChCylinderShape(wheel_radius, wheel_width))
right_rear_wheel.SetPos(chrono.ChVectorD(1, 0.5, -1))
right_rear_wheel.SetSteeringAngle(0)
gator.AddWheel(right_rear_wheel)

# Rear left wheel
left_rear_wheel = chrono.ChWheel()
left_rear_wheel.SetWheelGeometry(chrono.ChCylinderShape(wheel_radius, wheel_width))
left_rear_wheel.SetPos(chrono.ChVectorD(-1, 0.5, -1))
left_rear_wheel.SetSteeringAngle(0)
gator.AddWheel(left_rear_wheel)

# Set visualization types for vehicle parts
chassis.SetBodyFixed(False)
chassis.AddVisualShape(chrono.ChVisualShapeBox(
    chrono.ChVectorD(2, 1, 3), chrono.ChColor(0.5, 0.5, 0.5)))
right_front_wheel.AddVisualShape(chrono.ChVisualShapeCylinder(
    wheel_radius, wheel_width, chrono.ChColor(0, 0, 1)))
left_front_wheel.AddVisualShape(chrono.ChVisualShapeCylinder(
    wheel_radius, wheel_width, chrono.ChColor(0, 1, 0)))
right_rear_wheel.AddVisualShape(chrono.ChVisualShapeCylinder(
    wheel_radius, wheel_width, chrono.ChColor(1, 0, 0)))
left_rear_wheel.AddVisualShape(chrono.ChVisualShapeCylinder(
    wheel_radius, wheel_width, chrono.ChColor(1, 1, 0)))

# ---------------------------------------------------------------------
#
#  Create driver system
#
# ---------------------------------------------------------------------

driver = chrono.ChDriver()
driver.SetSystem(mysystem)
driver.SetVehicle(gator)

# ---------------------------------------------------------------------
#
#  Create sensor manager
#
# ---------------------------------------------------------------------

sensor_manager = chrono.ChSensorManager()
mysystem.AddSensorManager(sensor_manager)

# Add point lights
light1 = chrono.ChLightPoint()
light1.SetPos(chrono.ChVectorD(5, 5, 5))
light1.SetColor(chrono.ChColor(1, 1, 1))
sensor_manager.AddLight(light1)

light2 = chrono.ChLightPoint()
light2.SetPos(chrono.ChVectorD(-5, 5, 5))
light2.SetColor(chrono.ChColor(1, 1, 1))
sensor_manager.AddLight(light2)

# Add camera
camera = chrono.ChCamera()
camera.SetPos(chrono.ChVectorD(0, 2, 5))
camera.SetDir(chrono.ChVectorD(0, -0.5, -1))
camera.SetTarget(chassis.GetPos())
sensor_manager.AddCamera(camera)

# ---------------------------------------------------------------------
#
#  Create Irrlicht visualization
#
# ---------------------------------------------------------------------

vis = chronoirr.ChIrrApp(mysystem)
vis.SetCamera(camera)
vis.AddTypicalLights()
vis.AddSkyBox()
vis.SetWindowSize(800, 600)
vis.Initialize()

# ---------------------------------------------------------------------
#
#  Simulation loop
#
# ---------------------------------------------------------------------

while vis.Run():
    vis.BeginScene()
    vis.Render()

    # Update driver
    driver.Update(vis.GetTimeStep())

    # Update terrain
    terrain.Update(vis.GetTimeStep())

    # Update vehicle
    gator.Update(vis.GetTimeStep())

    # Update sensor manager
    sensor_manager.Update(vis.GetTimeStep())

    # Synchronize and advance simulation modules
    mysystem.DoStepDynamics(vis.GetTimeStep())

    vis.EndScene()

vis.GetDevice().closeDevice()