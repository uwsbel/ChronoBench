import pychrono as chrono
import pychrono.postprocess as postprocess
import pychrono.irrlicht as chronoirr

# ---------------------------------------------------------------------
#
#  Create the simulation system and add items
#
# ---------------------------------------------------------------------

mysystem = chrono.ChSystemNSC()
mysystem.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# Create the ground
ground = chrono.ChBodyEasyBox(
    100, 100, 1, 1000, True, True, chrono.ChMaterialSurface.NSC
)
ground.SetPos(chrono.ChVectorD(0, -1, 0))
mysystem.Add(ground)

# Create the Gator vehicle
gator = chrono.ChVehicle("Gator")
gator.SetChassisFixed(False)

# Chassis
chassis = chrono.ChBodyEasyBox(
    2.5, 1.2, 0.6, 1000, True, True, chrono.ChMaterialSurface.NSC
)
chassis.SetPos(chrono.ChVectorD(0, 0.3, 0))
gator.AddChassis(chassis)

# Wheels
wheel_radius = 0.3
wheel_width = 0.2

# Set different visualization types for each wheel
wheel_vis_types = [
    chrono.ChVisualizationType_MESH,
    chrono.ChVisualizationType_WIREFRAME,
    chrono.ChVisualizationType_NONE,
    chrono.ChVisualizationType_PRIMITIVES,
]

for i in range(4):
    wheel = chrono.ChWheel(
        chrono.ChVectorD(1.0 * (i % 2) * 1.5 - 1.5, -0.6, 1.0 * (i // 2) * 1.5 - 0.6),
        wheel_radius,
        wheel_width,
        20.0,
        1000,
        chrono.ChMaterialSurface.NSC,
    )
    wheel.SetVisualizationType(wheel_vis_types[i])
    gator.AddWheel(wheel)

mysystem.Add(gator)

# ---------------------------------------------------------------------
#
#  Create the driver system
#
# ---------------------------------------------------------------------

driver = chrono.ChDriver(gator)
driver.SetSteeringAngle(0)
driver.SetThrottle(0)
driver.SetBraking(0)

# ---------------------------------------------------------------------
#
#  Create the sensor manager
#
# ---------------------------------------------------------------------

sensor_manager = chrono.ChSensorManager(mysystem)

# Add point lights
sensor_manager.AddLight(
    chrono.ChLightPoint(chrono.ChVectorD(0, 5, 0)), chrono.ChColor(1, 1, 1)
)

# Add camera
camera = chrono.ChCamera(
    chrono.ChVectorD(0, 2, 5),
    chrono.ChVectorD(0, 0, 0),
    chrono.ChVectorD(0, 1, 0),
    chrono.ChRad(chrono.CH_C_PI / 4),
)
sensor_manager.AddCamera(camera)

# ---------------------------------------------------------------------
#
#  Create the Irrlicht visualization system
#
# ---------------------------------------------------------------------

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(mysystem)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle("Gator Simulation")
vis.Initialize()
vis.AddCamera(camera)
vis.AddLight(sensor_manager.GetLight(0))

# ---------------------------------------------------------------------
#
#  Simulation loop
#
# ---------------------------------------------------------------------

while vis.Run():
    # Update the driver
    driver.Update(mysystem.GetChTime())

    # Update the terrain (if needed)

    # Update the vehicle
    gator.Update(mysystem.GetChTime())

    # Update the sensor manager
    sensor_manager.Update(mysystem.GetChTime())

    # Render the scene
    vis.Render()

    # Advance the simulation
    mysystem.DoStepDynamics(0.01)

    # Synchronize the modules
    vis.Synchronize(mysystem.GetChTime(), mysystem.GetChTimeStep())