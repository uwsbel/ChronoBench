import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import os

# -------------------------------------------------------------------
# Create the Chrono system and set default parameters
# -------------------------------------------------------------------
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# -------------------------------------------------------------------
# Set up the simulation visualization with Irrlicht
# -------------------------------------------------------------------
application = chronoirr.ChIrrApp(system, "HMMWV Simulation", chronoirr.dimension2du(1024, 768))
application.AddTypicalSky()
application.AddTypicalLights()
application.SetShowInfos(True)
application.SetPaused(True)
application.AssetBindAll()
application.AssetUpdateAll()
camera = application.GetDevice().getSceneManager().addCameraSceneNode(distance=4.0)
camera.setTarget(chrono.ChVectorD(0, 1.5, 0))

# -------------------------------------------------------------------
# Load the HMMWV vehicle model (replace with your own model)
# -------------------------------------------------------------------
vehicle = chrono.vehicle.ChVehicle(system, "hmmwv.json", chrono.vehicle.GetDataPath() + "hmmwv/")
vehicle.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 1, 0), chrono.QUNIT))

# Set contact method to SMC and tire model to TMEASY
vehicle.SetContactMethod(chrono.ChContactMethod.SMC)
vehicle.SetTireType("TMEASY")

# Set mesh visualization for all vehicle components
for axle in vehicle.GetAxles():
    axle.m_suspension.m_pointOnChassis.SetVisualizationType(
        chrono.VisualizationType_MESH
    )
    axle.m_suspension.m_pointOnWheel.SetVisualizationType(
        chrono.VisualizationType_MESH
    )
    axle.m_wheel.SetVisualizationType(chrono.VisualizationType_MESH)

# -------------------------------------------------------------------
# Load custom terrain meshes
# -------------------------------------------------------------------
terrain_collision = chrono.ChTriangleMeshConnected(
    "Highway_col.obj",  # Replace with your mesh file
    chrono.ChVectorD(0, 0, 0),  # Position
    chrono.QUNIT,  # Orientation
    1.0,  # Scaling
)
terrain_collision.SetCollisionFlags(chrono.ChCollisionFlag.STATIC)
system.Add(terrain_collision)

terrain_visual = chrono.ChTriangleMeshShape()
terrain_visual.SetMesh(
    chrono.ChTriangleMeshConnected(
        "Highway_vis.obj", chrono.ChVectorD(0, 0, 0), chrono.QUNIT, 1.0
    )
)
terrain_collision.AddAsset(terrain_visual)

# -------------------------------------------------------------------
# Create an interactive driver
# -------------------------------------------------------------------
driver = chrono.vehicle.ChInteractiveDriver(
    system, vehicle, application.GetDevice()
)
driver.Initialize()

# -------------------------------------------------------------------
# Simulation loop
# -------------------------------------------------------------------
application.SetTimestep(1.0 / 50)
while application.GetDevice().run():
    time = system.GetChTime()

    # Update vehicle and driver
    vehicle.Update(time)
    driver.Synchronize(time)

    # Advance simulation
    system.DoStepDynamics(1.0 / 50)

    # Update visualization
    application.BeginScene()
    application.DrawAll()
    application.EndScene()