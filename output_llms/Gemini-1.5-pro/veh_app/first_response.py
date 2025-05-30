import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import os

# ---------------------------------------------------------------------
# Create the mechanical system
# ---------------------------------------------------------------------
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))  # Set gravity

# ---------------------------------------------------------------------
# Create the ground
# ---------------------------------------------------------------------
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 200, 200)
patch.SetTexture(veh.GetDataDirectory() + 'terrain/textures/tile4.jpg', 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# ---------------------------------------------------------------------
# Create the vehicle Irrlicht application
# ---------------------------------------------------------------------
app = veh.ChVehicleIrrApp(system)

# Easy shortcuts to add camera, lights, logo and sky
app.SetSkyBox()
app.AddTypicalLights()
app.AddLogo(veh.GetDataDirectory() + 'logo_pychrono_alpha.png')

# ---------------------------------------------------------------------
# Create the Gator vehicle and attach it to the terrain
# ---------------------------------------------------------------------
vehicle = veh.Gator(system)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1.6), chrono.ChQuaternionD(1, 0, 0, 0)))
vehicle.Initialize()

tire_vis_type = veh.VisualizationType_MESH
vehicle.SetTireVisualizationType(tire_vis_type)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_PRIMITIVES)

app.AssetBindAll()
app.AssetUpdateAll()

# ---------------------------------------------------------------------
# Create the interactive driver system
# ---------------------------------------------------------------------
driver = veh.ChInteractiveDriverIRR(app)
driver.Initialize()

# ---------------------------------------------------------------------
# Create a sensor manager and add sensors
# ---------------------------------------------------------------------
manager = veh.ChSensorManager(system)

# Add a point light
manager.AddLight(chrono.ChVectorD(0.8, -0.2, 1.5), chrono.ChColor(1, 1, 1), 1000)

# Add a camera
camera = manager.AddCamera(
    chrono.ChFrameD(chrono.ChVectorD(0.8, -0.2, 1.5), chrono.Q_from_AngAxis(0, chrono.ChVectorD(1, 0, 0))),
    60.0,
    60.0,
    0.01,
    100,
    vehicle.GetChassisBody(),
)
camera.SetLag(0.2)
camera.SetName("camera")

manager.Initialize()

# ---------------------------------------------------------------------
# Simulation loop
# ---------------------------------------------------------------------
time_step = 2e-3

while app.GetDevice().run():
    time = vehicle.GetSystem().GetChTime()

    # Collect output data from modules (for inter-module communication)
    driver_inputs = driver.GetInputs()

    # Update modules (process inputs from other modules)
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    manager.Synchronize(time)
    app.Synchronize(time, driver_inputs)

    # Advance simulation for one timestep for all modules
    driver.Advance(time_step)
    terrain.Advance(time_step)
    vehicle.Advance(time_step)
    manager.Advance(time_step)
    app.Advance(time_step)

    # Update the sensor manager
    manager.Update()

    # Render scene
    app.BeginScene(True, True, irr.SColor(255, 140, 161, 192))
    app.DrawAll()
    app.EndScene()