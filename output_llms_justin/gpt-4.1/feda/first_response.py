import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import os

# -----------------------------------------------------------------------------
# INITIALIZATION
# -----------------------------------------------------------------------------
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Simulation parameters
step_size = 2e-3
render_step_size = 1.0 / 50  # 50 FPS

# Vehicle initial parameters
initLoc = chrono.ChVectorD(0, 0, 1.0)
initYaw = chrono.CH_C_PI / 12  # 15 degrees
contact_method = chrono.ChContactMethod_NSC
tire_type = veh.TireModelType.TMEASY

# -----------------------------------------------------------------------------
# CREATE THE VEHICLE
# -----------------------------------------------------------------------------
vehicle = veh.FEDA(False, veh.ChContactMethod(contact_method))
vehicle.SetInitPosition(chrono.ChCoordsysD(initLoc, chrono.Q_from_AngZ(initYaw)))
vehicle.SetTireType(tire_type)
vehicle.SetTireStepSize(step_size)
vehicle.SetChassisVisualizationType(veh.VisualizationType.MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType.MESH)
vehicle.SetSteeringVisualizationType(veh.VisualizationType.MESH)
vehicle.SetWheelVisualizationType(veh.VisualizationType.MESH)
vehicle.SetTireVisualizationType(veh.VisualizationType.MESH)
vehicle.Initialize()

# -----------------------------------------------------------------------------
# CREATE THE TERRAIN
# -----------------------------------------------------------------------------
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(
    chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT),
    chrono.ChVectorD(200, 200, 2),  # size
    0.0,  # height
)
# Set custom texture
patch.SetTexture(chrono.GetChronoDataFile("path/to/your/texture.jpg"), 200, 200)
terrain.Initialize()

# -----------------------------------------------------------------------------
# IRRLICHT VISUALIZATION
# -----------------------------------------------------------------------------
app = veh.ChWheeledVehicleIrrApp(vehicle, 'FEDA Demo', irr.dimension2du(1024, 768))
app.SetSkyBox()
app.AddTypicalLights()
app.AddLightWithShadow(chrono.ChVectorF(30, 100, 100), chrono.ChVectorF(0, 0, 0), 250, 1, 10, 60)
app.SetChaseCamera(chrono.ChVectorD(0.0, 0.0, 1.75), 6.0, 0.5)  # (track point, distance, height)
app.SetTimestep(step_size)
app.AssetBindAll()
app.AssetUpdateAll()

# -----------------------------------------------------------------------------
# DRIVER SYSTEM (interactive)
# -----------------------------------------------------------------------------
driver = veh.ChIrrGuiDriver(app)
driver.SetSteeringDelta(0.04)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.1)
driver.Initialize()

# -----------------------------------------------------------------------------
# SIMULATION LOOP
# -----------------------------------------------------------------------------
realtime_timer = chrono.ChRealtimeStepTimer()
render_steps = int(render_step_size / step_size)
step_number = 0

while app.GetDevice().run():
    # Render scene
    if step_number % render_steps == 0:
        app.BeginScene()
        app.DrawAll()
        app.EndScene()

    # Get driver inputs
    driver_inputs = driver.GetInputs()

    # Update modules
    driver.Synchronize(app.GetSimulationTime())
    terrain.Synchronize(app.GetSimulationTime())
    vehicle.Synchronize(app.GetSimulationTime(), driver_inputs, terrain)
    app.Synchronize("FEDA Demo", driver_inputs)

    # Advance simulation
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    app.Advance(step_size)

    # Maintain real-time pace
    realtime_timer.Spin(step_size)
    step_number += 1