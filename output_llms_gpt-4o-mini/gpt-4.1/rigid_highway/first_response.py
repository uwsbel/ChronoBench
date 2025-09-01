import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import os

# -----------------------------------------------------------------------------
# INITIALIZATION
# -----------------------------------------------------------------------------

# Set Chrono data directories
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(veh.GetDataPath())

# Simulation parameters
step_size = 1.0 / 500.0
render_step_size = 1.0 / 50.0  # 50 FPS

# Vehicle initial position and orientation
initLoc = chrono.ChVectorD(0, 0.5, 0)
initYaw = chrono.CH_C_PI / 12  # 15 degrees

# Contact method
contact_method = chrono.ChContactMethod_NSC

# -----------------------------------------------------------------------------
# CREATE THE PHYSICAL SYSTEM
# -----------------------------------------------------------------------------

system = chrono.ChSystemNSC()

# -----------------------------------------------------------------------------
# CREATE THE VEHICLE
# -----------------------------------------------------------------------------

vehicle = veh.HMMWV_Full(system)
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysD(initLoc, chrono.Q_from_AngY(initYaw)))
vehicle.SetInitFwdVel(0.0)
vehicle.SetPowertrainType(veh.PowertrainModelType_SHAFTS)
vehicle.SetDriveType(veh.DrivelineTypeWV_AWD)
vehicle.SetTireType(veh.TireModelType_TMEASY)
vehicle.SetTireStepSize(step_size)
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)
vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_MESH)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.Initialize()

# -----------------------------------------------------------------------------
# CREATE THE TERRAIN (MESH-BASED)
# -----------------------------------------------------------------------------

terrain = veh.RigidTerrain(system)

# Path to mesh files
mesh_dir = os.getcwd()
collision_mesh = os.path.join(mesh_dir, "Highway_col.obj")
visual_mesh = os.path.join(mesh_dir, "Highway_vis.obj")

patch = terrain.AddPatch(
    chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT),
    collision_mesh,  # collision mesh
    visual_mesh      # visual mesh
)
patch.SetContactFrictionCoefficient(0.9)
patch.SetContactRestitutionCoefficient(0.01)
patch.SetContactMaterialProperties(2e7, 0.3)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.8))
patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 200, 200)
terrain.Initialize()

# -----------------------------------------------------------------------------
# CREATE THE DRIVER (INTERACTIVE)
# -----------------------------------------------------------------------------

driver = veh.ChIrrGuiDriver(
    veh.ChVehicleIrrApp(vehicle, "HMMWV on Mesh Terrain", irr.dimension2du(1280, 720))
)
driver.SetSteeringDelta(0.04)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.1)
driver.Initialize()

# -----------------------------------------------------------------------------
# IRRLICHT VISUALIZATION
# -----------------------------------------------------------------------------

app = driver.GetApp()
app.AddTypicalLights()
app.AddTypicalSky()
app.AddTypicalLogo()
app.SetChaseCamera(chrono.ChVectorD(0.0, 0.5, 0.0), 6.0, 0.5)
app.AssetBindAll()
app.AssetUpdateAll()

# -----------------------------------------------------------------------------
# SIMULATION LOOP
# -----------------------------------------------------------------------------

realtime_timer = chrono.ChRealtimeStepTimer()
render_steps = int(render_step_size / step_size)
step_number = 0

while app.GetDevice().run():
    app.BeginScene()
    app.DrawAll()

    # Get driver inputs
    driver_inputs = driver.GetInputs()

    # Update modules
    driver.Synchronize(app.GetSimulationTime())
    terrain.Synchronize(app.GetSimulationTime())
    vehicle.Synchronize(app.GetSimulationTime(), driver_inputs, terrain)
    app.Synchronize("HMMWV Simulation", driver_inputs)

    # Advance simulation
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    app.Advance(step_size)

    # Render at specified FPS
    if step_number % render_steps == 0:
        pass  # Rendering handled by Irrlicht

    chrono.ChRealtimeStepTimer().Spin(step_size)
    step_number += 1

    app.EndScene()

    # Optional: exit after a certain time
    # if app.GetSimulationTime() > 60:
    #     break