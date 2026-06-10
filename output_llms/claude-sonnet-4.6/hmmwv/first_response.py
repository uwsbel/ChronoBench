import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# ---------------------
# Simulation Parameters
# ---------------------
step_size = 1e-3          # Simulation step size
tire_step_size = 1e-3     # Tire simulation step size
render_fps = 50           # Rendering frames per second
render_step_size = 1.0 / render_fps

# --------------------------
# Vehicle Initial Conditions
# --------------------------
initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts
vis_type = veh.VisualizationType_PRIMITIVES

# Collision type for chassis & wheels
chassis_collision_type = veh.CollisionType_NONE

# Contact method
contact_method = chrono.ChContactMethod_NSC

# -------------------------
# Initialize PyChrono System
# -------------------------
# Create the HMMWV vehicle
hmmwv = veh.HMMWV_Full()

hmmwv.SetContactMethod(contact_method)
hmmwv.SetChassisCollisionType(chassis_collision_type)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
hmmwv.SetEngineType(veh.EngineModelType_SIMPLE)
hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
hmmwv.SetTireType(veh.TireModelType_TMEASY)
hmmwv.SetTireStepSize(tire_step_size)

hmmwv.Initialize()

# Set visualization types for vehicle components
hmmwv.SetChassisVisualizationType(vis_type)
hmmwv.SetSuspensionVisualizationType(vis_type)
hmmwv.SetSteeringVisualizationType(vis_type)
hmmwv.SetWheelVisualizationType(vis_type)
hmmwv.SetTireVisualizationType(vis_type)

# ------------------
# Create the Terrain
# ------------------
terrain = veh.RigidTerrain(hmmwv.GetSystem())

# Define terrain patch
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.CSYSNORM.rot),
    200.0,   # length
    200.0    # width
)

patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))

terrain.Initialize()

# -----------------------
# Create the Irrlicht App
# -----------------------
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV Full Vehicle Demo")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 0.75), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(hmmwv.GetVehicle())

# -----------------------
# Create the Driver System
# -----------------------
driver = veh.ChInteractiveDriverIRR(vis)

# Set driver input gains (steering, throttle, braking)
driver.SetSteeringDelta(render_step_size / 1.0)   # time to go from 0 to max steer
driver.SetThrottleDelta(render_step_size / 1.0)   # time to go from 0 to max throttle
driver.SetBrakingDelta(render_step_size / 0.3)    # time to go from 0 to max braking

driver.Initialize()

# Print vehicle info
print("HMMWV Full Vehicle Simulation")
print(f"  Contact method     : NSC")
print(f"  Tire model         : TMEASY")
print(f"  Visualization      : Primitives")
print(f"  Simulation step    : {step_size}")
print(f"  Render FPS         : {render_fps}")
print("-------------------------------")
print("Controls:")
print("  W/S       : throttle / brake")
print("  A/D       : steer left / right")
print("  ESC       : quit")
print("-------------------------------")

# ----------------
# Simulation Loop
# ----------------
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_steps = int(render_step_size / step_size)

while vis.Run():
    time = hmmwv.GetSystem().GetChTime()

    # Render scene at specified FPS
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    # Collect driver inputs
    driver_inputs = driver.GetInputs()

    # Update modules (order matters!)
    driver.Synchronize(time)
    terrain.Synchronize(time)
    hmmwv.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance simulation
    driver.Advance(step_size)
    terrain.Advance(step_size)
    hmmwv.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1

    # Enforce real-time simulation
    realtime_timer.Spin(step_size)