"""
CityBus simulation with data-driven driver (ChDataDriver).
System type: NSC (default for rigid-terrain catalog vehicles).
Main bodies: CityBus chassis, 4 wheels/tires, rigid terrain patch.
Driver: veh.ChDataDriver with pre-programmed sequence:
  t=0.0 -> throttle=0.0, steering=0.0, braking=0.0
  t=0.1 -> throttle=1.0, steering=0.0, braking=0.0
  t=0.5 -> throttle=1.0, steering=0.7, braking=0.0
Expected behavior: bus accelerates straight then turns left.
No time-response settings (not applicable to data-driven driver).
"""


import pychrono.core as chrono
import pychrono.vehicle as veh

# === Constants ===
time_step = 2e-3          # physics step size (s)
sim_end = 20.0            # simulation duration (s)
render_fps = 50.0         # Irrlicht render rate
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once

TERRAIN_LENGTH = 200.0    # terrain patch length (m)
TERRAIN_WIDTH = 200.0     # terrain patch width (m)
INIT_X = 0.0
INIT_Y = 0.0
INIT_Z = 0.5              # CityBus chassis init height above terrain
SUSPENSION_REF_HEIGHT = 0.5  # chassis origin above wheel-bottom at rest (inferred default)

veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# === Vehicle setup (CityBus wrapper) ===
bus = veh.CityBus()
bus.SetContactMethod(chrono.ChContactMethod_NSC)
bus.SetChassisCollisionType(veh.CollisionType_NONE)
bus.SetChassisFixed(False)
bus.SetInitPosition(chrono.ChCoordsysd(
    chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z),
    chrono.QUNIT
))
bus.SetTireType(veh.TireModelType_TMEASY)
bus.SetTireStepSize(time_step)
bus.Initialize()

# === System & bodies (created by the veh.CityBus wrapper) ===
sys = bus.GetSystem()                          # ChSystemNSC owned by the wrapper
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED for contact
chassis = bus.GetChassisBody()                 # cache: main chassis rigid body
# wheels/spindles: bus.GetVehicle().GetAxle(i); terrain: RigidTerrain patch below
# joints: suspension + steering links created inside the wrapper

# === Visualization types ===
bus.SetChassisVisualizationType(veh.VisualizationType_MESH)
bus.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
bus.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
bus.SetWheelVisualizationType(veh.VisualizationType_MESH)
bus.SetTireVisualizationType(veh.VisualizationType_MESH)

# === Terrain ===
terrain = veh.RigidTerrain(sys)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,
    TERRAIN_LENGTH,
    TERRAIN_WIDTH
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Data-driven driver (ChDataDriver) ===
# Pre-programmed sequence: (time, steering, throttle, braking)
driver_data = veh.vector_Entry([
    veh.DataDriverEntry(0.0, 0.0, 0.0, 0.0),   # t=0.0: all zero
    veh.DataDriverEntry(0.1, 0.0, 1.0, 0.0),   # t=0.1: full throttle, no steering
    veh.DataDriverEntry(0.5, 0.7, 1.0, 0.0),   # t=0.5: full throttle + steering
])
driver = veh.ChDataDriver(bus.GetVehicle(), driver_data)
driver.Initialize()

# === Visualization (ChWheeledVehicleVisualSystemIrrlicht) ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("CityBus - Data Driver")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 12.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AttachVehicle(bus.GetVehicle())

# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
frame = 0
try:
    while vis.Run() and sys.GetChTime() < sim_end:
        time = sys.GetChTime()  # cache: current sim time

        if frame % render_every == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()

        driver.Synchronize(time)
        terrain.Synchronize(time)
        bus.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)


        driver.Advance(time_step)
        terrain.Advance(time_step)
        bus.Advance(time_step)
        vis.Advance(time_step)

        frame += 1
        realtime_timer.Spin(time_step)

except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
