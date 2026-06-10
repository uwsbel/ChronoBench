"""Gator wheeled vehicle on flat rigid terrain (PyChrono 9.0.0, Irrlicht).

Models the catalog `veh.Gator` UTV initialized at a chosen location/orientation
with the NSC contact method and TMEASY tires, driven by an interactive keyboard
driver (steering / throttle / braking). The vehicle rolls on a single flat
`veh.RigidTerrain` patch carrying a custom tile texture; all vehicle subsystems
(chassis, wheels, tires, suspension, steering) use mesh visualization. The loop
synchronizes and advances the full driver -> terrain -> vehicle -> vis stack in
real time at 50 rendered frames per second. Expected behavior: the Gator rests on
the terrain and responds to driver inputs (here driven open-loop for the recorded
run), translating across the patch without sinking through it.
"""

import math

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Parameters === geometry / timing constants (no bare literals downstream)
STEP_SIZE = 1e-3                  # integration step (s)
SIM_END = 12.0                    # bounded run length (s) for the recording
RENDER_FPS = 50.0                 # rendered frames per second (real-time target)
RENDER_STEPS = math.ceil((1.0 / RENDER_FPS) / STEP_SIZE)  # precomputed once

TERRAIN_LENGTH = 100.0           # rigid patch X extent (m)
TERRAIN_WIDTH = 100.0            # rigid patch Y extent (m)
TERRAIN_FRICTION = 0.9           # tire-ground friction coefficient
TERRAIN_RESTITUTION = 0.01       # near-inelastic contact

INIT_HEIGHT = 0.5                # chassis-origin spawn height above terrain top (m)
INIT_LOC = chrono.ChVector3d(0, 0, INIT_HEIGHT)
INIT_ROT = chrono.ChQuaterniond(1, 0, 0, 0)   # identity: +X heading

VIS_TYPE = veh.VisualizationType_MESH          # mesh visuals for all components

# === Data paths === locate bundled Chrono + vehicle assets (truth-faithful)
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Vehicle === catalog Gator UTV with NSC contact + TMEASY tires
vehicle = veh.Gator()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)   # rigid terrain -> NSC
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle.SetChassisFixed(False)                         # MANDATORY: fixed chassis won't move
vehicle.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
vehicle.SetTireType(veh.TireModelType_TMEASY)          # prompt: TMEASY tire model
vehicle.SetTireStepSize(STEP_SIZE)
vehicle.Initialize()

# Mesh visualization for every vehicle subsystem (prompt: mesh for all components).
vehicle.SetChassisVisualizationType(VIS_TYPE)
vehicle.SetSuspensionVisualizationType(VIS_TYPE)
vehicle.SetSteeringVisualizationType(VIS_TYPE)
vehicle.SetWheelVisualizationType(VIS_TYPE)
vehicle.SetTireVisualizationType(VIS_TYPE)

# === System & bodies (created and owned by the veh.Gator wrapper) ===
system = vehicle.GetSystem()                       # ChSystemNSC owned by the wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED for contact
chassis = vehicle.GetChassisBody()                 # cache: main chassis body, reused below
# spindles/wheels via vehicle.GetVehicle().GetAxles(); terrain patch body created below;
# suspension + steering joints are created internally by the wrapper.
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())   # report total vehicle mass

# Footprint sanity: all spindles must rest on (not through) the terrain top (z=0).
veh_obj = vehicle.GetVehicle()                     # cache: fetched once for the assert
TIRE_RADIUS = veh_obj.GetAxles()[0].GetWheels()[0].GetTire().GetRadius()
TERRAIN_TOP_Z = 0.0
ZTOL = 0.05
wheel_bottom_z = min(
    veh_obj.GetSpindlePos(axle, side).z
    for axle in range(veh_obj.GetNumberAxles())
    for side in (veh.LEFT, veh.RIGHT)
) - TIRE_RADIUS
assert wheel_bottom_z >= TERRAIN_TOP_Z - ZTOL, (
    f"vehicle sinks into terrain: wheel bottom z={wheel_bottom_z:.3f} "
    f"vs terrain top z={TERRAIN_TOP_Z:.3f}; raise INIT_HEIGHT"
)

# === Terrain === single flat rigid patch with a custom tile texture
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(TERRAIN_FRICTION)
patch_mat.SetRestitution(TERRAIN_RESTITUTION)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)  # custom texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Visualization === vehicle-aware Irrlicht window: sky + chase camera + lights
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Gator on Rigid Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()                          # vehicle truths use directional light
vis.AttachVehicle(vehicle.GetVehicle())

# === Driver === interactive keyboard driver bound to the visual system
driver = veh.ChInteractiveDriverIRR(vis)
render_step_size = 1.0 / RENDER_FPS
driver.SetSteeringDelta(render_step_size / 1.0)    # 1 s to full steering
driver.SetThrottleDelta(render_step_size / 1.0)    # 1 s to full throttle
driver.SetBrakingDelta(render_step_size / 0.3)     # 0.3 s to full brake
driver.Initialize()

# === Main loop === real-time driver -> terrain -> vehicle -> vis sync/advance

realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
frame = 0
try:
    while vis.Run() and system.GetChTime() < SIM_END:
        time = system.GetChTime()

        if step_number % RENDER_STEPS == 0:        # throttled rendering at 50 fps
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()

        driver.Synchronize(time)
        terrain.Synchronize(time)
        vehicle.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        driver.Advance(STEP_SIZE)
        terrain.Advance(STEP_SIZE)
        vehicle.Advance(STEP_SIZE)                  # advances the wrapper-owned system
        vis.Advance(STEP_SIZE)

        step_number += 1
        realtime_timer.Spin(STEP_SIZE)             # match wall-clock to sim time
except (RuntimeError, ValueError) as exc:          # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
