"""MAN 10t heavy truck driving on flat rigid terrain (PyChrono 9.0.x + Irrlicht).

Model
-----
- System: NSC multibody system OWNED by the ``veh.MAN_10t`` wrapper (chassis,
  eight wheels / spindles, suspension + steering links are all created inside it).
- Tire model: TMEASY (handling tire model) on every wheel, stepped at the tire
  step size.
- Terrain: a single flat ``veh.RigidTerrain`` patch (Bullet contact) with a tiled
  dirt texture, sized to comfortably contain the driving maneuver.
- Driver: a scripted ``veh.ChDriver`` subclass providing time-based steering,
  throttle and braking inputs (no human-in-the-loop, which would be inert in a
  windowed batch run).

Expected behavior
------------------
The truck brakes briefly to settle on its tires, then accelerates forward under
throttle while applying a gentle sinusoidal steering input, so the chassis
translates several metres across the terrain over the simulated interval. A
chase camera follows the chassis with a skybox, directional lights and a logo.
"""

# === Imports ===
import math
import os
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Named constants === geometry / physics / timing (no bare literals downstream)
TIME_STEP = 2.0e-3                       # integration + tire step (s)
TIRE_STEP = 1.0e-3                       # TMEASY tire sub-step (s)
SIM_END = 12.0                           # simulated duration (s)
RENDER_FPS = 50.0                        # review-render cadence (frames/s)

TERRAIN_LENGTH = 200.0                   # rigid patch X extent (m)
TERRAIN_WIDTH = 100.0                    # rigid patch Y extent (m)
TERRAIN_TOP_Z = 0.0                      # top surface height of the patch (m)

SUSPENSION_REF_HEIGHT = 1.05             # MAN 10t chassis origin above wheel-bottom at rest (m)
TIRE_RADIUS = 0.6                        # nominal MAN 10t tire radius for footprint check (m)
ZTOL = 0.10                              # allowed wheel-bottom clearance/overlap vs terrain (m)

INIT_X = -TERRAIN_LENGTH / 2.0 + 20.0    # spawn well inside the patch, room to drive +X
INIT_Y = 0.0
INIT_Z = TERRAIN_TOP_Z + SUSPENSION_REF_HEIGHT   # derived chassis-origin height
INIT_LOC = chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z)
INIT_ROT = chrono.QUNIT                  # facing +X

RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # precomputed once


# === Driver === scripted time-based steering / throttle / braking controls
class ScriptedDriver(veh.ChDriver):
    """Open-loop controller: settle on brakes, then accelerate with a slow weave."""

    def __init__(self, vehicle):
        super().__init__(vehicle)

    def Synchronize(self, time):
        # Brief brake hold lets the suspension settle before drive-off.
        if time < 1.0:
            self.SetThrottle(0.0)
            self.SetBraking(0.8)
        else:
            self.SetThrottle(0.6)
            self.SetBraking(0.0)
        # Gentle sinusoidal steering so the truck curves visibly but stays on patch.
        self.SetSteering(0.20 * math.sin(0.4 * time))


# === Vehicle === MAN 10t wrapper owns the ChSystem, chassis, wheels and joints
truck = veh.MAN_10t()
truck.SetContactMethod(chrono.ChContactMethod_NSC)
truck.SetChassisCollisionType(veh.CollisionType_NONE)
truck.SetChassisFixed(False)
truck.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
truck.SetEngineType(veh.EngineModelType_SIMPLE_MAP)
truck.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
truck.SetTireType(veh.TireModelType_TMEASY)   # prompt: TMEASY tire model
truck.SetTireStepSize(TIRE_STEP)
truck.Initialize()

truck.SetChassisVisualizationType(chrono.VisualizationType_MESH)
truck.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
truck.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
truck.SetWheelVisualizationType(chrono.VisualizationType_MESH)
truck.SetTireVisualizationType(chrono.VisualizationType_MESH)

# === System & bodies (created by the veh.MAN_10t wrapper) ===
sys = truck.GetSystem()                       # ChSystemNSC owned by the wrapper
chassis = truck.GetChassisBody()              # cache: main chassis rigid body, reused every step
veh_obj = truck.GetVehicle()                  # cache: ChWheeledVehicle handle for spindle queries
# wheels/spindles: veh_obj.GetSpindlePos(axle, side); joints: suspension + steering links internal.

# Collision/contact scene -> Bullet collision system (vehicle tires vs rigid terrain).
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Terrain === flat rigid patch with friction + tiled texture, attached to the owned system
terrain = veh.RigidTerrain(sys)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/dirt.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Footprint check === assert the wheels rest on (not through) the terrain after Initialize
spindle_world = []
for axle in range(veh_obj.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        spindle_world.append(veh_obj.GetSpindlePos(axle, side))
wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
assert wheel_bottom_z >= TERRAIN_TOP_Z - ZTOL, (
    f"vehicle sinks into terrain: wheel bottom z={wheel_bottom_z:.3f} "
    f"vs terrain top z={TERRAIN_TOP_Z:.3f}; raise SUSPENSION_REF_HEIGHT by "
    f"{TERRAIN_TOP_Z - wheel_bottom_z:.3f} m"
)

# === Driver === instantiate the scripted controller on the vehicle
driver = ScriptedDriver(veh_obj)
driver.Initialize()

# === Visualization === vehicle-aware Irrlicht: window + chase cam + sky + lights + logo
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("MAN 10t on Rigid Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 2.0), 14.0, 1.0)   # chase camera behind chassis
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()                          # standard directional lighting
vis.AddLightDirectional()                       # extra directional sun light
vis.AttachVehicle(veh_obj)
vis.AttachDriver(driver)                         # steering / throttle / brake HUD bars

# === Output setup === guard against a missing output directory before logging

# === Main loop === render-cadence outer loop; advance the full vehicle subsystem stack
try:

    frame = 0
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(RENDER_EVERY):
            sim_time = sys.GetChTime()
            driver_inputs = driver.GetInputs()
            driver.Synchronize(sim_time)
            terrain.Synchronize(sim_time)
            truck.Synchronize(sim_time, driver_inputs, terrain)
            vis.Synchronize(sim_time, driver_inputs)
            driver.Advance(TIME_STEP)
            terrain.Advance(TIME_STEP)
            truck.Advance(TIME_STEP)          # advances the wrapper-owned ChSystem
            vis.Advance(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / invalid vehicle state
    import traceback
    traceback.print_exc()
    raise
finally:
    # Always release the render device, even if a step diverged mid-run.
    vis.GetDevice().closeDevice()

# === Post-processing === assemble review video + timeseries plot, then clean frames
