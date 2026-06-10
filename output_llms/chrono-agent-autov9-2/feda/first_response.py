"""FEDA wheeled vehicle driving on flat rigid terrain (PyChrono 9.0.1, Irrlicht).

Model: a single FEDA (Ford Expedition Demo Article) catalog vehicle is created
through the `veh.FEDA()` wrapper, which owns its own SMC ChSystem, chassis rigid
body, four wheel/spindle bodies, suspension + steering links, engine, transmission,
and PAC02 tires. The vehicle is placed on a flat `veh.RigidTerrain` patch carrying a
custom tile texture. All vehicle parts (chassis, wheels, tires, suspension, steering)
use the MESH visualization type.

System type: SMC (smooth/penalty contact), as required by the FEDA wrapper and the
rigid terrain patch. Collision detection uses the Bullet collision system.

Control: a scripted driver (a `veh.ChDriver` subclass standing in for the interactive
steering/throttle/braking controls) commands a short brake-then-accelerate schedule
with a gentle steering sweep, so the vehicle pulls away from rest and is observed to
translate across the terrain.

Expected behavior: the four tires rest on the terrain at spawn; after the initial
brake-release the FEDA accelerates forward (+X) under throttle while the chase camera
tracks it, and the body steers mildly left/right following the scripted steering law.
The simulation renders in real time at 50 frames per second.
"""

# === Imports ===
import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Named constants === geometry / physics / timing (no bare literals downstream)
TIME_STEP = 1e-3                       # integration step (s)
SIM_END = 10.0                         # total simulated time (s)
RENDER_FPS = 50.0                      # review + on-screen frame rate (fps)
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once: steps per frame

TERRAIN_LENGTH = 200.0                 # rigid terrain patch X extent (m)
TERRAIN_WIDTH = 100.0                  # rigid terrain patch Y extent (m)
TERRAIN_TOP_Z = 0.0                    # terrain surface height (m)

SUSPENSION_REF_HEIGHT = 0.5            # FEDA chassis-origin height above wheel-bottom at rest (m)
TIRE_RADIUS = 0.4987                   # FEDA PAC02 tire radius (m), from the tire geometry
ZTOL = 0.05                            # allowed wheel-bottom clearance/overlap vs terrain top (m)

INIT_X = 0.0                           # spawn X on the terrain (m)
INIT_Y = 0.0                           # spawn Y on the terrain (m)
INIT_Z = TERRAIN_TOP_Z + SUSPENSION_REF_HEIGHT  # derived chassis-origin Z so wheels rest on terrain
init_loc = chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z)
init_rot = chrono.QUNIT                # facing +X

# Chase-camera placement (relative to chassis), precomputed once.
CHASE_POINT = chrono.ChVector3d(0.0, 0.0, 1.75)  # tracked point on chassis
CHASE_DISTANCE = 8.0                   # camera distance behind vehicle (m)
CHASE_HEIGHT = 0.6                     # camera height offset (m)


# === Driver === scripted steering/throttle/braking control (ChDriver subclass)
class ScriptedDriver(veh.ChDriver):
    """Time-based steering/throttle/braking law standing in for interactive controls."""

    def __init__(self, vehicle):
        super().__init__(vehicle)

    def Synchronize(self, time):
        # Hold the brake briefly, then accelerate forward and steer gently.
        if time < 1.0:
            self.SetThrottle(0.0)
            self.SetBraking(1.0)
        else:
            self.SetThrottle(0.6)
            self.SetBraking(0.0)
        self.SetSteering(0.25 * math.sin(0.4 * time))


def build_and_run():
    # === Vehicle === FEDA wrapper owns its SMC system, chassis, wheels, tires, links
    feda = veh.FEDA()
    feda.SetContactMethod(chrono.ChContactMethod_SMC)
    feda.SetChassisCollisionType(veh.CollisionType_NONE)
    feda.SetChassisFixed(False)
    feda.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
    feda.SetEngineType(veh.EngineModelType_SIMPLE_MAP)
    feda.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
    feda.SetTireType(veh.TireModelType_PAC02)          # prompt: specific tire model
    feda.SetTireStepSize(TIME_STEP)
    feda.Initialize()

    # Mesh visualization for ALL vehicle parts (prompt: mesh visualization type).
    feda.SetChassisVisualizationType(chrono.VisualizationType_MESH)
    feda.SetSuspensionVisualizationType(chrono.VisualizationType_MESH)
    feda.SetSteeringVisualizationType(chrono.VisualizationType_MESH)
    feda.SetWheelVisualizationType(chrono.VisualizationType_MESH)
    feda.SetTireVisualizationType(chrono.VisualizationType_MESH)

    # === System & bodies (created by the veh.FEDA wrapper) ===
    system = feda.GetSystem()                  # cache: ChSystemSMC owned by the wrapper, fetched once
    veh_obj = feda.GetVehicle()                # cache: ChWheeledVehicle handle, reused below
    chassis = feda.GetChassisBody()            # cache: main chassis rigid body
    # wheels/spindles: veh_obj.GetSpindlePos(axle, side); links: suspension + steering inside wrapper

    # === Collision system === Bullet, required for vehicle/terrain contact (set after Initialize)
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    # === Footprint assertion === verify the wheels rest ON the terrain, not through it
    spindle_world = []
    for axle in range(veh_obj.GetNumberAxles()):
        for side in (veh.LEFT, veh.RIGHT):
            spindle_world.append(veh_obj.GetSpindlePos(axle, side))
    wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS  # precomputed once after Initialize
    assert wheel_bottom_z >= TERRAIN_TOP_Z - ZTOL, (
        f"FEDA sinks into terrain: wheel bottom z={wheel_bottom_z:.3f} vs terrain "
        f"top z={TERRAIN_TOP_Z:.3f}; raise SUSPENSION_REF_HEIGHT by "
        f"{TERRAIN_TOP_Z - wheel_bottom_z:.3f} m"
    )

    # === Terrain === flat rigid patch with a custom tile texture
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChContactMaterialSMC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch_mat.SetYoungModulus(2e7)
    patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
    patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/tile4.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    # === Driver === scripted controller for steering / throttle / braking
    driver = ScriptedDriver(veh_obj)
    driver.Initialize()

    # === Visualization === vehicle-aware Irrlicht: window + chase cam + sky + lights + logo
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle("FEDA on Rigid Terrain")
    vis.SetWindowSize(1280, 720)
    vis.SetChaseCamera(CHASE_POINT, CHASE_DISTANCE, CHASE_HEIGHT)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddTypicalLights()
    vis.AttachVehicle(veh_obj)
    vis.AttachDriver(driver)


    # === Main loop === render once per frame (50 fps); advance the full subsystem stack
    frame = 0
    try:
        while vis.Run() and system.GetChTime() < SIM_END:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            for _ in range(RENDER_EVERY):
                sim_time = system.GetChTime()
                driver_inputs = driver.GetInputs()
                driver.Synchronize(sim_time)
                terrain.Synchronize(sim_time)
                veh_obj.Synchronize(sim_time, driver_inputs, terrain)
                vis.Synchronize(sim_time, driver_inputs)
                driver.Advance(TIME_STEP)
                terrain.Advance(TIME_STEP)
                veh_obj.Advance(TIME_STEP)        # advances the wrapper-owned system
                vis.Advance(TIME_STEP)
                if system.GetChTime() >= SIM_END:
                    break
    except (RuntimeError, ValueError) as exc:     # solver divergence / bad state
        import traceback
        traceback.print_exc()
        raise

    # === Post-processing === close writers, assemble videos + plot, drop frame dirs (review only)


# === Entry point ===
if __name__ == "__main__":
    build_and_run()
