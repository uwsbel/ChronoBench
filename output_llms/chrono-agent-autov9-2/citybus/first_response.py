"""CityBus on flat rigid terrain — wheeled-vehicle driving demo (PyChrono + Irrlicht).

Model:
  * A catalog `veh.CityBus` wheeled vehicle (chassis + 2 axles / 4 spindles,
    suspension and steering links created by the wrapper) driving on a flat
    `veh.RigidTerrain` patch with a custom road texture.
  * System type: SMC (the wrapper owns a `ChSystemSMC`); collision via Bullet.
  * Visualization mixes MESH (chassis, wheels, tires) and PRIMITIVE (suspension,
    steering) types, with a chase camera that follows the bus from behind.
  * A scripted `veh.ChDriver` subclass supplies steering / throttle / braking so
    the run is autonomous (no human-in-the-loop): the bus brakes briefly, then
    accelerates forward and makes a gentle steering sweep.

Expected behavior:
  The bus starts with its wheels resting on the terrain (z=0), then drives
  forward and translates several meters while remaining upright. The simulation
  loop advances the full vehicle subsystem stack at a 50 fps render cadence in
  real time.
"""

import math
import os

import pychrono.core as chrono
import pychrono.vehicle as veh

# === Parameters === geometry / timing constants; no bare position literals downstream
TIME_STEP = 1e-3                 # integration step (s)
SIM_END = 12.0                   # total simulated time (s)
RENDER_FPS = 50.0                # review/render cadence (frames per second)
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # precomputed once: physics steps per frame

# CityBus geometry: spindle centers sit ~0.545 m above the chassis-origin INIT_Z,
# so INIT_Z = 0 seats the wheel bottoms on the z=0 terrain plane.
INIT_Z = 0.0
INIT_LOC = chrono.ChVector3d(0.0, 0.0, INIT_Z)
INIT_ROT = chrono.ChQuaterniond(1, 0, 0, 0)   # identity: bus faces +X
TERRAIN_LENGTH = 200.0           # rigid patch X extent (m)
TERRAIN_WIDTH = 100.0            # rigid patch Y extent (m)
TERRAIN_TOP_Z = 0.0              # flat patch top surface height (m)
ZTOL = 0.08                      # allowed wheel-bottom clearance/overlap vs terrain top


# === Driver === scripted time-based control (autonomous; no keyboard input)
class ScriptedDriver(veh.ChDriver):
    """Open-loop steering/throttle/braking as a function of time."""

    def __init__(self, vehicle):
        super().__init__(vehicle)

    def Synchronize(self, time):
        # Brief initial brake to settle the suspension, then drive forward.
        if time < 1.0:
            self.SetThrottle(0.0)
            self.SetBraking(0.8)
        else:
            self.SetThrottle(0.6)
            self.SetBraking(0.0)
        # Gentle steering sweep so the path curves (stays well within stability).
        self.SetSteering(0.25 * math.sin(0.4 * time))


def main():
    # === Vehicle === catalog CityBus; wrapper creates and owns its ChSystemSMC
    bus = veh.CityBus()
    bus.SetContactMethod(chrono.ChContactMethod_SMC)
    bus.SetChassisCollisionType(veh.CollisionType_NONE)
    bus.SetChassisFixed(False)
    bus.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
    bus.SetTireType(veh.TireModelType_TMEASY)      # rolling tire model for rigid road
    bus.SetTireStepSize(TIME_STEP)
    bus.Initialize()

    # Visualization types: MESH for body/wheels/tires, PRIMITIVES for the linkages.
    bus.SetChassisVisualizationType(chrono.VisualizationType_MESH)
    bus.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
    bus.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
    bus.SetWheelVisualizationType(chrono.VisualizationType_MESH)
    bus.SetTireVisualizationType(chrono.VisualizationType_MESH)

    # === System & bodies (created by the veh.CityBus wrapper) ===
    system = bus.GetSystem()                       # ChSystemSMC owned by the wrapper
    chassis = bus.GetChassisBody()                 # cache: main chassis rigid body, reused every step
    veh_obj = bus.GetVehicle()                     # cache: ChWheeledVehicle handle, reused every step
    # spindles: veh_obj.GetSpindlePos(axle, side); joints: suspension + steering links inside wrapper.

    # Bullet collision is REQUIRED — vehicle tires + terrain are contact bodies.
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    # === Terrain === flat rigid patch with a custom road texture
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChContactMaterialSMC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch_mat.SetYoungModulus(2e7)
    patch = terrain.AddPatch(
        patch_mat,
        chrono.CSYSNORM,                           # centered at origin, top at z=0
        TERRAIN_LENGTH,
        TERRAIN_WIDTH,
    )
    patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/tile4.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.8))
    terrain.Initialize()

    # === Footprint assert === wheels must rest ON the terrain, not through it
    tire_radius = veh_obj.GetAxle(0).GetWheel(0).GetTire().GetRadius()  # cache: tire radius
    spindle_world = []
    for axle in range(veh_obj.GetNumberAxles()):
        for side in (veh.LEFT, veh.RIGHT):
            spindle_world.append(veh_obj.GetSpindlePos(axle, side))
    wheel_bottom_z = min(p.z for p in spindle_world) - tire_radius
    assert wheel_bottom_z >= TERRAIN_TOP_Z - ZTOL, (
        f"vehicle sinks into terrain: wheel bottom z={wheel_bottom_z:.3f} "
        f"vs terrain top z={TERRAIN_TOP_Z:.3f}; raise INIT_Z by "
        f"{TERRAIN_TOP_Z - wheel_bottom_z:.3f} m"
    )

    # === Driver === scripted autonomous control
    driver = ScriptedDriver(veh_obj)
    driver.Initialize()

    # === Visualization === vehicle-aware Irrlicht: window + chase camera + sky + lights
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle("CityBus on Rigid Terrain")
    vis.SetWindowSize(1280, 720)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 2.0), 12.0, 0.6)   # follow from behind/above
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddTypicalLights()
    vis.AddGrid(2.0, 2.0, 50, 50,
                chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.01), chrono.QUNIT),
                chrono.ChColor(0.45, 0.45, 0.45))   # ground reference grid
    vis.AttachVehicle(veh_obj)
    vis.AttachDriver(driver)                          # steering/throttle/brake HUD bars


    # === Main loop === Synchronize/Advance the full vehicle stack at render cadence
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
                bus.Synchronize(sim_time, driver_inputs, terrain)
                vis.Synchronize(sim_time, driver_inputs)
                driver.Advance(TIME_STEP)
                terrain.Advance(TIME_STEP)
                bus.Advance(TIME_STEP)               # advances the wrapper-owned system
                vis.Advance(TIME_STEP)
                if system.GetChTime() >= SIM_END:
                    break
    except (RuntimeError, ValueError) as exc:          # solver divergence / bad state
        import traceback
        traceback.print_exc()
        raise


if __name__ == "__main__":
    main()
