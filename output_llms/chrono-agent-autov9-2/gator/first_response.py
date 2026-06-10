"""Gator UTV on flat rigid terrain (PyChrono 9.0.x + Irrlicht).

Models the John Deere Gator wheeled utility vehicle driving on a flat
RigidTerrain patch. System type is NSC (the Gator wrapper builds and owns its
own ChSystemNSC). The main rigid bodies are the wrapper-created chassis, four
spindles/wheels, and the fixed terrain patch body. The vehicle uses the TMEASY
tire model and mesh visualization for every component (chassis, suspension,
steering, wheels, tires).

A scripted driver supplies steering, throttle, and braking through the standard
DriverInputs channel, so the vehicle brakes briefly, then accelerates forward
while gently steering. Expected behavior: the Gator starts at rest on the
terrain, then rolls forward (positive X displacement) with its wheels turning.
The render/physics loop runs in real time at 50 frames per second.
"""

# === Imports ===
import math
import os
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Named constants: geometry / physics / control ===
TIME_STEP = 1.0e-3                       # integration step (s)
TIRE_STEP = 1.0e-3                       # TMEASY tire sub-step (s)
SIM_END = 10.0                           # simulated duration (s)
RENDER_FPS = 50.0                        # real-time review/render rate (fps)

TERRAIN_LENGTH = 100.0                   # rigid terrain X extent (m)
TERRAIN_WIDTH = 100.0                    # rigid terrain Y extent (m)
TERRAIN_HEIGHT = 0.0                     # top surface Z of the flat patch (m)
FRICTION = 0.9                           # terrain/tire friction coefficient
RESTITUTION = 0.01                       # terrain restitution (near-inelastic)

SUSPENSION_REF_HEIGHT = 0.5              # chassis origin above wheel-bottom at rest (m)
INIT_X = 0.0                             # spawn X on the terrain (m)
INIT_Y = 0.0                             # spawn Y on the terrain (m)
INIT_Z = TERRAIN_HEIGHT + SUSPENSION_REF_HEIGHT   # derived chassis origin height
INIT_YAW = 0.0                           # initial heading about world Z (rad)

TIRE_RADIUS = 0.30                       # nominal Gator tire radius (m), for footprint check
ZTOL = 0.10                              # allowed wheel-bottom clearance/overlap vs terrain (m)

# Derived render cadence — precomputed once (never recomputed in the loop).
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # physics steps per frame


# === Driver: scripted steering / throttle / braking ===
# A ChDriver subclass drives the vehicle through the standard input channel
# (steering, throttle, braking) without keyboard input, so the maneuver is
# reproducible in a non-interactive run.
class ScriptedDriver(veh.ChDriver):
    def __init__(self, vehicle):
        super().__init__(vehicle)

    def Synchronize(self, time):
        # Hold the brake briefly so the vehicle settles, then accelerate.
        if time < 0.5:
            self.SetThrottle(0.0)
            self.SetBraking(1.0)
        else:
            self.SetThrottle(0.6)
            self.SetBraking(0.0)
        # Gentle sinusoidal steering exercises the steering input.
        self.SetSteering(0.3 * math.sin(0.5 * time))


def main():
    # === Vehicle (Gator wrapper owns its ChSystemNSC) ===
    # Configure spawn pose, contact method, and the TMEASY tire model, then
    # initialize; the wrapper creates the chassis, spindles, wheels, and joints.
    init_loc = chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z)
    init_rot = chrono.QuatFromAngleZ(INIT_YAW)

    gator = veh.Gator()
    gator.SetContactMethod(chrono.ChContactMethod_NSC)          # prompt: contact method
    gator.SetChassisCollisionType(veh.CollisionType_NONE)
    gator.SetChassisFixed(False)
    gator.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))   # prompt: location + orientation
    gator.SetTireType(veh.TireModelType_TMEASY)                 # prompt: TMEASY tire model
    gator.SetTireStepSize(TIRE_STEP)
    gator.Initialize()

    # Mesh visualization for ALL vehicle components (prompt requirement).
    gator.SetChassisVisualizationType(chrono.VisualizationType_MESH)
    gator.SetSuspensionVisualizationType(chrono.VisualizationType_MESH)
    gator.SetSteeringVisualizationType(chrono.VisualizationType_MESH)
    gator.SetWheelVisualizationType(chrono.VisualizationType_MESH)
    gator.SetTireVisualizationType(chrono.VisualizationType_MESH)

    # === System & bodies (created by the veh.Gator wrapper) ===
    system = gator.GetSystem()                  # ChSystemNSC owned by the wrapper
    veh_obj = gator.GetVehicle()                # cache: vehicle handle, reused below
    chassis = gator.GetChassisBody()            # cache: main chassis rigid body, reused every step
    # spindles/wheels: veh_obj.GetSpindlePos(axle, side) ; terrain: RigidTerrain patch body below
    # joints: suspension + steering links are created inside the wrapper

    # === Collision system === Bullet narrow-phase for tire/terrain contact
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    # === Terrain === flat rigid patch with a custom texture (prompt: dimensions + texture)
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(FRICTION)
    patch_mat.SetRestitution(RESTITUTION)

    terrain = veh.RigidTerrain(system)
    patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
    patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/tile4.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    # === Footprint check === assert the wheels rest on (not through) the terrain
    spindle_world = []
    for axle in range(veh_obj.GetNumberAxles()):
        for side in (veh.LEFT, veh.RIGHT):
            spindle_world.append(veh_obj.GetSpindlePos(axle, side))
    wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
    assert wheel_bottom_z >= TERRAIN_HEIGHT - ZTOL, (
        f"vehicle sinks into terrain: wheel bottom z={wheel_bottom_z:.3f} "
        f"vs terrain top z={TERRAIN_HEIGHT:.3f}; raise SUSPENSION_REF_HEIGHT by "
        f"{TERRAIN_HEIGHT - wheel_bottom_z:.3f} m"
    )

    # === Driver === scripted steering / throttle / braking inputs
    driver = ScriptedDriver(veh_obj)
    driver.Initialize()

    # === Visualization === vehicle-aware Irrlicht: window + chase camera + sky + lights + logo
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle("Gator on Rigid Terrain")
    vis.SetWindowSize(1280, 720)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.5), 8.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddTypicalLights()
    vis.AttachVehicle(veh_obj)
    vis.AttachDriver(driver)

    # === Output setup ===

    # === Main loop === real-time render-cadence loop; physics in inner batches
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
                gator.Synchronize(sim_time, driver_inputs, terrain)
                vis.Synchronize(sim_time, driver_inputs)
                driver.Advance(TIME_STEP)
                terrain.Advance(TIME_STEP)
                gator.Advance(TIME_STEP)            # internally steps the wrapper-owned system
                vis.Advance(TIME_STEP)
                if system.GetChTime() >= SIM_END:
                    break
    except (RuntimeError, ValueError) as exc:       # solver divergence / bad state
        import traceback
        traceback.print_exc()
        raise


if __name__ == "__main__":
    main()
