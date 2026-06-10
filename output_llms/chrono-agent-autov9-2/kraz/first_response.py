"""Kraz tractor-and-semitrailer driving on flat rigid terrain (PyChrono 9.0.1).

Model
-----
- A Kraz articulated heavy vehicle (tractor + semitrailer) created by the
  `veh.Kraz` catalog wrapper, which owns its own `ChSystemNSC` and a fixed
  TMEASY-class tire model. The tractor is a `ChWheeledVehicle`.
- A flat `veh.RigidTerrain` patch with defined friction and restitution provides
  the driving surface.
- A scripted `veh.ChDriver` subclass releases the brake and applies throttle with
  a gentle sinusoidal steering law so the rig accelerates forward and turns.

System type: NSC (the Kraz wrapper builds a ChSystemNSC; the Bullet collision
system is set explicitly on it after Initialize).

Expected behavior
------------------
The tractor-trailer starts at rest with wheels seated on the terrain (z=0),
brakes briefly, then accelerates forward while gently steering, remaining upright
throughout. A chase camera follows the tractor chassis.
"""

# === Imports ===
import math
import os
import pychrono.core as chrono
import pychrono.vehicle as veh

# === Named constants: geometry / physics / driver schedule ===
TIME_STEP = 2e-3                       # integration step (s)
TIRE_STEP = 1e-3                       # tire substep (s)
SIM_END = 12.0                         # simulation duration (s)
RENDER_FPS = 50.0                      # review render cadence (frames/s)

TIRE_RADIUS = 0.5588                   # Kraz tire radius (m) — wheel-bottom offset
GROUND_CLEARANCE = 0.02                # small lift so wheels start just on z=0
INIT_X = 0.0                           # spawn X (m)
INIT_Y = 0.0                           # spawn Y (m)
INIT_Z = TIRE_RADIUS + GROUND_CLEARANCE  # chassis-origin Z seats wheels on z=0

TERRAIN_LENGTH = 300.0                 # rigid patch X extent (m)
TERRAIN_WIDTH = 300.0                  # rigid patch Y extent (m)
TERRAIN_FRICTION = 0.9                 # patch friction coefficient
TERRAIN_RESTITUTION = 0.01             # patch restitution (bounciness)

BRAKE_RELEASE_TIME = 1.0               # hold brake until this time (s)
DRIVE_THROTTLE = 0.6                   # throttle after brake release
STEER_AMPLITUDE = 0.25                 # peak steering (-1..+1)
STEER_RATE = 0.4                       # steering sinusoid angular rate (rad/s)

# Derived once (precomputed once — never recompute inside the loop).
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # precomputed once


# === Driver: scripted time-based control law ===
# Brake first so the rig settles, then throttle forward with gentle steering.
class ScriptedDriver(veh.ChDriver):
    def __init__(self, vehicle):
        super().__init__(vehicle)

    def Synchronize(self, time):
        if time < BRAKE_RELEASE_TIME:
            self.SetThrottle(0.0)
            self.SetBraking(1.0)
        else:
            self.SetThrottle(DRIVE_THROTTLE)
            self.SetBraking(0.0)
        self.SetSteering(STEER_AMPLITUDE * math.sin(STEER_RATE * time))


def main():

    # === Vehicle (Kraz tractor + semitrailer; wrapper owns the ChSystem) ===
    init_loc = chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z)
    init_rot = chrono.QUNIT
    kraz = veh.Kraz()
    kraz.SetContactMethod(chrono.ChContactMethod_NSC)
    kraz.SetChassisFixed(False)
    kraz.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
    kraz.SetTireStepSize(TIRE_STEP)
    kraz.Initialize()

    # Visualization setters take (tractor, trailer) except steering (one arg).
    kraz.SetChassisVisualizationType(chrono.VisualizationType_MESH,
                                     chrono.VisualizationType_MESH)
    kraz.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES,
                                        chrono.VisualizationType_PRIMITIVES)
    kraz.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
    kraz.SetWheelVisualizationType(chrono.VisualizationType_MESH,
                                   chrono.VisualizationType_MESH)
    kraz.SetTireVisualizationType(chrono.VisualizationType_MESH,
                                  chrono.VisualizationType_MESH)

    # === System & components (created by the veh.Kraz wrapper) ===
    sys = kraz.GetSystem()                      # ChSystemNSC owned by the wrapper
    tractor = kraz.GetTractor()                 # ChWheeledVehicle (speed/spindles)
    chassis_body = kraz.GetTractorChassisBody()  # cache: chassis rigid body, reused
    # Bullet collision is REQUIRED for vehicle+terrain contact; set after Initialize.
    sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    # Seat-check: tractor wheel bottoms must rest on (not through) the terrain.
    num_axles = tractor.GetNumberAxles()
    wheel_bottom_z = min(
        tractor.GetSpindlePos(ax, side).z
        for ax in range(num_axles)
        for side in (veh.LEFT, veh.RIGHT)
    ) - TIRE_RADIUS
    assert wheel_bottom_z >= -0.1, (
        f"tractor sinks into terrain: wheel bottom z={wheel_bottom_z:.3f}; "
        f"raise INIT_Z"
    )

    # === Terrain (flat rigid patch with defined friction / restitution) ===
    terrain = veh.RigidTerrain(sys)
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(TERRAIN_FRICTION)
    patch_mat.SetRestitution(TERRAIN_RESTITUTION)
    patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM,
                             TERRAIN_LENGTH, TERRAIN_WIDTH)
    patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/tile4.jpg"),
                     200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    # === Driver (scripted brake-then-throttle with gentle steering) ===
    driver = ScriptedDriver(tractor)
    driver.Initialize()

    # === Visualization === vehicle Irrlicht window: chase cam + sky + lights + logo
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle("Kraz tractor-trailer on rigid terrain")
    vis.SetWindowSize(1280, 720)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 14.0, 0.8)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddTypicalLights()
    vis.AttachVehicle(tractor)
    vis.AttachDriver(driver)


    # === Main loop === render-cadence outer loop; Synchronize/Advance per step
    try:
        while vis.Run() and sys.GetChTime() < SIM_END:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            for _ in range(RENDER_EVERY):
                time = sys.GetChTime()
                driver_inputs = driver.GetInputs()
                driver.Synchronize(time)
                terrain.Synchronize(time)
                kraz.Synchronize(time, driver_inputs, terrain)
                vis.Synchronize(time, driver_inputs)
                driver.Advance(TIME_STEP)
                terrain.Advance(TIME_STEP)
                kraz.Advance(TIME_STEP)       # advances the wrapper-owned system
                vis.Advance(TIME_STEP)
                if sys.GetChTime() >= SIM_END:
                    break
    except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
        import traceback
        traceback.print_exc()
        raise
    finally:
        pass


if __name__ == "__main__":
    main()
