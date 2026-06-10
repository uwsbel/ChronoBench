"""HMMWV full vehicle on flat rigid terrain driven by a custom scripted driver.

System type: NSC (rigid-terrain catalog vehicle, ChContactMethod_NSC).
Main bodies: HMMWV_Full chassis + 4 wheels/spindles + suspension/steering links
(all created by the veh.HMMWV_Full wrapper), and a flat RigidTerrain patch.

Driver: a custom MyDriver(veh.ChDriver) subclass that replaces the default driver.
It applies a 0.5 s input delay, then ramps throttle smoothly up to 0.7 over the
0.2 s following the delay, and applies a sinusoidal steering pattern that starts
at t = 2 s. Braking stays released once the delay elapses.

Expected behavior: the vehicle stays put for the first 0.5 s, then accelerates
forward to a steady throttle of 0.7, driving roughly straight until t = 2 s and
then weaving left/right under the sinusoidal steering. The run ends at t = 4 s.
"""

import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Simulation constants === geometry / timing / driver control law
TIME_STEP = 1e-3                 # integration step (s)
TIRE_STEP = 1e-3                 # tire substep (s)
SIM_END = 4.0                    # end the simulation at t = 4 s (prompt condition)
RENDER_FPS = 50.0
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # precomputed once

DRIVER_DELAY = 0.5               # input delay before any control is applied (s)
THROTTLE_TARGET = 0.7            # steady throttle reached after the ramp
THROTTLE_RAMP = 0.2             # ramp duration to reach THROTTLE_TARGET (s)
STEER_START = 2.0                # sinusoidal steering begins at t = 2 s
STEER_AMPLITUDE = 0.5            # steering amplitude (-1..+1 input units)
STEER_FREQ = 0.5                 # steering angular frequency (rad/s)

TERRAIN_LENGTH = 200.0           # rigid terrain patch size in X (m)
TERRAIN_WIDTH = 200.0            # rigid terrain patch size in Y (m)
INIT_LOC = chrono.ChVector3d(0, 0, 0.5)   # chassis spawn (HMMWV origin = geometric center)
INIT_ROT = chrono.ChQuaterniond(1, 0, 0, 0)


# === Custom driver === scripted, time-based control replacing the default driver
class MyDriver(veh.ChDriver):
    """Custom driver: delayed start, throttle ramp to 0.7, sinusoidal steering."""

    def __init__(self, vehicle, delay):
        super().__init__(vehicle)
        self.delay = delay       # input delay before control is applied (s)

    def Synchronize(self, time):
        eff = time - self.delay          # time elapsed since the delayed start
        if eff < 0.0:
            # Still inside the input delay window: hold everything released.
            self.SetThrottle(0.0)
            self.SetSteering(0.0)
            self.SetBraking(0.0)
            return

        # Throttle gradually increases to 0.7 over THROTTLE_RAMP after the delay.
        if eff < THROTTLE_RAMP:
            throttle = THROTTLE_TARGET * (eff / THROTTLE_RAMP)
        else:
            throttle = THROTTLE_TARGET
        self.SetThrottle(throttle)
        self.SetBraking(0.0)

        # Sinusoidal steering, active only once simulation time passes STEER_START.
        if time >= STEER_START:
            self.SetSteering(STEER_AMPLITUDE * math.sin(STEER_FREQ * (time - STEER_START)))
        else:
            self.SetSteering(0.0)


def main():
    # === Data paths === locate bundled Chrono + vehicle assets
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())
    veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

    # === Vehicle === HMMWV full model on rigid terrain (NSC)
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)   # NSC for rigid terrain
    hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
    hmmwv.SetChassisFixed(False)                         # MANDATORY — fixed chassis won't move
    hmmwv.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
    hmmwv.SetTireType(veh.TireModelType_TMEASY)          # rigid terrain: TMEASY tire
    hmmwv.SetTireStepSize(TIRE_STEP)
    hmmwv.Initialize()

    hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

    # === System & bodies (created by the veh.HMMWV_Full wrapper) ===
    system = hmmwv.GetSystem()                           # ChSystemNSC owned by the wrapper
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED, after Initialize
    chassis = hmmwv.GetChassisBody()                     # cache: main chassis body, reused below
    print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())
    # wheels/spindles: hmmwv.GetVehicle().GetAxles(); joints: suspension + steering inside wrapper

    # === Terrain === flat rigid patch under the vehicle
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    # === Footprint check === assert the wheels rest on (not through) the terrain
    TIRE_RADIUS = 0.464          # HMMWV tire radius (m) from the wheel geometry
    ZTOL = 0.1
    veh_obj = hmmwv.GetVehicle()
    spindle_world = []
    for axle in range(veh_obj.GetNumberAxles()):
        for side in (veh.LEFT, veh.RIGHT):
            spindle_world.append(veh_obj.GetSpindlePos(axle, side))
    wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
    assert wheel_bottom_z >= -ZTOL, (
        f"vehicle sinks into terrain: wheel bottom z={wheel_bottom_z:.3f} "
        f"vs terrain top z=0.0; raise INIT_LOC.z"
    )

    # === Driver === custom scripted driver replacing the default, initialized with the delay
    driver = MyDriver(veh_obj, DRIVER_DELAY)
    driver.Initialize()

    # === Visualization === full vehicle Irrlicht scene: window + sky + camera + lights
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle("HMMWV custom-driver simulation")
    vis.SetWindowSize(1280, 720)
    vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddLightDirectional()
    vis.AttachVehicle(veh_obj)

    # === Output / logging (review-only) ===

    # === Main loop === real-time stepping: driver -> terrain -> vehicle -> vis
    realtime_timer = chrono.ChRealtimeStepTimer()
    step_number = 0
    try:
        while vis.Run() and system.GetChTime() < SIM_END:
            time = system.GetChTime()

            if step_number % render_every == 0:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()

            driver_inputs = driver.GetInputs()


            driver.Synchronize(time)
            terrain.Synchronize(time)
            hmmwv.Synchronize(time, driver_inputs, terrain)
            vis.Synchronize(time, driver_inputs)

            driver.Advance(TIME_STEP)
            terrain.Advance(TIME_STEP)
            hmmwv.Advance(TIME_STEP)
            vis.Advance(TIME_STEP)

            step_number += 1
            realtime_timer.Spin(TIME_STEP)
    except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
        import traceback
        traceback.print_exc()
        raise

    # === Post-processing (review-only) ===


if __name__ == "__main__":
    main()
