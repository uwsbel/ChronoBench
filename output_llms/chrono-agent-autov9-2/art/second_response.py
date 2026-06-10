"""ARTcar wheeled-vehicle simulation on flat rigid terrain (PyChrono 9.0.1, Irrlicht).

Model: the small electric ARTcar catalog vehicle (NSC contact) driving on a wide
flat RigidTerrain patch. The chassis carries a MESH collision representation; the
running gear is rendered with PRIMITIVES; the wheels use the FIALA tire force
model. A scripted ChDriver applies a short brake-then-accelerate-with-gentle-steer
schedule so the car drives forward across the patch.

System type: ChSystemNSC (owned by the veh.ARTcar wrapper).
Main bodies: chassis + four wheel spindles + tires (created by the wrapper),
plus the rigid terrain patch body.
Expected behavior: the car rests on the terrain at spawn, then accelerates
forward and translates several metres while steering gently — no sinking,
no NaN.
"""

import os
import math

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr


# === Constants === geometry / physics / timing (no bare literals downstream)
TIME_STEP = 1e-3                       # integration step (s)
TIRE_STEP = 1e-3                       # FIALA tire force-model step (s)
SIM_END = 10.0                         # simulation duration (s)
RENDER_FPS = 50.0                      # review render cadence (frames/s)

INIT_LOC = chrono.ChVector3d(1.0, 0.0, 0.5)   # vehicle spawn (world frame)
INIT_ROT = chrono.QUNIT                        # spawn orientation (facing +X)

TERRAIN_LENGTH = 200.0                 # X size of the rigid patch (m) — wide patch
TERRAIN_WIDTH = 200.0                  # Y size of the rigid patch (m)
TERRAIN_TOP_Z = 0.0                    # top surface height of the flat patch (m)
TIRE_RADIUS = 0.10                     # ARTcar tire radius (m), for footprint assert
ZTOL = 0.10                            # allowed wheel-bottom clearance vs support top

RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # precomputed once


class ScriptedDriver(veh.ChDriver):
    """Open-loop time-based control: brake briefly, then accelerate with a gentle turn."""

    def __init__(self, vehicle):
        super().__init__(vehicle)

    def Synchronize(self, time):
        if time < 1.0:
            self.SetThrottle(0.0)
            self.SetBraking(1.0)
        else:
            self.SetThrottle(0.7)
            self.SetBraking(0.0)
        self.SetSteering(0.25 * math.sin(0.4 * time))   # gentle weave


def main():

    # === Vehicle === ARTcar wrapper owns its ChSystemNSC; configure then Initialize
    car = veh.ARTcar()
    car.SetContactMethod(chrono.ChContactMethod_NSC)
    car.SetChassisFixed(False)
    car.SetChassisCollisionType(veh.CollisionType_MESH)        # chassis collision: MESH
    car.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
    car.SetTireType(veh.TireModelType_FIALA)                   # tire model: FIALA
    car.SetTireStepSize(TIRE_STEP)
    car.Initialize()

    # Running gear rendered as PRIMITIVES (note: VisualizationType_* lives in chrono.*)
    car.SetChassisVisualizationType(chrono.VisualizationType_PRIMITIVES)
    car.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
    car.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
    car.SetWheelVisualizationType(chrono.VisualizationType_PRIMITIVES)
    car.SetTireVisualizationType(chrono.VisualizationType_PRIMITIVES)

    # === System & bodies (created by the veh.ARTcar wrapper) ===
    system = car.GetSystem()                  # ChSystemNSC owned by the wrapper
    chassis = car.GetChassisBody()            # cache: main chassis rigid body, reused every step
    veh_obj = car.GetVehicle()                # cache: vehicle subsystem handle, reused for state
    # wheels/spindles + suspension/steering joints are created inside the wrapper.

    # Collision system MUST be BULLET for the vehicle/terrain contact to resolve.
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    # === Terrain === wide flat rigid patch on the wrapper-owned system
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
    patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/tile4.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    # === Footprint assert === wheels must rest on (not through) the patch top
    spindle_world = []
    for axle in range(veh_obj.GetNumberAxles()):
        for side in (veh.LEFT, veh.RIGHT):
            spindle_world.append(veh_obj.GetSpindlePos(axle, side))
    wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
    assert wheel_bottom_z >= TERRAIN_TOP_Z - ZTOL, (
        f"vehicle sinks into terrain: wheel bottom z={wheel_bottom_z:.3f} "
        f"vs terrain top z={TERRAIN_TOP_Z:.3f}; raise INIT_LOC.z"
    )

    # === Driver === scripted open-loop control (no human-in-the-loop in batch runs)
    driver = ScriptedDriver(veh_obj)
    driver.Initialize()

    # === Visualization === vehicle-aware Irrlicht: window + chase cam + sky + lights + logo
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle("ARTcar on Rigid Terrain")
    vis.SetWindowSize(1280, 720)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 0.5), 4.0, 0.4)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddTypicalLights()
    vis.AttachVehicle(veh_obj)
    vis.AttachDriver(driver)


    # === Main loop === render once per frame; advance vehicle stack each physics step
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
                car.Synchronize(sim_time, driver_inputs, terrain)   # terrain arg: FIALA ground contact
                vis.Synchronize(sim_time, driver_inputs)
                driver.Advance(TIME_STEP)
                terrain.Advance(TIME_STEP)
                car.Advance(TIME_STEP)              # advances the wrapper-owned system
                vis.Advance(TIME_STEP)
                if system.GetChTime() >= SIM_END:
                    break
    except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
        import traceback
        traceback.print_exc()
        raise
    finally:
        pass


if __name__ == "__main__":
    main()
