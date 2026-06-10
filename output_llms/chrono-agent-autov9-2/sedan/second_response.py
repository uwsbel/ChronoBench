"""Two Sedan passenger cars driving across a flat rigid-terrain road.

Model
-----
- System type: NSC contact (the Sedan catalog wrapper builds a ChSystemNSC).
- Bodies: two `veh.Sedan` wheeled vehicles (chassis + four wheels/tires each)
  sharing ONE physical system, plus a flat `veh.RigidTerrain` patch textured
  with concrete.
- Each vehicle is steered by its own scripted `veh.ChDriver` subclass that
  applies a constant forward throttle and a sinusoidal steering signal, so both
  cars accelerate and weave side-to-side as they cross the road.

Expected behavior
-----------------
Both cars start side by side at one end of the road and drive forward together
under constant throttle while their front wheels oscillate left/right from the
sinusoidal steering input. A static elevated camera frames the whole road so the
forward motion of both vehicles is clearly visible.
"""

# === Imports ===
import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Named constants === geometry / physics / control (no bare literals downstream)
TIME_STEP = 2e-3                       # integration step (s)
TIRE_STEP = 1e-3                       # tire model sub-step (s)
SIM_END = 12.0                         # simulated duration (s)
RENDER_FPS = 50.0                      # review-video frame rate

TERRAIN_LENGTH = 120.0                 # road extent along X (m)
TERRAIN_WIDTH = 60.0                   # road extent along Y (m)
TERRAIN_FRICTION = 0.9                 # tire/road friction coefficient
TERRAIN_RESTITUTION = 0.01             # near-inelastic road contact

INIT_Z = 0.5                           # chassis-origin spawn height above road (m)
LANE_OFFSET = 3.0                      # half the lateral gap between the two cars (m)
INIT_X = -50.0                         # both cars start near the -X end of the road

THROTTLE = 0.4                         # constant forward throttle for both cars
STEER_AMP = 0.25                       # sinusoidal steering amplitude (-1..1)
STEER_FREQ = 0.4                       # sinusoidal steering frequency (Hz)

# Derived spawn poses for the two side-by-side cars (computed once).
INIT_POS_1 = chrono.ChVector3d(INIT_X, +LANE_OFFSET, INIT_Z)
INIT_POS_2 = chrono.ChVector3d(INIT_X, -LANE_OFFSET, INIT_Z)
INIT_ROT = chrono.QuatFromAngleZ(0.0)  # facing +X (drive direction)

RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once
TWO_PI_FREQ = 2.0 * math.pi * STEER_FREQ                      # precomputed once


# === Driver === scripted constant-throttle + sinusoidal-steering control law
class SinusoidDriver(veh.ChDriver):
    """Applies constant throttle and a phase-shifted sinusoidal steering input."""

    def __init__(self, vehicle, phase):
        super().__init__(vehicle)
        self.phase = phase                 # per-vehicle steering phase offset (rad)

    def Synchronize(self, time):
        self.SetThrottle(THROTTLE)
        self.SetBraking(0.0)
        self.SetSteering(STEER_AMP * math.sin(TWO_PI_FREQ * time + self.phase))


def main():
    # === First vehicle (creates and owns the shared ChSystemNSC) ===
    car1 = veh.Sedan()
    car1.SetContactMethod(chrono.ChContactMethod_NSC)
    car1.SetChassisCollisionType(veh.CollisionType_NONE)
    car1.SetChassisFixed(False)
    car1.SetInitPosition(chrono.ChCoordsysd(INIT_POS_1, INIT_ROT))
    car1.SetTireType(veh.TireModelType_TMEASY)
    car1.SetTireStepSize(TIRE_STEP)
    car1.Initialize()
    car1.SetChassisVisualizationType(chrono.VisualizationType_MESH)
    car1.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
    car1.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
    car1.SetWheelVisualizationType(chrono.VisualizationType_MESH)
    car1.SetTireVisualizationType(chrono.VisualizationType_MESH)

    # === System & bodies (created by the veh.Sedan wrapper) ===
    system = car1.GetSystem()                 # ChSystemNSC owned by the first wrapper
    chassis1 = car1.GetChassisBody()          # main chassis rigid body, car 1
    # wheels/spindles + suspension & steering joints are created inside the wrapper.

    # === Collision system === Bullet narrowphase for tire/terrain contact
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    # === Second vehicle (shares the SAME system via the system constructor) ===
    car2 = veh.Sedan(system)
    car2.SetChassisCollisionType(veh.CollisionType_NONE)
    car2.SetChassisFixed(False)
    car2.SetInitPosition(chrono.ChCoordsysd(INIT_POS_2, INIT_ROT))
    car2.SetTireType(veh.TireModelType_TMEASY)
    car2.SetTireStepSize(TIRE_STEP)
    car2.Initialize()
    car2.SetChassisVisualizationType(chrono.VisualizationType_MESH)
    car2.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
    car2.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
    car2.SetWheelVisualizationType(chrono.VisualizationType_MESH)
    car2.SetTireVisualizationType(chrono.VisualizationType_MESH)
    chassis2 = car2.GetChassisBody()          # main chassis rigid body, car 2

    # === Terrain === flat rigid road patch with concrete texture
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(TERRAIN_FRICTION)
    patch_mat.SetRestitution(TERRAIN_RESTITUTION)
    patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
    patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/concrete.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.8))
    terrain.Initialize()

    # Confirm both cars rest on (not through) the road before driving.
    veh_obj1 = car1.GetVehicle()
    veh_obj2 = car2.GetVehicle()
    for veh_obj in (veh_obj1, veh_obj2):
        tire_radius = veh_obj.GetAxle(0).GetWheel(0, veh.LEFT).GetTire().GetRadius()
        wheel_bottom_z = veh_obj.GetSpindlePos(0, veh.LEFT).z - tire_radius
        assert wheel_bottom_z >= -0.1, (
            f"vehicle sinks into road: wheel bottom z={wheel_bottom_z:.3f}; "
            f"raise INIT_Z by {-wheel_bottom_z:.3f} m"
        )

    # === Drivers === one scripted sinusoidal driver per vehicle (phase-shifted)
    driver1 = SinusoidDriver(veh_obj1, phase=0.0)
    driver2 = SinusoidDriver(veh_obj2, phase=math.pi)   # opposite-phase weave
    driver1.Initialize()
    driver2.Initialize()

    # === Visualization === generic Irrlicht (two vehicles) with a static elevated camera
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("Two Sedans driving across a concrete road")
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(INIT_X - 10.0, -35.0, 18.0),
                  chrono.ChVector3d(INIT_X + 25.0, 0.0, 0.0))  # static elevated view
    vis.AddTypicalLights()
    vis.AddGrid(2.0, 2.0, 40, 40,
                chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.01), chrono.QUNIT),
                chrono.ChColor(0.4, 0.4, 0.4))


    # === Main loop === render once per frame; Synchronize/Advance both vehicles + drivers
    frame = 0
    try:
        while vis.Run() and system.GetChTime() < SIM_END:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            for _ in range(RENDER_EVERY):
                time = system.GetChTime()
                inputs1 = driver1.GetInputs()
                inputs2 = driver2.GetInputs()

                driver1.Synchronize(time)
                driver2.Synchronize(time)
                terrain.Synchronize(time)
                car1.Synchronize(time, inputs1, terrain)
                car2.Synchronize(time, inputs2, terrain)

                driver1.Advance(TIME_STEP)
                driver2.Advance(TIME_STEP)
                terrain.Advance(TIME_STEP)
                car1.Advance(TIME_STEP)        # steps the shared system once for this dt
                car2.Advance(TIME_STEP)        # advances car-2 subsystems (system already stepped)
                if system.GetChTime() >= SIM_END:
                    break
    except (RuntimeError, ValueError) as exc:   # solver divergence / invalid state
        import traceback
        traceback.print_exc()
        raise
    finally:
        pass


if __name__ == "__main__":
    main()
