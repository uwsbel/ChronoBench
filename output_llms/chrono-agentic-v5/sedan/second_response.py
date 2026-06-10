"""Two BMW_E90 sedans on a flat rigid terrain, each driven by its own scripted
driver applying a sinusoidal steering input while holding a constant throttle.

System type: NSC (rigid-terrain catalog sedans, ChContactMethod_NSC).
Main bodies: two wheeled sedan vehicles (chassis + suspensions + wheels + tires)
sharing one ChSystemNSC, and a single flat RigidTerrain patch textured with
concrete.jpg.  Each vehicle is steered by a custom ChDriver subclass whose
Synchronize() sets steering = A*sin(omega*t) and a fixed forward throttle.

Expected behavior: both sedans accelerate forward and weave left/right following
their sinusoidal steering laws, staying on the terrain for the whole run.
"""

import math

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Parameters === geometry / physics constants (no bare literals downstream)
step_size = 2e-3                 # integration step (s)
tire_step_size = 1e-3            # tire model sub-step (s)
sim_end = 12.0                   # bounded run length (s)
render_fps = 50.0                # review render cadence (frames/s)

# Sinusoidal steering law:  steering(t) = STEER_AMP * sin(STEER_OMEGA * t + phase)
# Each sedan uses its own amplitude / frequency / throttle so the two weave
# distinctly (the second is not glued to the chase camera tracking the first).
STEER_AMP_1 = 0.35               # sedan 1 peak steering (-1..+1)
STEER_OMEGA_1 = 0.8             # sedan 1 angular frequency (rad/s)
THROTTLE_1 = 0.45              # sedan 1 constant forward throttle (0..+1)

STEER_AMP_2 = 0.50               # sedan 2 peak steering (-1..+1)
STEER_OMEGA_2 = 1.4             # sedan 2 angular frequency (rad/s)
THROTTLE_2 = 0.30             # sedan 2 constant forward throttle (0..+1)
STEER_PHASE_2 = math.pi         # second sedan steers in anti-phase

# Spawn poses: sedan 1 leads, sedan 2 trails well to the side, both facing +X.
SEDAN_INIT_Z = 0.5              # chassis-origin rest height above flat terrain (m)
SEDAN1_INIT = chrono.ChVector3d(0.0, 0.0, SEDAN_INIT_Z)
SEDAN2_INIT = chrono.ChVector3d(-8.0, -6.0, SEDAN_INIT_Z)
INIT_ROT = chrono.ChQuaterniond(1, 0, 0, 0)   # identity -> facing +X

TERRAIN_LENGTH = 200.0          # terrain patch X extent (m)
TERRAIN_WIDTH = 200.0           # terrain patch Y extent (m)
TIRE_RADIUS = 0.33              # sedan tire radius (m), for footprint assert
ZTOL = 0.10                     # allowed wheel-bottom clearance vs terrain top

vis_type = veh.VisualizationType_MESH


# === Scripted drivers === each sedan gets a sinusoidal-steering ChDriver
class SinusoidalDriver(veh.ChDriver):
    """Open-loop driver: constant throttle, steering = amp * sin(omega*t + phase)."""

    def __init__(self, vehicle, amp, omega, phase, throttle):
        super().__init__(vehicle)
        self.amp = amp
        self.omega = omega
        self.phase = phase
        self.throttle = throttle

    def Synchronize(self, time):
        self.SetSteering(self.amp * math.sin(self.omega * time + self.phase))
        self.SetThrottle(self.throttle)
        self.SetBraking(0.0)


# === Data paths === locate bundled Chrono + vehicle assets (truth components)
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Vehicles === two BMW_E90 sedans sharing ONE system (second on shared sys)
sedan1 = veh.BMW_E90()
sedan1.SetContactMethod(chrono.ChContactMethod_NSC)
sedan1.SetChassisCollisionType(veh.CollisionType_NONE)
sedan1.SetChassisFixed(False)                       # MANDATORY — fixed chassis won't move
sedan1.SetInitPosition(chrono.ChCoordsysd(SEDAN1_INIT, INIT_ROT))
sedan1.SetTireType(veh.TireModelType_TMEASY)        # rigid-terrain compatible tire
sedan1.SetTireStepSize(tire_step_size)
sedan1.Initialize()

system = sedan1.GetSystem()                          # ChSystemNSC owned by sedan1 wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED, after Initialize

sedan2 = veh.BMW_E90(system)                         # SHARE sedan1's system (no orphan world)
sedan2.SetChassisCollisionType(veh.CollisionType_NONE)
sedan2.SetChassisFixed(False)
sedan2.SetInitPosition(chrono.ChCoordsysd(SEDAN2_INIT, INIT_ROT))
sedan2.SetTireType(veh.TireModelType_TMEASY)
sedan2.SetTireStepSize(tire_step_size)
sedan2.Initialize()

print("VEHICLE MASS: ", sedan1.GetVehicle().GetMass())
print("VEHICLE MASS: ", sedan2.GetVehicle().GetMass())

for sedan in (sedan1, sedan2):
    sedan.SetChassisVisualizationType(vis_type)
    sedan.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    sedan.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    sedan.SetWheelVisualizationType(vis_type)
    sedan.SetTireVisualizationType(vis_type)

# Footprint sanity: wheel bottoms must rest on (not through) the flat terrain.
for sedan in (sedan1, sedan2):
    veh_obj = sedan.GetVehicle()
    spindle_z = [veh_obj.GetSpindlePos(a, s).z
                 for a in range(veh_obj.GetNumberAxles())
                 for s in (veh.LEFT, veh.RIGHT)]
    wheel_bottom_z = min(spindle_z) - TIRE_RADIUS
    assert wheel_bottom_z >= -ZTOL, (
        f"sedan sinks into terrain: wheel bottom z={wheel_bottom_z:.3f} "
        f"vs terrain top z=0.0; raise SEDAN_INIT_Z by {-wheel_bottom_z:.3f} m"
    )

# === Terrain === single flat rigid patch textured with concrete.jpg
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
patch.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.8))
terrain.Initialize()

# === Visualization === full Irrlicht scene: window + sky + camera + lights
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Two Sedans — Sinusoidal Steering")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 10.0, 0.6)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()                            # vehicle truths use a directional light
vis.AttachVehicle(sedan1.GetVehicle())

# === Drivers === one sinusoidal-steering driver per sedan (anti-phase weave)
driver1 = SinusoidalDriver(sedan1.GetVehicle(), STEER_AMP_1, STEER_OMEGA_1, 0.0, THROTTLE_1)
driver1.Initialize()
driver2 = SinusoidalDriver(sedan2.GetVehicle(), STEER_AMP_2, STEER_OMEGA_2, STEER_PHASE_2, THROTTLE_2)
driver2.Initialize()

# === Main loop === advance both vehicles + both drivers + terrain + vis each step
render_steps = math.ceil((1.0 / render_fps) / step_size)   # precomputed once
realtime_timer = chrono.ChRealtimeStepTimer()

try:
    chassis1 = sedan1.GetChassisBody()    # cache: fetched once, reused every step
    chassis2 = sedan2.GetChassisBody()    # cache: fetched once, reused every step

    step_number = 0
    frame = 0
    while vis.Run() and system.GetChTime() < sim_end:
        time = system.GetChTime()

        if step_number % render_steps == 0:          # throttled rendering
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        inputs1 = driver1.GetInputs()
        inputs2 = driver2.GetInputs()


        driver1.Synchronize(time)
        driver2.Synchronize(time)
        terrain.Synchronize(time)
        sedan1.Synchronize(time, inputs1, terrain)
        sedan2.Synchronize(time, inputs2, terrain)
        vis.Synchronize(time, inputs1)

        driver1.Advance(step_size)
        driver2.Advance(step_size)
        terrain.Advance(step_size)
        sedan1.Advance(step_size)
        sedan2.Advance(step_size)
        vis.Advance(step_size)

        step_number += 1
        realtime_timer.Spin(step_size)
except (RuntimeError, ValueError) as exc:   # solver divergence / bad simulation state
    import traceback; traceback.print_exc()
    raise
finally:
    pass

# === Post-processing === assemble review video + plot, then drop frame dirs
