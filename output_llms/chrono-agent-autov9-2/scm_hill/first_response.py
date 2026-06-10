"""HMMWV climbing a deformable soft-soil hill (SCM terrain) in PyChrono.

Models a full HMMWV wheeled vehicle (NSC system owned by the veh.HMMWV_Full
wrapper, SMC contact method) driving up a convex hill. The hill is a
veh.SCMTerrain Bekker-Wong deformable soil surface initialized from a heightmap
(convex bump), so the wheels sink slightly and the chassis climbs the slope.
TMEASY tires with per-spindle collision cylinders provide the slip/grip needed
to actually move on soft soil. A scripted ChDriver applies full throttle so the
vehicle accelerates from the hill toe toward the crest.

Expected behavior: the chassis starts near the foot of the hill, the wheels grip
the firm soil, and the vehicle drives forward and gains elevation (climbs the
hill) over the simulated interval.
"""

# === Imports ===
import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Named constants === geometry / physics / soil / driving parameters
TIME_STEP = 2e-3                       # integration step (s)
TIRE_STEP = 1e-3                       # tire force-model substep (s)
SIM_END = 8.0                          # simulated duration (s)
RENDER_FPS = 50.0                      # review-video frame rate

# Hill (heightmap) extent and elevation range — a climbable convex bump.
HILL_SIZE_X = 40.0                     # terrain length along X (m)
HILL_SIZE_Y = 40.0                     # terrain width  along Y (m)
HILL_H_MIN = 0.0                       # heightmap black -> this elevation (m)
HILL_H_MAX = 1.8                       # heightmap white -> crest elevation (m)
SCM_DELTA = 0.08                       # SCM grid resolution (m)
HEIGHTMAP = "terrain/height_maps/convex64.bmp"   # convex bump = a single hill

# Vehicle spawn: at the hill toe (negative X edge), facing +X toward the crest.
VEH_INIT_X = -18.0                     # toe of the hill in X (m)
VEH_INIT_Y = 0.0                       # centered in Y (m)
SUSPENSION_REF_HEIGHT = 0.5            # chassis origin above wheel-bottom at rest (HMMWV)
TIRE_RADIUS = 0.46                     # nominal HMMWV tire radius (m), used for footprint assert
ZTOL = 0.15                            # allowed wheel-bottom clearance/overlap vs soil top (m)

# Soil — firm soil so the hill stays climbable (Bekker / Mohr-Coulomb params).
SOIL_KPHI = 2e6                        # Bekker frictional modulus (Pa)
SOIL_KC = 0.0                          # Bekker cohesive modulus
SOIL_N = 1.1                           # Bekker exponent
SOIL_COHESION = 5e3                    # Mohr cohesion (Pa)
SOIL_FRICTION = 30.0                   # Mohr friction angle (deg)
SOIL_JANOSI = 0.01                     # Janosi shear coefficient (m)
SOIL_ELASTIC_K = 2e8                   # elastic stiffness (Pa/m)
SOIL_DAMPING_R = 3e4                   # vertical damping (Pa.s/m)

TIRE_FAMILY = 1                        # collision family for the tire cylinders

# Derived render cadence — precomputed once (never recompute inside the loop).
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # precomputed once

# === Pre-sample terrain height === throwaway SCMTerrain to find the soil top at spawn
# A heightmap SCMTerrain needs a collision system on its host ChSystem, so build a
# scratch system + terrain purely to query GetHeight at the spawn XY, then derive
# the chassis Z so the wheels start ON the hill toe (not buried / floating).
scratch_sys = chrono.ChSystemSMC()
scratch_sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
scratch_terrain = veh.SCMTerrain(scratch_sys)
scratch_terrain.SetSoilParameters(
    SOIL_KPHI, SOIL_KC, SOIL_N, SOIL_COHESION,
    SOIL_FRICTION, SOIL_JANOSI, SOIL_ELASTIC_K, SOIL_DAMPING_R,
)
scratch_terrain.Initialize(
    veh.GetVehicleDataFile(HEIGHTMAP),
    HILL_SIZE_X, HILL_SIZE_Y, HILL_H_MIN, HILL_H_MAX, SCM_DELTA,
)
toe_height = scratch_terrain.GetHeight(chrono.ChVector3d(VEH_INIT_X, VEH_INIT_Y, 0))
INIT_Z = toe_height + SUSPENSION_REF_HEIGHT      # chassis-origin Z so wheels rest on soil

# === Vehicle === HMMWV_Full wrapper owns its ChSystem (SMC); TMEASY tires for SCM
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(
    chrono.ChCoordsysd(chrono.ChVector3d(VEH_INIT_X, VEH_INIT_Y, INIT_Z), chrono.QUNIT)
)
hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
hmmwv.SetTireType(veh.TireModelType_TMEASY)        # SCM needs a slip/grip tire, not RIGID
hmmwv.SetTireStepSize(TIRE_STEP)                   # required alongside TMEASY on SCM
hmmwv.Initialize()
hmmwv.SetChassisVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(chrono.VisualizationType_MESH)

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
system = hmmwv.GetSystem()                 # ChSystemSMC owned by the wrapper
chassis = hmmwv.GetChassisBody()           # cache: main chassis rigid body, reused every step
vehicle_obj = hmmwv.GetVehicle()           # cache: ChWheeledVehicle handle, reused every step
# spindles: vehicle_obj.GetAxles()[i].m_wheels[s].GetSpindle() ; terrain: SCMTerrain below
# joints: suspension + steering links created inside the wrapper

# === Collision system === required for contact (vehicle wheels on SCM soil)
# Set Bullet explicitly on the wrapper-owned system after Initialize (per framework recipe).
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Terrain === SCM Bekker-Wong deformable hill from the convex heightmap
terrain = veh.SCMTerrain(system)
terrain.SetSoilParameters(
    SOIL_KPHI, SOIL_KC, SOIL_N, SOIL_COHESION,
    SOIL_FRICTION, SOIL_JANOSI, SOIL_ELASTIC_K, SOIL_DAMPING_R,
)
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0.0, 0.1)   # color sinkage; before Initialize
terrain.Initialize(
    veh.GetVehicleDataFile(HEIGHTMAP),
    HILL_SIZE_X, HILL_SIZE_Y, HILL_H_MIN, HILL_H_MAX, SCM_DELTA,
)
terrain.SetMeshWireframe(False)
terrain.SetTexture(chrono.GetChronoDataFile("vehicle/terrain/textures/dirt.jpg"), 40, 40)

# === Tire collision cylinders === TMEASY tires need explicit per-spindle geometry on SCM
tire_rad = vehicle_obj.GetAxles()[0].m_wheels[0].GetTire().GetRadius()   # cache: tire geom once
tire_w = vehicle_obj.GetAxles()[0].m_wheels[0].GetTire().GetWidth()      # cache: tire geom once
tire_mat = chrono.ChContactMaterialSMC()
tire_mat.SetFriction(0.9)
tire_mat.SetRestitution(0.1)
tire_mat.SetYoungModulus(2e7)
for axle in vehicle_obj.GetAxles():
    for iw in range(2):
        spindle = axle.m_wheels[iw].GetSpindle()
        spindle.AddCollisionShape(
            chrono.ChCollisionShapeCylinder(tire_mat, tire_rad + 0.04, tire_w),
            chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(math.pi / 2)),
        )
        spindle.EnableCollision(True)
        sp_cm = spindle.GetCollisionModel()
        sp_cm.SetFamily(TIRE_FAMILY)
        sp_cm.DisallowCollisionsWith(TIRE_FAMILY)   # wheels never collide each other
system.GetCollisionSystem().BindAll()               # rebuild models so ray-casts see cylinders

# === Footprint assert === confirm the wheels start ON the hill toe (not buried / floating)
spindle_world = []
for axle in range(vehicle_obj.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        spindle_world.append(vehicle_obj.GetSpindlePos(axle, side))
wheel_bottom_z = min(p.z for p in spindle_world) - tire_rad
assert abs(wheel_bottom_z - toe_height) <= max(ZTOL, TIRE_RADIUS), (
    f"wheels not on hill toe: wheel bottom z={wheel_bottom_z:.3f} vs soil top "
    f"z={toe_height:.3f}; adjust SUSPENSION_REF_HEIGHT (diff "
    f"{toe_height - wheel_bottom_z:.3f} m)"
)

# === Driver === scripted full-throttle controller so the vehicle climbs the hill
class HillClimbDriver(veh.ChDriver):
    """Open-loop driver: brief settle, then full throttle straight ahead up the hill."""

    def __init__(self, vehicle):
        super().__init__(vehicle)

    def Synchronize(self, time):
        if time < 0.5:
            self.SetThrottle(0.0)      # let the suspension settle on the soil first
            self.SetBraking(1.0)
        else:
            self.SetThrottle(1.0)      # full throttle to drive up the slope
            self.SetBraking(0.0)
        self.SetSteering(0.0)          # straight line toward the crest


driver = HillClimbDriver(vehicle_obj)
driver.Initialize()

# === Visualization === full vehicle-aware Irrlicht scene: window + sky + chase cam + lights
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV Climbing an SCM Hill")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.6)   # follow point, distance, height
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AddCamera(chrono.ChVector3d(VEH_INIT_X - 8, -8, INIT_Z + 4),
              chrono.ChVector3d(VEH_INIT_X, 0, INIT_Z))      # initial overview vantage
vis.AttachVehicle(vehicle_obj)
vis.AttachDriver(driver)

# === Output setup === guard against a missing output directory before opening writers


# === Main loop === render once per frame; Synchronize/Advance the vehicle stack per step
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
            hmmwv.Synchronize(sim_time, driver_inputs, terrain)
            vis.Synchronize(sim_time, driver_inputs)
            driver.Advance(TIME_STEP)
            terrain.Advance(TIME_STEP)
            hmmwv.Advance(TIME_STEP)         # internally steps the wrapper-owned system
            vis.Advance(TIME_STEP)
            if system.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:          # solver divergence / bad state mid-run
    import traceback
    traceback.print_exc()
    raise
finally:
    pass

# === Post-processing === assemble review video + timeseries plot, then clean frames
