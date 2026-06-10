"""HMMWV full vehicle driving straight over SCM deformable (Bekker-Wong) terrain.

System type: NSC vehicle wrapper system reconfigured to SMC-style soft-soil
contact (SCM deformable terrain). The main bodies are the HMMWV chassis, four
TMEASY-tire wheels/spindles, and the SCM deformable terrain grid. A TMEASY tire
model is used because the rigid default cannot drive on SCM. The driver applies
a constant zero steering input, so the vehicle is expected to travel forward in
a straight line while the wheels sink into and leave ruts in the deformable
soil.
"""

import math

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Named constants === geometry / physics / soil parameters (no bare literals downstream)
time_step = 2e-3                      # integration step (s)
sim_end = 8.0                         # simulation duration (s)
render_fps = 50.0                     # review render cadence (frames/s)

INIT_LOC = chrono.ChVector3d(-5.0, 0.0, 0.6)   # chassis spawn (front of terrain, lifted onto soil)
INIT_ROT = chrono.ChQuaterniond(1, 0, 0, 0)    # identity heading (+X forward)

TERRAIN_LENGTH = 40.0                 # SCM patch length, X (m)
TERRAIN_WIDTH = 20.0                  # SCM patch width, Y (m)
TERRAIN_RES = 0.1                     # SCM grid resolution (m)

THROTTLE = 0.7                        # constant forward throttle
STEERING = 0.0                        # constant steering — vehicle drives straight

TIRE_FAMILY = 1                       # collision family for tire cylinders
SUPPORT_FAMILY = 4                    # reserved support family

render_every = max(1, round(1.0 / (render_fps * time_step)))   # precomputed once: physics steps per frame

# === Vehicle === HMMWV full model on deformable soil (SMC contact, TMEASY tires)
chrono.SetChronoDataPath(chrono.GetChronoDataPath())           # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')       # locate vehicle data files

hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)             # SMC for SCM deformable terrain
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)                                   # MANDATORY — fixed chassis won't move
hmmwv.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
hmmwv.SetTireType(veh.TireModelType_TMEASY)                    # SCM requires a non-rigid tire
hmmwv.SetTireStepSize(time_step)
hmmwv.Initialize()

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
system = hmmwv.GetSystem()                                     # ChSystemSMC owned by the wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)   # REQUIRED before SCM terrain
chassis = hmmwv.GetChassisBody()      # cache: main chassis rigid body, reused every step
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())          # report total vehicle mass
# wheels/spindles: hmmwv.GetVehicle().GetAxles()[i].m_wheels[j].GetSpindle()
# joints: suspension + steering links created inside the wrapper

# === Terrain === SCM Bekker-Wong deformable soft-soil patch with a moving active patch
terrain = veh.SCMTerrain(system)
terrain.SetSoilParameters(
    2e6,    # Bekker_Kphi    — frictional modulus (Pa)
    0,      # Bekker_Kc      — cohesive modulus
    1.1,    # Bekker_n       — exponent
    0,      # Mohr_cohesion  — cohesive limit (Pa)
    30,     # Mohr_friction  — friction angle (deg)
    0.01,   # Janosi_shear   — shear coefficient (m)
    2e8,    # elastic_K      — elastic stiffness (Pa/m)
    3e4,    # damping_R      — vertical damping (Pa·s/m)
)
terrain.AddMovingPatch(
    chassis,                          # chassis body — stable level OOBB projection
    chrono.ChVector3d(0, 0, 0),       # local OOBB centre offset
    chrono.ChVector3d(5, 3, 1),       # OOBB dimensions (m)
)
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0.0, 0.1)     # sinkage heatmap overlay
terrain.Initialize(TERRAIN_LENGTH, TERRAIN_WIDTH, TERRAIN_RES)
terrain.SetMeshWireframe(False)
terrain.SetTexture(chrono.GetChronoDataFile("vehicle/terrain/textures/dirt.jpg"), 80, 80)

# === Tire collision cylinders === TMEASY tires need explicit spindle collision for SCM ray-cast
tire_rad = hmmwv.GetVehicle().GetAxles()[0].m_wheels[0].GetTire().GetRadius()   # cache: once
tire_w = hmmwv.GetVehicle().GetAxles()[0].m_wheels[0].GetTire().GetWidth()      # cache: once
tire_mat = chrono.ChContactMaterialSMC()
tire_mat.SetFriction(0.9)
tire_mat.SetRestitution(0.1)
tire_mat.SetYoungModulus(2e7)
for axle in hmmwv.GetVehicle().GetAxles():
    for iw in range(2):
        spindle = axle.m_wheels[iw].GetSpindle()
        spindle.AddCollisionShape(
            chrono.ChCollisionShapeCylinder(tire_mat, tire_rad + 0.04, tire_w),
            chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(math.pi / 2)),
        )
        spindle.EnableCollision(True)
        sp_cm = spindle.GetCollisionModel()
        sp_cm.SetFamily(TIRE_FAMILY)
        sp_cm.DisallowCollisionsWith(TIRE_FAMILY)
        sp_cm.DisallowCollisionsWith(SUPPORT_FAMILY)
system.GetCollisionSystem().BindAll()                          # rebuild collision models after edits

# === Footprint assert === wheels must rest on (not through) the SCM rest plane
veh_obj = hmmwv.GetVehicle()                                   # cache: vehicle handle
wheel_bottom_z = min(
    veh_obj.GetSpindlePos(a, s).z
    for a in range(veh_obj.GetNumberAxles())
    for s in (veh.LEFT, veh.RIGHT)
) - tire_rad
assert wheel_bottom_z >= -0.2, (
    f"vehicle sinks too far into SCM: wheel bottom z={wheel_bottom_z:.3f}; "
    f"raise INIT_LOC.z"
)

# === Driver === scripted constant-throttle, constant-zero-steering (straight drive)
driver = veh.ChDataDriver(veh_obj, veh.vector_Entry([
    veh.DataDriverEntry(0.0, STEERING, 0.0, 0.0),
    veh.DataDriverEntry(0.5, STEERING, THROTTLE, 0.0),
    veh.DataDriverEntry(sim_end, STEERING, THROTTLE, 0.0),
]))
driver.Initialize()

# === Visualization === full Irrlicht vehicle scene: window + sky + chase camera + light
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV on SCM deformable terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()                                      # vehicle truths use a directional light
vis.AttachVehicle(hmmwv.GetVehicle())

# === Main loop === advance driver/terrain/vehicle/vis in lockstep; vehicle drives straight

try:

    realtime_timer = chrono.ChRealtimeStepTimer()
    step_number = 0
    while vis.Run() and system.GetChTime() < sim_end:
        time = system.GetChTime()
        if step_number % render_every == 0:          # throttled rendering
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()

        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)


        driver.Advance(time_step)
        terrain.Advance(time_step)
        hmmwv.Advance(time_step)
        vis.Advance(time_step)

        step_number += 1
        realtime_timer.Spin(time_step)
except (RuntimeError, ValueError) as exc:          # solver divergence / invalid state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass

# === Post-processing === assemble review video + timeseries plot (review-only)
