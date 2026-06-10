"""
HMMWV on SCM Deformable Terrain with Heightmap (scm_hill)

Models an HMMWV full vehicle driving over an SCM Bekker-Wong soft-soil terrain
initialized from a bump heightmap (bump64.bmp). The SCM terrain deforms under
the vehicle's weight, producing realistic sinkage and ruts.

System: ChSystemNSC (wrapper-owned by HMMWV_Full)
Contact method: SMC (required for SCM deformable terrain)
Main bodies: HMMWV chassis, 4 spindles, SCM terrain
Expected behavior: vehicle drives forward over bumps, terrain deforms under tires.
"""

import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


# === Constants ===
step_size = 2e-3          # simulation time step (s)
sim_end = 20.0            # simulation end time (s)
render_fps = 50.0         # Irrlicht render rate (Hz)
render_every = max(1, round(1.0 / (render_fps * step_size)))  # precomputed once

# terrain
SCM_SIZE_X = 40.0         # SCM patch length (m)
SCM_SIZE_Y = 40.0         # SCM patch width (m)
SCM_RESOLUTION = 0.02     # grid cell size (m)

# vehicle initial placement — spawn above terrain rest plane
SUSPENSION_REF_HEIGHT = 0.5  # chassis origin above wheel-bottom at rest (HMMWV)
INIT_X = -15.0
INIT_Y = 0.0
INIT_Z = SUSPENSION_REF_HEIGHT  # terrain rest plane is z=0 via heightmap

# === Data Paths ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Vehicle Setup ===
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)      # SMC required for SCM terrain
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)                            # MANDATORY — fixed chassis won't move
hmmwv.SetInitPosition(chrono.ChCoordsysd(
    chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z),
    chrono.QUNIT
))
hmmwv.SetTireType(veh.TireModelType_TMEASY)             # TMEASY required on SCM (RIGID won't drive)
hmmwv.SetTireStepSize(step_size)
hmmwv.Initialize()

# === System & Bodies (created by veh.HMMWV_Full wrapper) ===
system = hmmwv.GetSystem()                              # ChSystemSMC owned by the wrapper
chassis = hmmwv.GetChassisBody()                        # cache: main chassis rigid body; reused below
# Wheels/spindles: hmmwv.GetVehicle().GetAxle(i)...
# Joints: suspension + steering links created inside the wrapper

system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED before SCMTerrain

print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())

# Visualization types (after Initialize)
hmmwv.SetChassisVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(chrono.VisualizationType_MESH)

# === SCM Terrain ===
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
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0.0, 0.1)  # sinkage heatmap

# Moving patch on chassis body (NOT spindle — spindles rotate and break projection)
terrain.AddMovingPatch(
    chassis,
    chrono.ChVector3d(0, 0, 0),     # local OOBB centre offset
    chrono.ChVector3d(5, 3, 1),     # OOBB dimensions (m)
)

# Initialize from heightmap (bump64.bmp) — scm_hill truth signature
terrain.Initialize(
    veh.GetDataFile("terrain/height_maps/bump64.bmp"),
    SCM_SIZE_X, SCM_SIZE_Y, -1, 1, SCM_RESOLUTION
)
terrain.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 6.0, 6.0)
terrain.SetMeshWireframe(False)

# === Tire Collision Cylinders (TMEASY tires don't add collision geometry automatically) ===
tire_rad = hmmwv.GetVehicle().GetAxles()[0].m_wheels[0].GetTire().GetRadius()  # cache: read once
tire_w   = hmmwv.GetVehicle().GetAxles()[0].m_wheels[0].GetTire().GetWidth()   # cache: read once
tire_mat = chrono.ChContactMaterialSMC()
tire_mat.SetFriction(0.9)
tire_mat.SetRestitution(0.1)
tire_mat.SetYoungModulus(2e7)

TIRE_FAMILY    = 1
SUPPORT_FAMILY = 4

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

# Rebuild all collision models after post-init shape changes
system.GetCollisionSystem().BindAll()

# === Driver ===
# Irrlicht visualization must be built before driver
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV on SCM Deformable Hill Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()   # vehicle truths use directional light, not AddTypicalLights()
vis.AttachVehicle(hmmwv.GetVehicle())

driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
render_step_size = 1.0 / render_fps   # precomputed once
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()


# === Main Loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
try:
    while vis.Run() and system.GetChTime() < sim_end:
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


        driver.Advance(step_size)
        terrain.Advance(step_size)
        hmmwv.Advance(step_size)
        vis.Advance(step_size)

        step_number += 1
        realtime_timer.Spin(step_size)

except (RuntimeError, ValueError) as exc:  # solver divergence / bad vehicle state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
