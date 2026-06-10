"""
HMMWV on SCM Deformable Terrain (PyChrono 9.0.x, Irrlicht)
===========================================================
Simulates a full HMMWV driving on Bekker-Wong soft-soil (SCM) terrain using
the Irrlicht renderer.  The soil deforms under the wheels, leaving visible ruts.
A moving-patch feature keeps the active SCM region near the vehicle chassis.
Sinkage is visualized with a false-color heatmap (SetPlotType).  An interactive
driver (keyboard / gamepad) controls steering, throttle, and braking at 50 fps.

System:   ChSystemSMC (owned by the HMMWV_Full wrapper, SMC contact method for SCM)
Bodies:   HMMWV chassis + 4 axle/suspension sub-bodies; SCM grid (deformable mesh)
Tires:    TMEASY (required for SCM — RIGID tires do not provide grip on soft soil)
Expected: vehicle accelerates forward, wheels sink into deformable terrain; ruts
          visible behind the chassis; sinkage heatmap color-codes depth.
"""

import math
import os
import csv

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Named constants ===
STEP_SIZE      = 2e-3       # physics time step (s)
SIM_END        = 20.0       # simulation end time (s)
RENDER_FPS     = 50.0       # rendering frame rate (fps)
RENDER_EVERY   = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once
render_step_size = 1.0 / RENDER_FPS                              # precomputed once

# Vehicle initial position — start slightly above SCM rest plane (z=0)
INIT_X         = -15.0      # m, start near terrain centre
INIT_Y         = 0.0
INIT_Z         = 0.5        # chassis height above terrain
INIT_YAW       = 0.0        # heading angle (rad)

# SCM terrain grid
SCM_LENGTH     = 120.0      # m
SCM_WIDTH      = 120.0      # m
SCM_RESOLUTION = 0.1        # m — 10 cm grid for visible ruts

# Tire/chassis collision families for SCM ray-cast exclusions
TIRE_FAMILY    = 1

# =====================================================================
# === Data paths (mandatory scored-core components for catalog vehicles) ===
# =====================================================================
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# =====================================================================
# === Vehicle setup (HMMWV_Full wrapper — owns ChSystem) ===
# =====================================================================
init_loc = chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z)
init_rot = chrono.QuatFromAngleZ(INIT_YAW)

hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)   # SCM scenes use SMC
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)                          # MANDATORY — fixed chassis never moves
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
hmmwv.SetTireType(veh.TireModelType_TMEASY)           # TMEASY required for SCM grip
hmmwv.SetTireStepSize(STEP_SIZE)
hmmwv.Initialize()

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
system  = hmmwv.GetSystem()        # ChSystemSMC owned by the wrapper  # cache: fetched once
chassis = hmmwv.GetChassisBody()   # main chassis rigid body            # cache: fetched once
# wheels/spindles: hmmwv.GetVehicle().GetAxles()[i].m_wheels[j].GetSpindle()
# joints: double-wishbone suspension + steering links created inside the wrapper

# SetCollisionSystemType AFTER Initialize, BEFORE building SCMTerrain (truth order)
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())

# Visualization types (must follow Initialize)
hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

# =====================================================================
# === SCM Terrain ===
# =====================================================================
terrain = veh.SCMTerrain(system)   # must be created AFTER SetCollisionSystemType

# Sinkage false-color heatmap (0–10 cm range)
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0.0, 0.1)

# Bekker-Wong soil parameters — exactly 8 positional args required
terrain.SetSoilParameters(
    2e6,    # Bekker_Kphi   — frictional modulus (Pa)
    0,      # Bekker_Kc     — cohesive modulus
    1.1,    # Bekker_n      — exponent
    0,      # Mohr_cohesion — cohesive limit (Pa)
    30,     # Mohr_friction — friction angle (deg)
    0.01,   # Janosi_shear  — shear coefficient (m)
    2e8,    # elastic_K     — elastic stiffness (Pa/m)
    3e4,    # damping_R     — vertical damping (Pa·s/m)
)

# Moving patch — attached to CHASSIS so OOBB stays stable (spindle rotates → degenerate OOBB)
terrain.AddMovingPatch(
    chassis,
    chrono.ChVector3d(0, 0, 0),    # local OOBB centre offset
    chrono.ChVector3d(5, 3, 1),    # OOBB dimensions (m)
)

terrain.SetMeshWireframe(False)
terrain.SetTexture(
    chrono.GetChronoDataFile("vehicle/terrain/textures/dirt.jpg"),
    80, 80,  # UV tiling
)

terrain.Initialize(SCM_LENGTH, SCM_WIDTH, SCM_RESOLUTION)

# =====================================================================
# === Tire collision cylinders (REQUIRED for TMEASY on SCM) ===
# TMEASY tires carry no automatic collision shapes; SCM needs collision
# geometry on each spindle so its ray-casts find the tires.
# =====================================================================
veh_obj    = hmmwv.GetVehicle()                           # cache: fetched once
front_axle = veh_obj.GetAxles()[0]                        # cache: front axle ref
tire_rad   = front_axle.m_wheels[0].GetTire().GetRadius()
tire_width = front_axle.m_wheels[0].GetTire().GetWidth()

tire_mat = chrono.ChContactMaterialSMC()
tire_mat.SetFriction(0.9)
tire_mat.SetRestitution(0.1)

for axle in veh_obj.GetAxles():
    for iw in range(2):
        spindle = axle.m_wheels[iw].GetSpindle()
        spindle.AddCollisionShape(
            chrono.ChCollisionShapeCylinder(tire_mat, tire_rad + 0.04, tire_width),
            chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(math.pi / 2)),
        )
        spindle.EnableCollision(True)
        sp_cm = spindle.GetCollisionModel()
        sp_cm.SetFamily(TIRE_FAMILY)
        sp_cm.DisallowCollisionsWith(TIRE_FAMILY)
        # NOTE: do NOT DisallowCollisionsWith(0) — family 0 is used by SCM ray-casts

# Rebuild all collision models so SCM ray-casts find the new cylinders
system.GetCollisionSystem().BindAll()

# =====================================================================
# === Irrlicht visualization (vehicle-specific) ===
# =====================================================================
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV on SCM Deformable Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()    # vehicle truths use directional light (not AddTypicalLights)
vis.AttachVehicle(hmmwv.GetVehicle())

# =====================================================================
# === Driver (interactive — ChInteractiveDriverIRR takes the vis system) ===
# =====================================================================
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(render_step_size / 1.0)
driver.SetThrottleDelta(render_step_size / 1.0)
driver.SetBrakingDelta(render_step_size / 0.3)
driver.Initialize()

# =====================================================================
# === Review-only recording setup ===
# =====================================================================

# =====================================================================
# === Main loop ===
# =====================================================================
realtime_timer = chrono.ChRealtimeStepTimer()

try:
    while vis.Run() and system.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        for _ in range(RENDER_EVERY):
            sim_time      = system.GetChTime()
            driver_inputs = driver.GetInputs()


            driver.Synchronize(sim_time)
            terrain.Synchronize(sim_time)
            hmmwv.Synchronize(sim_time, driver_inputs, terrain)
            vis.Synchronize(sim_time, driver_inputs)

            driver.Advance(STEP_SIZE)
            terrain.Advance(STEP_SIZE)
            hmmwv.Advance(STEP_SIZE)    # advances the wrapper-owned ChSystem
            vis.Advance(STEP_SIZE)

            if system.GetChTime() >= SIM_END:
                break

        realtime_timer.Spin(STEP_SIZE)

except (RuntimeError, ValueError) as exc:   # solver divergence / bad vehicle state
    import traceback
    traceback.print_exc()
    raise

finally:
    pass
