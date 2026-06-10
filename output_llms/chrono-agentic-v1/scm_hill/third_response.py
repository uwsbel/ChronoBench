"""
HMMWV on Rigid Terrain with Height Map (scm_hill turn3)

Models a full HMMWV wheeled vehicle driving over a rigid terrain patch
initialized from a bump64 BMP height map. The contact method is NSC
(ChContactMethod_NSC) and a ChContactMaterialNSC is used for the terrain
patch. TMEASY tires are used for realistic force modeling. An interactive
Irrlicht-based driver is provided for real-time control. The vehicle drives
over hills and bumps in the terrain, demonstrating rigid terrain response.

System: NSC (ChSystemNSC owned by the HMMWV_Full wrapper)
Bodies: HMMWV chassis, suspension, spindles, wheels (created by wrapper)
Terrain: RigidTerrain with a single height-map patch (bump64.bmp)
Expected behavior: vehicle rolls over bumpy rigid terrain; no sinkage deformation.
"""

import os
import math
import csv

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# Anchor vehicle data path so GetDataFile returns absolute paths regardless of CWD
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# === Named constants ===
# Simulation parameters
TIME_STEP = 2e-3          # physics step size (s) — suitable for rigid terrain
SIM_END = 30.0            # total simulation time (s)
RENDER_FPS = 50.0         # render frame rate (Hz)

# Terrain parameters
TERRAIN_LENGTH = 60.0     # X extent of terrain patch (m)
TERRAIN_WIDTH = 60.0      # Y extent of terrain patch (m)
TERRAIN_HEIGHT_MIN = -1.0 # minimum height from bump map (m)
TERRAIN_HEIGHT_MAX = 1.0  # maximum height from bump map (m)

# Vehicle initial position — start slightly back and elevated for bump terrain
INIT_X = -20.0
INIT_Y = 0.0
INIT_Z = 0.5              # chassis origin height above terrain rest plane (m)

# Suspension reference height for HMMWV_Full (chassis origin to wheel bottom at rest)
SUSPENSION_REF_HEIGHT = 0.5   # m

# Precomputed render cadence
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# === Vehicle setup ===
# HMMWV_Full wrapper owns its own ChSystemNSC internally.
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)     # NSC: matches rigid-terrain truth
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)  # no chassis collision mesh
hmmwv.SetChassisFixed(False)                           # MANDATORY — fixed chassis won't move

init_loc = chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)           # identity quaternion
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
hmmwv.SetTireType(veh.TireModelType_TMEASY)            # TMEASY for realistic tire forces
hmmwv.SetTireStepSize(TIME_STEP)
hmmwv.Initialize()

# Set visualization types AFTER Initialize() — VisualizationType is in veh namespace (9.0.0)
hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
system = hmmwv.GetSystem()          # ChSystemNSC owned by the wrapper
chassis = hmmwv.GetChassisBody()    # cache: main chassis rigid body
# wheels/spindles accessible via hmmwv.GetVehicle().GetAxles()
# joints: suspension + steering links created inside the wrapper

# Set collision system type on the wrapper-owned system — REQUIRED for terrain contact
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Rigid Terrain with height map ===
# Use RigidTerrain with an NSC contact material and a BMP height-map patch.
# The bump64.bmp ship map provides hills/bumps over a 60×60 m area.
terrain = veh.RigidTerrain(system)

patch_mat = chrono.ChContactMaterialNSC()  # NSC material matches the contact method
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

# AddPatch heightmap overload: (mat, coordsys, bmp_file, length, width, hMin, hMax)
patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,                                       # centered at origin
    veh.GetDataFile("terrain/height_maps/bump64.bmp"),     # shipped height map
    TERRAIN_LENGTH,
    TERRAIN_WIDTH,
    TERRAIN_HEIGHT_MIN,
    TERRAIN_HEIGHT_MAX,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 6.0, 6.0)
patch.SetColor(chrono.ChColor(0.8, 0.7, 0.5))

terrain.Initialize()

# === Assert vehicle footprint after Initialize ===
# Verify wheel bottoms are above (or at) the terrain to detect spawn errors.
TIRE_RADIUS = hmmwv.GetVehicle().GetAxles()[0].m_wheels[0].GetTire().GetRadius()
TERRAIN_TOP_Z = 0.0  # terrain patch rests at z=0 (hMin/hMax centered on z=0 plane)
ZTOL = 0.15          # generous tolerance for bumpy terrain

spindle_world_z = []
for axle_idx in range(hmmwv.GetVehicle().GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        p = hmmwv.GetVehicle().GetSpindlePos(axle_idx, side)
        spindle_world_z.append(p.z)

wheel_bottom_z = min(spindle_world_z) - TIRE_RADIUS
assert wheel_bottom_z >= TERRAIN_TOP_Z - ZTOL, (
    f"Vehicle sinks into terrain: wheel bottom z={wheel_bottom_z:.3f} "
    f"vs terrain top z={TERRAIN_TOP_Z:.3f}; increase INIT_Z by "
    f"{TERRAIN_TOP_Z - wheel_bottom_z:.3f} m"
)

# === Visualization — ChWheeledVehicleVisualSystemIrrlicht ===
# Vehicle-specific Irrlicht visual system; Initialize() FIRST, then scene elements.
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV on Rigid Height-Map Terrain (NSC)")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AttachVehicle(hmmwv.GetVehicle())

# === Driver — interactive IRR driver (scored core) ===
# ChInteractiveDriverIRR takes the visual system (not the vehicle).
render_step_size = 1.0 / RENDER_FPS

driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(render_step_size / 1.0)   # 1.0 s to reach full steering
driver.SetThrottleDelta(render_step_size / 1.0)   # 1.0 s to reach full throttle
driver.SetBrakingDelta(render_step_size / 0.3)    # 0.3 s to reach full braking
driver.Initialize()


# === Main simulation loop ===
# Real-time loop: render every render_every steps; synchronize all subsystems each step.
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

try:
    while vis.Run() and hmmwv.GetSystem().GetChTime() < SIM_END:
        sim_time = hmmwv.GetSystem().GetChTime()

        if step_number % render_every == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()  # scored core: use interactive driver


        # Synchronize all subsystems in the correct order
        driver.Synchronize(sim_time)
        terrain.Synchronize(sim_time)
        hmmwv.Synchronize(sim_time, driver_inputs, terrain)
        vis.Synchronize(sim_time, driver_inputs)


        # Advance all subsystems; hmmwv.Advance internally calls DoStepDynamics
        driver.Advance(TIME_STEP)
        terrain.Advance(TIME_STEP)
        hmmwv.Advance(TIME_STEP)
        vis.Advance(TIME_STEP)

        step_number += 1
        realtime_timer.Spin(TIME_STEP)

except (RuntimeError, ValueError) as exc:  # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
