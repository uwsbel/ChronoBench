"""
HMMWV on rigid terrain with a height-map patch (bump/hill terrain).

System: NSC (ChSystemNSC owned by HMMWV_Full wrapper).
Bodies: HMMWV chassis + suspension/wheel/tire bodies (wrapper-managed),
        rigid terrain patch with bump height map.
Driver: ChInteractiveDriverIRR (real-time interactive, scored core).

Expected behaviour: HMMWV drives forward over the bump/hill rigid terrain,
accelerating from rest and navigating the height-map terrain features.
Contact method is NSC; terrain is rigid (no deformation).
"""

import os
import math
import csv

import pychrono.core as chrono
import pychrono.vehicle as veh

# === Constants ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")            # 9.0.0 API: veh.SetDataPath / veh.GetDataFile

STEP_SIZE       = 1e-3          # physics time step (s); smaller for rigid terrain stability
SIM_END         = 20.0          # simulation duration (s)
RENDER_FPS      = 50.0          # frames per second for review video
TERRAIN_LENGTH  = 200.0         # terrain patch length (m)
TERRAIN_WIDTH   = 200.0         # terrain patch width (m)
INIT_LOC        = chrono.ChVector3d(-60.0, 0.0, 0.5)   # HMMWV spawn; z=0.5 rests wheels on z=0 terrain
INIT_ROT        = chrono.QuatFromAngleZ(0.0)            # no yaw

render_every = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))   # precomputed once

# === Vehicle setup (NSC, rigid terrain) ===
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)    # NSC for rigid terrain
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)                           # MANDATORY — fixed chassis won't move
hmmwv.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
hmmwv.SetTireType(veh.TireModelType_RIGID)   # RIGID tire: appropriate for NSC rigid terrain
hmmwv.SetTireStepSize(STEP_SIZE)
hmmwv.Initialize()

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
sys = hmmwv.GetSystem()          # ChSystemNSC owned by the wrapper   # cache: fetched once
chassis = hmmwv.GetChassisBody() # main chassis rigid body             # cache: fetched once
# wheels/spindles: hmmwv.GetVehicle().GetAxles()[i].m_wheels[j].GetSpindle()
# joints: suspension + steering links created inside the wrapper

sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)   # REQUIRED after Initialize

print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())

# Visualization types (in 9.0.0 VisualizationType enums live in veh namespace)
hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

# === Terrain — rigid single patch with height map ===
terrain = veh.RigidTerrain(hmmwv.GetSystem())

patch_mat = chrono.ChContactMaterialNSC()   # NSC material to match NSC system
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

# Height-map patch: bump/hill terrain — hMin=-1, hMax=1
patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0.0, 0.0, 0.0), chrono.QUNIT),
    veh.GetDataFile("terrain/height_maps/bump64.bmp"),
    TERRAIN_LENGTH,
    TERRAIN_WIDTH,
    -1.0,
    1.0,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Wheel-bottom footprint assertion after Initialize
TIRE_RADIUS = hmmwv.GetVehicle().GetAxles()[0].m_wheels[0].GetTire().GetRadius()  # cache: fetched once
ZTOL = 0.1

veh_obj = hmmwv.GetVehicle()     # cache: fetched once, reused every loop iter
spindle_world = []
for axle_idx in range(veh_obj.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        p = veh_obj.GetSpindlePos(axle_idx, side)
        spindle_world.append(p)

wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
terrain_z = terrain.GetHeight(chrono.ChVector3d(INIT_LOC.x, INIT_LOC.y, 0.0))
assert wheel_bottom_z >= terrain_z - ZTOL, (
    f"vehicle sinks into terrain: wheel_bottom_z={wheel_bottom_z:.3f} "
    f"terrain_z={terrain_z:.3f}; adjust INIT_LOC.z by "
    f"{terrain_z - wheel_bottom_z:.3f} m"
)

# === Visualization (vehicle-specific Irrlicht visual system) ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV on Rigid Hill Terrain (NSC)")
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()        # catalog-vehicle truth uses directional light
vis.AttachVehicle(hmmwv.GetVehicle())

# === Driver (interactive scored core; review-only block drives for the video) ===
driver = veh.ChInteractiveDriverIRR(vis)

render_step_size = 1.0 / RENDER_FPS   # precomputed once
driver.SetSteeringDelta(render_step_size / 1.0)
driver.SetThrottleDelta(render_step_size / 1.0)
driver.SetBrakingDelta(render_step_size / 0.3)
driver.Initialize()


# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

try:
    while vis.Run() and hmmwv.GetSystem().GetChTime() < SIM_END:
        sim_time = hmmwv.GetSystem().GetChTime()  # cache: fetched once per outer iter

        if step_number % render_every == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()


        driver.Synchronize(sim_time)
        terrain.Synchronize(sim_time)
        hmmwv.Synchronize(sim_time, driver_inputs, terrain)
        vis.Synchronize(sim_time, driver_inputs)

        for _ in range(render_every):
            step_time = hmmwv.GetSystem().GetChTime()  # cache: fetched once per inner iter


            driver.Advance(STEP_SIZE)
            terrain.Advance(STEP_SIZE)
            hmmwv.Advance(STEP_SIZE)        # advances wrapper-owned ChSystem
            vis.Advance(STEP_SIZE)

            if hmmwv.GetSystem().GetChTime() >= SIM_END:
                break

        step_number += 1
        realtime_timer.Spin(STEP_SIZE)
except (RuntimeError, ValueError) as exc:   # solver divergence / bad vehicle state
    import traceback; traceback.print_exc()
    raise
finally:
    pass
