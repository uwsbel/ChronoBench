"""
ARTcar simulation on rigid terrain (PyChrono 9.0.x, Irrlicht renderer).

System type : NSC (rigid terrain)
Vehicle     : veh.ARTcar — art car demo with FIALA tire model
Terrain     : RigidTerrain with a single flat NSC patch
Driver      : ChInteractiveDriverIRR (interactive keyboard)
Expected    : Vehicle spawned at (1, 0, 0.5), PRIMITIVES visualization,
              chassis MESH collision, FIALA tires; drives on flat ground.
"""

import os
import math
import csv
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr


# === Constants ===
step_size    = 1e-3
sim_end      = 15.0
render_fps   = 50.0
render_step  = 1.0 / render_fps                          # precomputed once
render_every = max(1, round(render_step / step_size))    # precomputed once

terrainLength = 100.0
terrainWidth  = 100.0

initLoc = chrono.ChVector3d(1, 0, 0.5)
initRot = chrono.QuatFromAngleZ(0.0)

# === Data paths (truth-faithful for catalog vehicles) ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# === Vehicle setup ===
artcar = veh.ARTcar()
artcar.SetContactMethod(chrono.ChContactMethod_NSC)
artcar.SetChassisCollisionType(veh.CollisionType_MESH)   # prompt: chassis MESH collision
artcar.SetChassisFixed(False)                            # MANDATORY: free chassis
artcar.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
artcar.SetTireType(veh.TireModelType_FIALA)              # prompt: FIALA tire model
artcar.SetTireStepSize(step_size)
artcar.Initialize()

# === System & bodies (created by the veh.ARTcar wrapper) ===
sys     = artcar.GetSystem()       # ChSystemNSC owned by the wrapper
chassis = artcar.GetChassisBody()  # cache: main chassis rigid body; fetched once
# wheels/spindles: artcar.GetVehicle().GetAxle(i) ...; terrain: RigidTerrain below
# joints: suspension + steering links created inside the wrapper
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize

print("VEHICLE MASS: ", artcar.GetVehicle().GetMass())

# Prevent chassis MESH from colliding with terrain — the tires handle ground contact.
# Chassis collision is only for prop/obstacle interactions.
chassis_cm = chassis.GetCollisionModel()
chassis_cm.SetFamily(2)                   # chassis family = 2
chassis_cm.DisallowCollisionsWith(4)      # family 4 = terrain (set below)

# Visualization types (set AFTER Initialize; prompt: PRIMITIVES for all parts)
artcar.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
artcar.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
artcar.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
artcar.SetWheelVisualizationType(veh.VisualizationType_PRIMITIVES)
artcar.SetTireVisualizationType(veh.VisualizationType_PRIMITIVES)

# === Terrain ===
terrain   = veh.RigidTerrain(sys)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Assign terrain patch body collision family so chassis mesh avoids it
patch_body = patch.GetGroundBody()
terrain_cm = patch_body.GetCollisionModel()
terrain_cm.SetFamily(4)                   # terrain family = 4
terrain_cm.DisallowCollisionsWith(2)      # block chassis (family 2) from terrain
sys.GetCollisionSystem().BindAll()        # rebuild after post-init collision edits

# === Visualization (vehicle-aware Irrlicht) ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("ARTcar Demo")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 0.5), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(artcar.GetVehicle())

# === Driver (interactive — truth-faithful ChInteractiveDriverIRR) ===
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(render_step / 1.0)   # 1.0 s to full steering
driver.SetThrottleDelta(render_step / 1.0)   # 1.0 s to full throttle
driver.SetBrakingDelta(render_step  / 0.3)   # 0.3 s to full braking
driver.Initialize()


# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()

try:
    while vis.Run() and sys.GetChTime() < sim_end:
        time = sys.GetChTime()   # cache: current sim time

        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        driver_inputs = driver.GetInputs()


        # Synchronize subsystems: driver → terrain → vehicle → vis
        driver.Synchronize(time)
        terrain.Synchronize(time)
        artcar.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        for _ in range(render_every):
            driver.Advance(step_size)
            terrain.Advance(step_size)
            artcar.Advance(step_size)
            vis.Advance(step_size)
            if sys.GetChTime() >= sim_end:
                break

        realtime_timer.Spin(step_size)

except (RuntimeError, ValueError) as exc:   # solver divergence / bad vehicle state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
