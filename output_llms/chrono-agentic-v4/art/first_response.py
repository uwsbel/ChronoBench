"""
ARTcar on rigid terrain — PyChrono 9.0.x MBS-in-scene simulation.

Vehicle:   veh.ARTcar()  (catalog wheeled vehicle wrapper)
Terrain:   veh.RigidTerrain with NSC contact material
Visual:    veh.ChWheeledVehicleVisualSystemIrrlicht + real-time loop at 50 fps
Driver:    veh.ChInteractiveDriverIRR (interactive keyboard steering/throttle/brake)
Contact:   ChContactMethod_NSC  (rigid terrain → NSC)
"""

import os
import math
import csv

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr


# === Simulation parameters ===
TIME_STEP = 1e-3
SIM_END = 10.0
RENDER_FPS = 50.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))

# === Paths (catalog vehicle) ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# === ARTcar vehicle ===
artcar = veh.ARTcar()
artcar.SetContactMethod(chrono.ChContactMethod_NSC)
artcar.SetChassisFixed(False)
artcar.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.5), chrono.QUNIT))
artcar.Initialize()
system = artcar.GetSystem()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
print("VEHICLE MASS: ", artcar.GetVehicle().GetMass())

# === RigidTerrain ===
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,
    100.0,
    100.0,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.7, 0.7, 0.6))
terrain.Initialize()

# === Visualization ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("ARTcar — Rigid Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(-3, 5, 3), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(artcar.GetVehicle())

# === Interactive driver ===
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(TIME_STEP / 1.0)
driver.SetThrottleDelta(TIME_STEP / 1.0)
driver.SetBrakingDelta(TIME_STEP / 0.3)
driver.Initialize()

# === System & bodies (wrapper-created — make visible) ===
# system  = artcar.GetSystem()
# chassis = artcar.GetChassisBody()
# terrain = RigidTerrain
# driver  = ChInteractiveDriverIRR
# vis     = ChWheeledVehicleVisualSystemIrrlicht

# === Cleanup ===
csv_file = None

frame = 0
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

try:
    # Open CSV inside try so scored-core finally can close it

    while vis.Run() and system.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()


        for _ in range(RENDER_EVERY):
            sim_time = system.GetChTime()
            driver_inputs = driver.GetInputs()

            driver.Synchronize(sim_time)
            terrain.Synchronize(sim_time)
            artcar.Synchronize(sim_time, driver_inputs, terrain)
            vis.Synchronize(sim_time, driver_inputs)

            driver.Advance(TIME_STEP)
            terrain.Advance(TIME_STEP)
            artcar.Advance(TIME_STEP)
            vis.Advance(TIME_STEP)

            step_number += 1
            realtime_timer.Spin(TIME_STEP)


            if system.GetChTime() >= SIM_END:
                break


except (RuntimeError, ValueError) as exc:
    import traceback
    traceback.print_exc()
    raise
finally:
    if csv_file:
        csv_file.close()
