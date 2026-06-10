"""
Kraz truck double lane change maneuver simulation.
System type: NSC (Non-Smooth Contact) for rigid terrain.
Main bodies: Kraz truck on flat rigid terrain performing a time-based double lane change.
Expected behavior: truck accelerates, executes double lane change via driver inputs.
"""
import os
import math

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Named constants ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

time_step = 1e-3
sim_end = 30.0
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))

REC = bool(os.environ.get("SIMBENCH_RECORD"))

# === Vehicle init parameters ===
init_loc = chrono.ChVector3d(-15, 0, 0.5)
init_rot = chrono.QUNIT

# === Create Kraz truck ===
kraz = veh.Kraz()
kraz.SetContactMethod(chrono.ChContactMethod_NSC)
kraz.SetChassisCollisionType(veh.CollisionType_NONE)
kraz.SetChassisFixed(False)
kraz.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
kraz.SetTireStepSize(time_step)
kraz.Initialize()

system = kraz.GetSystem()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
print("VEHICLE MASS: ", kraz.GetTractor().GetMass())

# === Terrain (rigid flat) ===
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,
    200.0,
    200.0,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Visualization ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Kraz Double Lane Change")
vis.SetWindowSize(1280, 720)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AddGrid(2.0, 2.0, 40, 40, chrono.ChCoordsysd(), chrono.ChColor(0.4, 0.4, 0.4))
vis.AttachVehicle(kraz.GetTractor())
vis.SetChaseCamera(chrono.ChVector3d(3, 0, 2.1), 25.0, 10.5)

# === Driver (interactive — scored core default) ===
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
driver.SetSteeringDelta(render_every * time_step / steering_time)
driver.SetThrottleDelta(render_every * time_step / throttle_time)
driver.SetBrakingDelta(render_every * time_step / braking_time)
driver.Initialize()

# === Scored core: scripted double lane change (time-based) ===
steering_profile = [
    (0.0, 0.0, 0.0, 0.0),
    (2.0, 0.0, 0.8, 0.0),
    (5.0, 0.4, 0.6, 0.0),
    (8.0, -0.4, 0.6, 0.0),
    (11.0, -0.4, 0.6, 0.0),
    (14.0, 0.0, 0.5, 0.0),
    (30.0, 0.0, 0.5, 0.0),
]

# === CSV logging (review-only) ===

# === Main loop ===
frame = 0
step_number = 0
driver_inputs = driver.GetInputs()

while vis.Run() and system.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()


    for _ in range(render_every):
        sim_time = system.GetChTime()

        # Scored core: apply time-based driver inputs for double lane change
        for t_start, s_target, th_target, br_target in steering_profile:
            if sim_time >= t_start:
                s_cur = driver_inputs.m_steering
                th_cur = driver_inputs.m_throttle
                br_cur = driver_inputs.m_braking
                alpha = 0.05
                driver_inputs.m_steering = s_cur + alpha * (s_target - s_cur)
                driver_inputs.m_throttle = th_cur + alpha * (th_target - th_cur)
                driver_inputs.m_braking = br_cur + alpha * (br_target - br_cur)

        driver.Synchronize(sim_time)
        terrain.Synchronize(sim_time)
        kraz.Synchronize(sim_time, driver_inputs, terrain)
        vis.Synchronize(sim_time, driver_inputs)

        driver.Advance(time_step)
        terrain.Advance(time_step)
        kraz.Advance(time_step)
        vis.Advance(time_step)

        step_number += 1


        if system.GetChTime() >= sim_end:
            break
