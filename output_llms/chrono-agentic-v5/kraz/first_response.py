"""Kraz tractor-trailer wheeled-vehicle simulation on rigid terrain.

Models a Kraz heavy truck (tractor + semitrailer) driving on a flat rigid
terrain patch. System type: NSC (rigid-terrain catalog vehicle default). The
main bodies are the tractor chassis, the trailer chassis, and the wheel/tire
assemblies created by the veh.Kraz wrapper, plus a single rigid RigidTerrain
patch with defined friction and restitution. A driver system feeds steering /
throttle / braking to the vehicle each step, and an Irrlicht chase-camera window
provides real-time visualization. Expected behavior: the truck rests on the
terrain at start and, when driven, accelerates forward along +X while the
subsystem stack (driver, terrain, vehicle, visualization) is synchronized and
advanced at every timestep in real time.
"""

import math
import os

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Parameters === geometry / physics constants (no bare literals downstream)
step_size = 1e-3                 # integration timestep (s)
sim_end = 12.0                   # bounded recording horizon (s)
render_fps = 50.0                # on-screen / capture frame rate
render_step_size = 1.0 / render_fps
render_steps = math.ceil(render_step_size / step_size)   # precomputed once

terrain_length = 200.0           # rigid patch size along X (m)
terrain_width = 100.0            # rigid patch size along Y (m)
terrain_friction = 0.8           # specified terrain friction
terrain_restitution = 0.01       # specified terrain restitution

init_loc = chrono.ChVector3d(-80.0, 0.0, 0.5)   # tractor spawn (on the patch)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)     # facing +X

# === Data paths === locate bundled Chrono + vehicle assets (truth components)
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Vehicle === Kraz tractor-trailer; wrapper owns its NSC ChSystem
vehicle = veh.Kraz()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)   # NSC for rigid terrain
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle.SetChassisFixed(False)                         # MANDATORY: fixed chassis won't move
vehicle.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
vehicle.Initialize()

vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH, veh.VisualizationType_MESH)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH, veh.VisualizationType_MESH)
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH, veh.VisualizationType_MESH)

# === System & bodies (created by the veh.Kraz wrapper) ===
system = vehicle.GetSystem()                          # ChSystemNSC owned by the wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED for contact
tractor = vehicle.GetTractor()                        # cache: tractor wheeled-vehicle handle
chassis_body = vehicle.GetTractorChassisBody()        # cache: main tractor chassis rigid body
# wheels/spindles: tractor.GetAxle(i)...; trailer + suspension joints built in the wrapper
print("VEHICLE MASS: ", tractor.GetMass())            # report tractor mass

# === Terrain === single flat rigid patch with the specified contact properties
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(terrain_friction)
patch_mat.SetRestitution(terrain_restitution)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, terrain_length, terrain_width)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Visualization === vehicle-aware Irrlicht window: chase camera + sky + lights
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Kraz Tractor-Trailer on Rigid Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 10.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(vehicle.GetTractor())

# === Driver === interactive real-time driver bound to the visual system
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0      # s to ramp steering 0 -> 1
throttle_time = 1.0      # s to ramp throttle 0 -> 1
braking_time = 0.3       # s to ramp brake 0 -> 1
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# === Main loop === synchronize + advance the full subsystem stack in real time

realtime_timer = chrono.ChRealtimeStepTimer()
frame = 0
step_number = 0
try:
    while vis.Run() and system.GetChTime() < sim_end:
        time = system.GetChTime()

        if step_number % render_steps == 0:          # throttled rendering
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()

        driver.Synchronize(time)
        terrain.Synchronize(time)
        vehicle.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        driver.Advance(step_size)
        terrain.Advance(step_size)
        vehicle.Advance(step_size)
        vis.Advance(step_size)

        step_number += 1
        realtime_timer.Spin(step_size)
except (RuntimeError, ValueError) as exc:   # solver divergence / invalid state
    import traceback
    traceback.print_exc()
    raise

# === Post-processing === assemble the review video + physics plot (record mode)
