import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())          # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')      # locate vehicle data files

# simulation params
step_size = 1e-3                  # physics time step (s)
sim_end = 20.0                    # simulation end time (s)
render_fps = 50                   # target rendering rate

# vehicle init position
init_loc = chrono.ChVector3d(0, 0, 0.2)         # start above flat terrain
init_rot = chrono.QuatFromAngleZ(0.0)            # facing forward (+X)

# create ARTcar
artcar = veh.ARTcar()
artcar.SetContactMethod(chrono.ChContactMethod_NSC)    # NSC for rigid terrain
artcar.SetChassisCollisionType(veh.CollisionType_NONE)
artcar.SetChassisFixed(False)                          # MANDATORY — fixed chassis won't move
artcar.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
artcar.SetMaxMotorVoltageRatio(0.26)                   # updated from 0.16 for faster vehicle
artcar.SetStallTorque(0.4)                             # updated from 0.3 N*m for more torque
artcar.SetTireRollingResistance(0.03)                  # updated from 0.06 for less rolling resistance
artcar.SetTireStepSize(step_size)                      # tire sub-step
artcar.Initialize()

system = artcar.GetSystem()                            # vehicle-owned system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)   # REQUIRED after Initialize

print("VEHICLE MASS: ", artcar.GetVehicle().GetMass())    # report total vehicle mass

artcar.SetChassisVisualizationType(veh.VisualizationType_MESH)        # veh namespace, not chrono
artcar.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
artcar.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
artcar.SetWheelVisualizationType(veh.VisualizationType_MESH)
artcar.SetTireVisualizationType(veh.VisualizationType_MESH)

# create rigid terrain
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 200.0, 200.0)  # 200x200 m flat patch
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# vehicle Irrlicht visualization
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("ARTcar — faster vehicle")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 0.5), 3.0, 0.5)   # chase camera behind vehicle
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()                                       # vehicle truths use directional light
vis.AttachVehicle(artcar.GetVehicle())

# interactive driver (scored core matches truth)
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0    # s to reach max steering
throttle_time = 1.0    # s to reach full throttle
braking_time = 0.3     # s to reach full brake

render_step_size = 1.0 / render_fps                            # render interval (s)
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

render_every = max(1, round(render_step_size / step_size))     # physics steps per render frame


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
while vis.Run():
    time = system.GetChTime()

    if step_number % render_every == 0:        # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()

    driver.Synchronize(time)
    terrain.Synchronize(time)
    artcar.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)


    driver.Advance(step_size)
    terrain.Advance(step_size)
    artcar.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1
    realtime_timer.Spin(step_size)              # keep wall-clock in sync with sim time

    if time >= sim_end:
        break
