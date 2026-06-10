import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                  # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')             # locate vehicle data files

init_loc = chrono.ChVector3d(0, 0, 0.5)                              # initial chassis location
init_rot = chrono.QuatFromAngleZ(0)                                 # initial heading (facing +X)
step_size = 1e-3                                                     # integration step
tire_step_size = 1e-3                                                # tire model step

vehicle = veh.Kraz()                                                # Kraz tractor-trailer truck
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)               # NSC for rigid terrain
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)            # no chassis collision mesh
vehicle.SetChassisFixed(False)                                     # MANDATORY — fixed chassis won't move
vehicle.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))    # spawn pose
vehicle.SetTireStepSize(tire_step_size)                            # tire integration step
vehicle.Initialize()                                               # build the vehicle

vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH, veh.VisualizationType_MESH)        # tractor + trailer chassis mesh
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)  # steering primitives
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES, veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH, veh.VisualizationType_MESH)          # wheel mesh
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH, veh.VisualizationType_MESH)           # tire mesh

system = vehicle.GetSystem()                                       # wrapper-owned ChSystem
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED for contact, after Initialize
print("VEHICLE MASS: ", vehicle.GetTractor().GetMass())           # report tractor mass (Kraz: GetTractor)

terrain = veh.RigidTerrain(system)                                # rigid terrain on the vehicle's system
patch_mat = chrono.ChContactMaterialNSC()                         # rigid-terrain contact material
patch_mat.SetFriction(0.9)                                        # ground friction
patch_mat.SetRestitution(0.01)                                    # ground restitution
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 300.0, 300.0)  # flat 300x300 m patch
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)  # tiled texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                     # patch tint
terrain.Initialize()                                              # finalize terrain

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                  # vehicle-specific Irrlicht window
vis.SetWindowTitle("Kraz Vehicle")                               # window title (before Initialize)
vis.SetWindowSize(1280, 1024)                                    # window size (before Initialize)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 2.1), 12.0, 0.5)      # follow camera on the chassis
vis.Initialize()                                                 # build device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png")) # logo after Initialize
vis.AddSkyBox()                                                  # sky box after Initialize
vis.AddLightDirectional()                                       # vehicle demos use a directional light
vis.AttachVehicle(vehicle.GetTractor())                          # bind chassis/wheel/tire visuals (Kraz: GetTractor)

render_step_size = 1.0 / 50.0                                    # 50 FPS render cadence
driver = veh.ChInteractiveDriverIRR(vis)                        # interactive (keyboard) driver
driver.SetSteeringDelta(render_step_size / 1.0)                 # 1 s to full steering
driver.SetThrottleDelta(render_step_size / 1.0)                # 1 s to full throttle
driver.SetBrakingDelta(render_step_size / 0.3)                # 0.3 s to full brake
driver.Initialize()                                            # finalize driver

render_every = max(1, round(render_step_size / step_size))     # untagged render cadence

realtime_timer = chrono.ChRealtimeStepTimer()                  # wall-clock pacing
while vis.Run():
    time = system.GetChTime()                                 # current sim time

    vis.BeginScene()                                          # render once per frame
    vis.Render()
    vis.EndScene()

    for _ in range(render_every):
        sim_time = system.GetChTime()                        # inner-step time
        driver.Synchronize(sim_time)                          # sync driver
        driver_inputs = driver.GetInputs()                   # current driver inputs

        terrain.Synchronize(sim_time)                        # sync terrain
        vehicle.Synchronize(sim_time, driver_inputs, terrain)  # sync vehicle with inputs + terrain
        vis.Synchronize(sim_time, driver_inputs)             # sync visuals

        driver.Advance(step_size)                            # advance driver
        terrain.Advance(step_size)                           # advance terrain
        vehicle.Advance(step_size)                           # advance vehicle (steps the system)
        vis.Advance(step_size)                               # advance visuals

        realtime_timer.Spin(step_size)                       # pace to real time
