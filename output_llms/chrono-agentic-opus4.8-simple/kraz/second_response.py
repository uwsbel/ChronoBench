import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                  # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')              # locate vehicle data files

terrainLength = 100.0                                                  # terrain size in X
terrainWidth = 100.0                                                   # terrain size in Y

step_size = 1e-3                                                       # dynamics step
tire_step_size = 1e-3                                                  # tire integration step
render_step_size = 1.0 / 50.0                                          # FPS = 50

init_loc = chrono.ChVector3d(-15, 0, 0.5)                             # truck spawn location
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                          # facing +X (forward)

truck = veh.Kraz()                                                    # semi-trailer truck wrapper
truck.SetChassisFixed(False)                                         # MANDATORY — fixed chassis won't move
truck.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))       # place tractor in world
truck.SetTireStepSize(tire_step_size)                               # tire substep
truck.SetInitFwdVel(0.0)                                            # start from rest
truck.Initialize()                                                  # build the multibody model

truck.SetChassisVisualizationType(veh.VisualizationType_MESH, veh.VisualizationType_PRIMITIVES)
truck.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
truck.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES, veh.VisualizationType_PRIMITIVES)
truck.SetWheelVisualizationType(veh.VisualizationType_MESH, veh.VisualizationType_MESH)
truck.SetTireVisualizationType(veh.VisualizationType_MESH, veh.VisualizationType_MESH)

system = truck.GetSystem()                                          # wrapper-owned system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED for contact/terrain
print("VEHICLE MASS: ", truck.GetTractor().GetMass())              # report tractor mass

terrain = veh.RigidTerrain(truck.GetSystem())                      # flat rigid road
patch_mat = chrono.ChContactMaterialNSC()                          # NSC material for rigid terrain
patch_mat.SetFriction(0.9)                                         # tire-road friction
patch_mat.SetRestitution(0.01)                                     # nearly inelastic
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, terrainLength, terrainWidth)
patch.SetColor(chrono.ChColor(0.5, 0.5, 1.0))                     # road color
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
terrain.Initialize()                                              # finalize terrain

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                  # vehicle-aware Irrlicht system
vis.SetWindowTitle("Semi-trailer truck :: Double Lane Change")    # window title
vis.SetWindowSize(1280, 1024)                                     # window size
vis.SetChaseCamera(chrono.ChVector3d(3, 0, 2.1), 25.0, 10.5)     # track point, distance, height
vis.Initialize()                                                 # build device first
vis.AddLightDirectional()                                        # vehicle demos use a directional light
vis.AddSkyBox()                                                  # sky
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))  # logo
vis.AttachVehicle(truck.GetTractor())                           # bind tractor visual assets


class DoubleLaneChangeDriver(veh.ChDriver):                       # scripted time-based maneuver
    def __init__(self, vehicle):
        super().__init__(vehicle)

    def Synchronize(self, time):
        self.SetThrottle(0.7)                                    # constant throttle to build speed
        self.SetBraking(0.0)                                     # no braking during the maneuver
        if time < 4.0:                                           # straight run-in
            steering = 0.0
        elif time < 5.5:                                         # steer left into the adjacent lane
            steering = 0.18
        elif time < 7.0:                                         # counter-steer to settle in left lane
            steering = -0.18
        elif time < 8.5:                                         # steer right back toward original lane
            steering = -0.18
        elif time < 10.0:                                        # counter-steer to settle in original lane
            steering = 0.18
        else:                                                    # straight again
            steering = 0.0
        self.SetSteering(steering)                               # apply scripted steering


driver = DoubleLaneChangeDriver(truck.GetTractor())             # bind to the tractor
driver.Initialize()                                            # finalize driver

render_steps = math.ceil(render_step_size / step_size)         # physics steps per rendered frame
render_every = render_steps                                    # untagged cadence constant
sim_end = 14.0                                                  # total simulation time


realtime_timer = chrono.ChRealtimeStepTimer()                 # real-time pacing
step_number = 0
while vis.Run() and truck.GetSystem().GetChTime() < sim_end:
    if step_number % render_steps == 0:                       # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    time = truck.GetSystem().GetChTime()                      # current sim time

    driver.Synchronize(time)                                  # compute scripted inputs for this time
    driver_inputs = driver.GetInputs()                       # fetch the freshly computed inputs
    terrain.Synchronize(time)                                 # update terrain
    truck.Synchronize(time, driver_inputs, terrain)          # feed inputs to the truck
    vis.Synchronize(time, driver_inputs)                     # update HUD/camera

    driver.Advance(step_size)                                # advance driver
    terrain.Advance(step_size)                               # advance terrain
    truck.Advance(step_size)                                 # advances the wrapper-owned system
    vis.Advance(step_size)                                   # advance visualization


    step_number += 1
    realtime_timer.Spin(step_size)                           # spin to match wall-clock
