import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                 # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')             # locate vehicle data files

init_loc = chrono.ChVector3d(-40, 0, 0.5)                            # initial vehicle position
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                          # no initial heading rotation
step_size = 1e-3                                                      # integration step (s)
tire_step_size = 1e-3                                                 # tire force model step (s)

uazbus = veh.UAZBUS()                                                 # UAZ bus catalog wrapper
uazbus.SetContactMethod(chrono.ChContactMethod_NSC)                  # NSC for rigid terrain
uazbus.SetChassisCollisionType(veh.CollisionType_NONE)              # no chassis collision mesh
uazbus.SetChassisFixed(False)                                        # chassis must be free to move
uazbus.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))      # spawn pose in world frame
uazbus.SetTireType(veh.TireModelType_TMEASY)                        # TMeasy tire force model
uazbus.SetTireStepSize(tire_step_size)                              # tire substep
uazbus.Initialize()                                                  # build vehicle subsystems

uazbus.SetChassisVisualizationType(veh.VisualizationType_MESH)     # chassis mesh
uazbus.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)  # suspension links
uazbus.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)    # steering links
uazbus.SetWheelVisualizationType(veh.VisualizationType_MESH)       # wheel rims
uazbus.SetTireVisualizationType(veh.VisualizationType_MESH)        # tire meshes

system = uazbus.GetSystem()                                          # wrapper-owned ChSystem
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET) # required for contact
print("VEHICLE MASS: ", uazbus.GetVehicle().GetMass())             # report total vehicle mass

terrainLength = 200.0                                                # terrain size along X (m)
terrainWidth = 100.0                                                 # terrain size along Y (m)
terrain = veh.RigidTerrain(system)                                   # flat rigid ground
patch_mat = chrono.ChContactMaterialNSC()                           # NSC contact material
patch_mat.SetFriction(0.9)                                           # tire/ground friction
patch_mat.SetRestitution(0.01)                                       # near-inelastic contact
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, terrainLength, terrainWidth)  # flat patch
patch.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 200, 200)       # concrete road
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.8))                       # light gray tint
terrain.Initialize()                                                 # finalize terrain

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                    # vehicle Irrlicht renderer
vis.SetWindowTitle("UAZBUS double lane change")                     # window title
vis.SetWindowSize(1280, 1024)                                        # window resolution
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 6.0, 0.5)        # chase cam track point/dist/height
vis.Initialize()                                                     # create the Irrlicht device
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))   # corner logo
vis.AddSkyBox()                                                      # sky background
vis.AddLightDirectional()                                           # directional light (vehicle truths)
vis.AttachVehicle(uazbus.GetVehicle())                             # bind vehicle visual assets


class DLCDriver(veh.ChDriver):                                       # scripted double-lane-change driver
    def __init__(self, vehicle):
        super().__init__(vehicle)

    def Synchronize(self, time):                                     # time-scheduled steering/throttle/braking
        throttle = 0.0
        steering = 0.0
        braking = 0.0
        if time < 0.5:                                               # ramp up to cruising throttle
            throttle = 0.5 * time / 0.5
        elif time < 5.0:                                             # straight-line cruise before maneuver
            throttle = 0.5
        elif time < 5.7:                                             # first steer (lane change left)
            throttle = 0.5
            steering = 0.45
        elif time < 6.5:                                             # counter-steer back to new lane
            throttle = 0.5
            steering = -0.45
        elif time < 7.0:                                             # straighten in adjacent lane
            throttle = 0.5
            steering = 0.0
        elif time < 7.7:                                             # second steer (return lane right)
            throttle = 0.5
            steering = -0.45
        elif time < 8.5:                                             # counter-steer back to original lane
            throttle = 0.5
            steering = 0.45
        elif time < 9.0:                                             # straighten after maneuver
            throttle = 0.4
            steering = 0.0
        else:                                                        # brake to a stop
            throttle = 0.0
            braking = 0.8
        self.SetThrottle(throttle)                                   # apply scheduled throttle
        self.SetSteering(steering)                                   # apply scheduled steering
        self.SetBraking(braking)                                     # apply scheduled braking


driver = DLCDriver(uazbus.GetVehicle())                            # instantiate scripted driver
driver.Initialize()                                                 # initialize driver state

render_step_size = 1.0 / 50.0                                        # render every 1/50 s
render_steps = math.ceil(render_step_size / step_size)             # physics steps per rendered frame
sim_end = 12.0                                                       # total simulated time (s)

render_every = max(1, round(1.0 / (50.0 * step_size)))             # untagged render cadence

realtime_timer = chrono.ChRealtimeStepTimer()                       # wall-clock pacing
step_number = 0                                                     # physics step counter
while vis.Run() and system.GetChTime() < sim_end:
    time = system.GetChTime()                                       # current sim time

    if step_number % render_steps == 0:                             # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()                             # current driver command

    driver.Synchronize(time)                                        # update scripted driver
    terrain.Synchronize(time)                                       # update terrain
    uazbus.Synchronize(time, driver_inputs, terrain)              # update vehicle
    vis.Synchronize(time, driver_inputs)                          # update visualization


    driver.Advance(step_size)                                       # advance driver
    terrain.Advance(step_size)                                      # advance terrain
    uazbus.Advance(step_size)                                       # advance wrapper-owned system
    vis.Advance(step_size)                                          # advance visualization

    step_number += 1                                               # next step
    realtime_timer.Spin(step_size)                                 # pace to wall-clock
