import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')            # locate vehicle data files


class MyDriver(veh.ChDriver):                                        # custom scripted time-based driver
    def __init__(self, vehicle, delay):
        super().__init__(vehicle)
        self.delay = delay                                          # driver-input delay (s) before any action

    def Synchronize(self, time):
        eff = time - self.delay                                     # delayed effective time
        if eff < 0:                                                 # within the initial delay window
            self.SetThrottle(0.0)
            self.SetSteering(0.0)
            self.SetBraking(0.0)
            return
        self.SetThrottle(min(0.7, 3.5 * eff))                       # ramp throttle to 0.7 over 0.2 s, then hold
        self.SetBraking(0.0)
        if eff > 2.0:                                               # sinusoidal steering once past 2 s
            self.SetSteering(0.6 * math.sin(2.0 * math.pi * (eff - 2.0) / 6.0))
        else:
            self.SetSteering(0.0)


init_loc = chrono.ChVector3d(0, 0, 0.5)                              # HMMWV spawn location (geometric-center origin)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                         # QUNIT — no rotation
step_size = 1e-3                                                     # integration step
tire_step_size = step_size                                          # tire force model step

hmmwv = veh.HMMWV_Full()                                            # full HMMWV catalog model
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)                 # NSC for rigid terrain
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)              # no chassis collision geometry
hmmwv.SetChassisFixed(False)                                        # MANDATORY — chassis must be free to move
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))      # initial pose
hmmwv.SetTireType(veh.TireModelType_TMEASY)                        # TMEASY tire model
hmmwv.SetTireStepSize(tire_step_size)                              # tire integration step
hmmwv.Initialize()                                                  # build the vehicle subsystems

system = hmmwv.GetSystem()                                          # wrapper-owned ChSystem
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED for contact, after Initialize
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())              # report total vehicle mass

hmmwv.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)   # primitive vehicle visualization
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetTireVisualizationType(veh.VisualizationType_PRIMITIVES)

terrainLength = 100.0                                              # X direction size
terrainWidth = 100.0                                               # Y direction size
terrain = veh.RigidTerrain(system)                                # rigid flat ground
patch_mat = chrono.ChContactMaterialNSC()                          # NSC contact material for the patch
patch_mat.SetFriction(0.9)                                          # tire-ground friction
patch_mat.SetRestitution(0.01)                                      # near-inelastic contact
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, terrainLength, terrainWidth)  # flat patch at origin
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)          # ground texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                      # ground tint
terrain.Initialize()                                               # finalize terrain

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                   # vehicle-specific Irrlicht window
vis.SetWindowTitle('HMMWV Demo')                                   # window title
vis.SetWindowSize(1280, 1024)                                      # window resolution
vis.SetChaseCamera(chrono.ChVector3d(-3, 0, 1.1), 6.0, 0.5)        # chase camera: trackPoint, dist, height
vis.Initialize()                                                   # build the device FIRST
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))   # corner logo (after Initialize)
vis.AddSkyBox()                                                    # sky box
vis.AddLightDirectional()                                          # single directional light (vehicle truth shape)
vis.AttachVehicle(hmmwv.GetVehicle())                              # bind the camera/HUD to the vehicle

driver = MyDriver(hmmwv.GetVehicle(), 0.5)                          # custom driver with a 0.5 s input delay
driver.Initialize()                                                # finalize the driver

render_step_size = 1.0 / 50.0                                      # 50 frames per second
render_steps = math.ceil(render_step_size / step_size)             # physics steps per rendered frame
sim_end = 4.0                                                      # end the simulation at 4 s
render_every = render_steps                                        # cadence (== render_steps)

realtime_timer = chrono.ChRealtimeStepTimer()                      # keep wall-clock == sim-clock
step_number = 0                                                     # physics step counter
while vis.Run() and system.GetChTime() < sim_end:                  # run until 4 s of simulation time
    time = system.GetChTime()                                      # current sim time

    if step_number % render_steps == 0:                            # render at 50 fps
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()                             # inputs set by MyDriver.Synchronize

    driver.Synchronize(time)                                       # evaluate the scripted control law
    terrain.Synchronize(time)                                      # update terrain
    hmmwv.Synchronize(time, driver_inputs, terrain)                # drive vehicle with the custom inputs
    vis.Synchronize(time, driver_inputs)                           # update visualization


    driver.Advance(step_size)                                      # advance driver
    terrain.Advance(step_size)                                     # advance terrain
    hmmwv.Advance(step_size)                                       # advance the wrapper-owned system
    vis.Advance(step_size)                                         # advance visualization

    step_number += 1                                               # next step
    realtime_timer.Spin(step_size)                                 # spin so wall-clock matches sim time
