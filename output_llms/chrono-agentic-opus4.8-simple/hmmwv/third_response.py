import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                 # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')             # locate vehicle data files

init_loc = chrono.ChVector3d(0, 0, 0.5)                              # chassis origin start (HMMWV ref height ~0.5)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                          # facing +X
step_size = 2e-3                                                     # integration step
tire_step_size = 1e-3                                                # tire substep
sim_end = 4.0                                                        # stop the sim at 4 s (prompt §3)

hmmwv = veh.HMMWV_Full()                                             # full HMMWV catalog model
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)                   # NSC for rigid terrain
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)               # no chassis collision mesh
hmmwv.SetChassisFixed(False)                                        # MANDATORY — fixed chassis won't move
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))       # spawn pose
hmmwv.SetTireType(veh.TireModelType_TMEASY)                         # TMEASY tire model
hmmwv.SetTireStepSize(tire_step_size)                              # tire integration step
hmmwv.Initialize()                                                  # build the vehicle
system = hmmwv.GetSystem()                                          # take the wrapper-owned system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET) # REQUIRED, after Initialize
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())               # report total vehicle mass

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)    # chassis mesh
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)  # suspension primitives
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)    # steering primitives
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)     # wheel mesh
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)      # tire mesh

terrain = veh.RigidTerrain(system)                                  # flat rigid ground
patch_mat = chrono.ChContactMaterialNSC()                          # NSC ground material
patch_mat.SetFriction(0.9)                                          # ground friction
patch_mat.SetRestitution(0.01)                                      # near-inelastic ground
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 200.0, 200.0)  # 200x200 m patch
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)  # tile texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                      # ground color
terrain.Initialize()                                               # build the terrain

class MyDriver(veh.ChDriver):                                       # custom scripted driver
    def __init__(self, vehicle, delay):                            # delay before any inputs act
        super().__init__(vehicle)                                  # base ChDriver init
        self.delay = delay                                         # input delay (s)

    def Synchronize(self, time):                                   # control law as a function of time
        eff = time - self.delay                                    # delayed effective time
        if eff < 0:                                                # before the delay elapses
            self.SetThrottle(0.0)                                  # no throttle
            self.SetSteering(0.0)                                  # no steering
            self.SetBraking(0.0)                                   # no braking
            return                                                 # hold neutral
        throttle = 0.0                                             # default throttle
        if eff > 0.2:                                              # ramp begins after 0.2 s
            throttle = min(0.7, 0.7 * (eff - 0.2) / 0.2)           # gradually reach 0.7
        self.SetThrottle(throttle)                                 # apply throttle
        steering = 0.0                                             # default steering
        if eff > 2.0:                                              # sinusoidal steering after 2 s
            steering = 0.5 * math.sin(2.0 * (eff - 2.0))           # sine steering pattern
        self.SetSteering(steering)                                 # apply steering
        self.SetBraking(0.0)                                       # no braking

driver = MyDriver(hmmwv.GetVehicle(), 0.5)                          # custom driver, 0.5 s delay
driver.Initialize()                                                # init the driver

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                   # vehicle Irrlicht window
vis.SetWindowTitle("HMMWV custom driver")                          # window title
vis.SetWindowSize(1280, 1024)                                      # window size
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)        # chase camera
vis.Initialize()                                                   # build the device first
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))  # logo
vis.AddSkyBox()                                                    # sky box
vis.AddLightDirectional()                                         # directional light (vehicle truths)
vis.AttachVehicle(hmmwv.GetVehicle())                             # bind vehicle visuals

render_step_size = 1.0 / 50.0                                      # 50 fps render cadence
render_steps = math.ceil(render_step_size / step_size)            # physics steps per frame
realtime_timer = chrono.ChRealtimeStepTimer()                     # wall-clock pacing
step_number = 0                                                   # physics step counter


while vis.Run() and hmmwv.GetSystem().GetChTime() < sim_end:       # stop at sim_end (4 s)
    time = hmmwv.GetSystem().GetChTime()                          # current sim time

    if step_number % render_steps == 0:                          # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()                           # current driver inputs

    driver.Synchronize(time)                                      # update scripted control
    terrain.Synchronize(time)                                     # update terrain
    hmmwv.Synchronize(time, driver_inputs, terrain)              # update vehicle
    vis.Synchronize(time, driver_inputs)                        # update visuals

    driver.Advance(step_size)                                    # advance driver
    terrain.Advance(step_size)                                   # advance terrain
    hmmwv.Advance(step_size)                                     # advances the wrapper-owned system
    vis.Advance(step_size)                                       # advance visuals


    step_number += 1                                            # advance step counter
    realtime_timer.Spin(step_size)                              # pace to wall-clock
