import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                 # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')             # locate vehicle data files

init_loc = chrono.ChVector3d(0, 0, 0.5)                              # chassis spawn location
init_rot = chrono.QuatFromAngleZ(0)                                  # facing +X
step_size = 1e-3                                                     # integration step
tire_step_size = 1e-3                                                # tire substep
sim_end = 4.0                                                        # stop the sim at t = 4 s

# --- Custom scripted driver: subclass veh.ChDriver, drive by sim time ---
class MyDriver(veh.ChDriver):
    def __init__(self, vehicle, delay):
        super().__init__(vehicle)                                   # bind to the vehicle
        self.delay = delay                                          # input delay (s) before commands act

    def Synchronize(self, time):
        eff = time - self.delay                                     # delayed effective time
        # throttle gradually ramps to 0.7 after 0.2 s (past the delay)
        if eff < 0:
            throttle = 0.0                                         # nothing until the delay elapses
        elif eff < 0.2:
            throttle = 0.7 * (eff / 0.2)                            # linear ramp 0 -> 0.7 over 0.2 s
        else:
            throttle = 0.7                                         # hold at 0.7
        self.SetThrottle(throttle)                                  # apply throttle
        # sinusoidal steering, only after 2 s of sim time
        if eff > 2.0:
            steering = 0.5 * math.sin(2.0 * (eff - 2.0))           # sinusoidal steering pattern
        else:
            steering = 0.0                                         # straight ahead before 2 s
        self.SetSteering(steering)                                  # apply steering
        self.SetBraking(0.0)                                        # no braking

# --- HMMWV full vehicle on rigid terrain ---
hmmwv = veh.HMMWV_Full()                                            # full HMMWV model (owns its system)
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)                  # NSC for rigid terrain
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)               # no chassis collision geometry
hmmwv.SetChassisFixed(False)                                        # chassis must be free to move
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))      # spawn pose
hmmwv.SetTireType(veh.TireModelType_TMEASY)                        # TMEASY tire model
hmmwv.SetTireStepSize(tire_step_size)                              # tire substep
hmmwv.Initialize()                                                 # build the vehicle subsystems

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)     # chassis mesh
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)       # wheel mesh
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)        # tire mesh

system = hmmwv.GetSystem()                                         # the wrapper-owned system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED for contact
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())             # report total vehicle mass

# --- Rigid terrain patch ---
terrain = veh.RigidTerrain(system)                                # rigid terrain on the vehicle's system
patch_mat = chrono.ChContactMaterialNSC()                         # NSC contact material
patch_mat.SetFriction(0.9)                                        # tire-road friction
patch_mat.SetRestitution(0.01)                                    # nearly no bounce
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 200.0, 200.0)  # 200x200 m flat patch
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)  # road texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                    # patch color
terrain.Initialize()                                             # finalize terrain

# --- Vehicle Irrlicht visualization ---
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                 # vehicle-specific Irrlicht vis
vis.SetWindowTitle("HMMWV Custom Driver")                        # window title
vis.SetWindowSize(1280, 1024)                                    # window size
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 6.0, 0.5)     # chase the chassis
vis.Initialize()                                                # build the device first
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))  # logo
vis.AddSkyBox()                                                 # sky box
vis.AddLightDirectional()                                       # directional light (vehicle convention)
vis.AttachVehicle(hmmwv.GetVehicle())                          # bind chassis/wheel/tire visuals

# --- Custom driver replacing the default driver system, delay = 0.5 ---
driver = MyDriver(hmmwv.GetVehicle(), 0.5)                       # custom driver with 0.5 s delay
driver.Initialize()                                             # initialize driver state

render_step_size = 1.0 / 50.0                                    # render cadence target
render_every = max(1, round(render_step_size / step_size))      # untagged cadence constant

realtime_timer = chrono.ChRealtimeStepTimer()                   # spin to keep wall-clock in sync
while vis.Run() and system.GetChTime() < sim_end:               # end the sim at t = 4 s
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        time = system.GetChTime()                              # current sim time
        driver_inputs = driver.GetInputs()                     # read scripted inputs

        driver.Synchronize(time)                               # update the scripted control law
        terrain.Synchronize(time)                              # update terrain
        hmmwv.Synchronize(time, driver_inputs, terrain)        # feed inputs into the vehicle
        vis.Synchronize(time, driver_inputs)                   # update the HUD/view

        driver.Advance(step_size)                              # advance driver
        terrain.Advance(step_size)                             # advance terrain
        hmmwv.Advance(step_size)                               # advances the wrapper-owned system
        vis.Advance(step_size)                                 # advance the visualization

        realtime_timer.Spin(step_size)                         # spin in place to match wall clock
        if system.GetChTime() >= sim_end:                      # stop condition
            break
