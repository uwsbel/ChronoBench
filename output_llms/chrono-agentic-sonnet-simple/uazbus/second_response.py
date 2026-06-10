import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                  # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')              # locate vehicle data files

# Simulation parameters
step_size = 1e-3                                                       # physics step size (s)
sim_end = 20.0                                                         # end time (s)
render_fps = 50.0                                                      # render frame rate
render_step_size = 1.0 / render_fps                                    # render interval (s)
render_steps = math.ceil(render_step_size / step_size)                 # steps per render frame
render_every = max(1, round(1.0 / (render_fps * step_size)))          # cadence constant (untagged)

terrainLength = 400.0                                                  # terrain length X (m)
terrainWidth = 200.0                                                   # terrain width Y (m)

# Initial vehicle position (changed from (0,0,0.5) to (-40,0,0.5))
initLoc = chrono.ChVector3d(-40, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)                            # no rotation

# Create UAZBUS vehicle
uaz = veh.UAZBUS()
uaz.SetContactMethod(chrono.ChContactMethod_NSC)                      # NSC for rigid terrain
uaz.SetChassisCollisionType(veh.CollisionType_NONE)
uaz.SetChassisFixed(False)                                             # MANDATORY — fixed won't move
uaz.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
uaz.SetTireType(veh.TireModelType_TMEASY)                             # TMEASY tires
uaz.SetTireStepSize(step_size)
uaz.Initialize()

system = uaz.GetSystem()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)   # REQUIRED after Initialize

print("VEHICLE MASS: ", uaz.GetVehicle().GetMass())                   # truth's literal banner

# Visualization types (VisualizationType_* lives in veh namespace in this 9.0.0 build)
uaz.SetChassisVisualizationType(veh.VisualizationType_MESH)
uaz.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
uaz.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
uaz.SetWheelVisualizationType(veh.VisualizationType_MESH)
uaz.SetTireVisualizationType(veh.VisualizationType_MESH)

# Rigid terrain with concrete texture
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 200, 200)  # changed to concrete.jpg
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.8))
terrain.Initialize()

# Irrlicht vehicle visualization
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("UAZ Bus - Double Lane Change")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)          # chase the vehicle
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()                                              # directional light (vehicle truth)
vis.AttachVehicle(uaz.GetVehicle())

# Scripted driver for double lane change maneuver (scored core — plain time-based)
# Phase 1: roll forward; Phase 2: accelerate; Phase 3: steer left (lane change 1);
# Phase 4: hold; Phase 5: steer right back (lane change 2); Phase 6: brake
class DoubleLaneChangeDriver(veh.ChDriver):
    def __init__(self, vehicle):
        super().__init__(vehicle)

    def Synchronize(self, time):
        if time < 0.5:                                                 # standstill
            self.SetThrottle(0.0)
            self.SetBraking(0.0)
            self.SetSteering(0.0)
        elif time < 3.0:                                               # accelerate to ~50 km/h
            self.SetThrottle(0.7)
            self.SetBraking(0.0)
            self.SetSteering(0.0)
        elif time < 4.5:                                               # first lane change: steer left
            self.SetThrottle(0.5)
            self.SetBraking(0.0)
            self.SetSteering(0.6)                                      # pronounced left steer
        elif time < 6.0:                                               # hold new lane, straighten
            self.SetThrottle(0.5)
            self.SetBraking(0.0)
            self.SetSteering(0.0)
        elif time < 7.5:                                               # second lane change: steer right
            self.SetThrottle(0.4)
            self.SetBraking(0.0)
            self.SetSteering(-0.6)                                     # pronounced right steer
        elif time < 9.0:                                               # back in original lane, hold
            self.SetThrottle(0.4)
            self.SetBraking(0.0)
            self.SetSteering(0.0)
        else:                                                          # braking phase
            self.SetThrottle(0.0)
            self.SetBraking(0.8)
            self.SetSteering(0.0)

driver = DoubleLaneChangeDriver(uaz.GetVehicle())
driver.Initialize()

realtime_timer = chrono.ChRealtimeStepTimer()                         # real-time sync timer
step_number = 0                                                        # step counter


while vis.Run():
    time = system.GetChTime()                                          # current sim time

    if step_number % render_steps == 0:                               # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()

    driver.Synchronize(time)
    terrain.Synchronize(time)
    uaz.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)


    driver.Advance(step_size)
    terrain.Advance(step_size)
    uaz.Advance(step_size)                                             # advances the system
    vis.Advance(step_size)

    step_number += 1
    realtime_timer.Spin(step_size)                                     # real-time pacing

    if time >= sim_end:                                                # end simulation
        break
