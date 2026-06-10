import math
import os
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                 # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')             # locate vehicle data files

step_size = 1e-3                                                      # integration step
tire_step_size = 1e-3                                                 # tire model step
init_loc = chrono.ChVector3d(-40, 0, 0.5)                            # initial vehicle position
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                          # facing +X

uazbus = veh.UAZBUS()                                                 # UAZ bus catalog wrapper
uazbus.SetContactMethod(chrono.ChContactMethod_NSC)                  # NSC for rigid terrain
uazbus.SetChassisCollisionType(veh.CollisionType_NONE)              # no chassis collision shape
uazbus.SetChassisFixed(False)                                        # chassis must be free to move
uazbus.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))      # spawn pose
uazbus.SetTireType(veh.TireModelType_TMEASY)                        # TMEASY tires on rigid road
uazbus.SetTireStepSize(tire_step_size)                              # tire integration step
uazbus.Initialize()                                                  # build the vehicle

uazbus.SetChassisVisualizationType(veh.VisualizationType_MESH)   # chassis mesh
uazbus.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)  # suspension links
uazbus.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)    # steering links
uazbus.SetWheelVisualizationType(veh.VisualizationType_MESH)     # wheel mesh
uazbus.SetTireVisualizationType(veh.VisualizationType_MESH)      # tire mesh

system = uazbus.GetSystem()                                          # wrapper-owned system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET) # required for contact
print("VEHICLE MASS: ", uazbus.GetVehicle().GetMass())              # report total vehicle mass

terrain = veh.RigidTerrain(system)                                   # flat rigid ground
patch_mat = chrono.ChContactMaterialNSC()                            # NSC patch material
patch_mat.SetFriction(0.9)                                           # road friction
patch_mat.SetRestitution(0.01)                                       # nearly inelastic
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 300.0, 50.0)   # long road patch
patch.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 200, 200)  # concrete texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.8))                       # light grey
terrain.Initialize()                                                 # build terrain


class DoubleLaneChangeDriver(veh.ChDriver):                          # scripted double lane change
    def __init__(self, vehicle):
        super().__init__(vehicle)

    def Synchronize(self, time):                                     # scripted control law
        if time < 2.0:                                               # accelerate straight
            self.SetThrottle(0.5)
            self.SetSteering(0.0)
            self.SetBraking(0.0)
        elif time < 4.0:                                            # steer left into lane 2
            self.SetThrottle(0.4)
            self.SetSteering(0.4)
            self.SetBraking(0.0)
        elif time < 5.0:                                            # straighten in lane 2
            self.SetThrottle(0.4)
            self.SetSteering(-0.4)
            self.SetBraking(0.0)
        elif time < 7.0:                                            # steer right back to lane 1
            self.SetThrottle(0.4)
            self.SetSteering(-0.4)
            self.SetBraking(0.0)
        elif time < 8.0:                                            # straighten in lane 1
            self.SetThrottle(0.4)
            self.SetSteering(0.4)
            self.SetBraking(0.0)
        else:                                                       # brake to a stop
            self.SetThrottle(0.0)
            self.SetSteering(0.0)
            self.SetBraking(0.8)


driver = DoubleLaneChangeDriver(uazbus.GetVehicle())                # scripted maneuver driver
driver.Initialize()                                                  # init the driver

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                     # vehicle Irrlicht window
vis.SetWindowTitle("UAZBUS Double Lane Change")                      # window title
vis.SetWindowSize(1280, 1024)                                        # window size
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)        # chase camera on chassis
vis.Initialize()                                                     # build the device first
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))    # logo overlay
vis.AddSkyBox()                                                      # sky box
vis.AddLightDirectional()                                           # directional light (vehicle truths)
vis.AttachVehicle(uazbus.GetVehicle())                             # bind vehicle visuals

render_step_size = 1.0 / 50.0                                        # 50 fps render cadence
render_steps = math.ceil(render_step_size / step_size)              # steps per rendered frame
render_every = render_steps                                         # untagged cadence constant
sim_end = 10.0                                                       # total sim duration


realtime_timer = chrono.ChRealtimeStepTimer()                       # wall-clock pacing
while vis.Run() and system.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        time = system.GetChTime()                                   # current sim time
        driver_inputs = driver.GetInputs()                          # scripted inputs

        driver.Synchronize(time)                                     # update scripted control
        terrain.Synchronize(time)                                   # update terrain
        uazbus.Synchronize(time, driver_inputs, terrain)           # feed inputs to vehicle
        vis.Synchronize(time, driver_inputs)                       # update HUD/visuals

        driver.Advance(step_size)                                  # advance driver
        terrain.Advance(step_size)                                 # advance terrain
        uazbus.Advance(step_size)                                  # advance wrapper-owned system
        vis.Advance(step_size)                                     # advance visuals

        realtime_timer.Spin(step_size)                             # match wall clock
        if system.GetChTime() >= sim_end:
            break
