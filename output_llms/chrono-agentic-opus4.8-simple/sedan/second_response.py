import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                 # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')             # locate vehicle data files

step_size = 2e-3                                                      # integration step (s)
sim_end = 12.0                                                        # simulation duration (s)

system = chrono.ChSystemNSC()                                        # shared world for both sedans
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED for contact
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))  # g = 9.81 down
system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)          # stable iterative solver for contacts
system.GetSolver().AsIterative().SetMaxIterations(150)              # solver iteration cap
system.SetMaxPenetrationRecoverySpeed(4.0)                          # contact recovery clamp

terrain = veh.RigidTerrain(system)                                  # rigid terrain on the shared system
patch_mat = chrono.ChContactMaterialNSC()                           # NSC patch material
patch_mat.SetFriction(0.9)                                          # tire grip
patch_mat.SetRestitution(0.01)                                      # nearly inelastic
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 200.0, 100.0)  # 200 x 100 m flat patch
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.8))                      # neutral road color
patch.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 200, 200)  # concrete texture
terrain.Initialize()                                               # build the terrain

init_loc1 = chrono.ChVector3d(0, -1.5, 0.5)                         # first sedan spawn
init_rot1 = chrono.QuatFromAngleZ(0)                                # facing +X
vehicle1 = veh.BMW_E90(system)                                     # first sedan on the shared system
vehicle1.SetChassisCollisionType(veh.CollisionType_NONE)           # no chassis collision shell
vehicle1.SetChassisFixed(False)                                    # MANDATORY — fixed chassis won't move
vehicle1.SetInitPosition(chrono.ChCoordsysd(init_loc1, init_rot1)) # place first sedan
vehicle1.SetTireType(veh.TireModelType_TMEASY)                     # TMEASY tire on rigid road
vehicle1.SetTireStepSize(step_size)                                # tire integration step
vehicle1.Initialize()                                             # build first sedan
vehicle1.SetChassisVisualizationType(veh.VisualizationType_MESH)           # chassis mesh
vehicle1.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)  # suspension links
vehicle1.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)    # steering links
vehicle1.SetWheelVisualizationType(veh.VisualizationType_MESH)             # wheels
vehicle1.SetTireVisualizationType(veh.VisualizationType_MESH)              # tires

init_loc2 = chrono.ChVector3d(0, 1.5, 0.5)                          # second sedan spawn (offset in Y)
init_rot2 = chrono.QuatFromAngleZ(0)                               # facing +X
vehicle2 = veh.BMW_E90(system)                                    # second sedan SHARES the same system
vehicle2.SetChassisCollisionType(veh.CollisionType_NONE)          # no chassis collision shell
vehicle2.SetChassisFixed(False)                                   # MANDATORY — fixed chassis won't move
vehicle2.SetInitPosition(chrono.ChCoordsysd(init_loc2, init_rot2)) # place second sedan
vehicle2.SetTireType(veh.TireModelType_TMEASY)                    # TMEASY tire on rigid road
vehicle2.SetTireStepSize(step_size)                               # tire integration step
vehicle2.Initialize()                                            # build second sedan
vehicle2.SetChassisVisualizationType(veh.VisualizationType_MESH)           # chassis mesh
vehicle2.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)  # suspension links
vehicle2.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)    # steering links
vehicle2.SetWheelVisualizationType(veh.VisualizationType_MESH)             # wheels
vehicle2.SetTireVisualizationType(veh.VisualizationType_MESH)              # tires

print("VEHICLE MASS: ", vehicle1.GetVehicle().GetMass())          # report first sedan mass
print("VEHICLE MASS: ", vehicle2.GetVehicle().GetMass())          # report second sedan mass


class SineDriver(veh.ChDriver):                                    # scripted sinusoidal-steering driver
    def __init__(self, vehicle, amplitude, frequency):
        super().__init__(vehicle)
        self.amplitude = amplitude                                # peak steering magnitude
        self.frequency = frequency                                # steering oscillation rate (Hz)

    def Synchronize(self, time):
        self.SetThrottle(0.4)                                     # steady forward throttle
        self.SetBraking(0.0)                                      # no braking
        self.SetSteering(self.amplitude * math.sin(2.0 * math.pi * self.frequency * time))  # sinusoidal steering


driver1 = SineDriver(vehicle1.GetVehicle(), 0.4, 0.2)            # first sedan steering law
driver1.Initialize()                                            # init first driver
driver2 = SineDriver(vehicle2.GetVehicle(), 0.4, 0.2)           # second sedan steering law
driver2.Initialize()                                           # init second driver

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                 # vehicle Irrlicht window
vis.SetWindowTitle("Two Sedans")                                # window title
vis.SetWindowSize(1280, 1024)                                   # window size
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)    # chase camera on the first sedan
vis.Initialize()                                               # build the device FIRST
vis.AddLightDirectional()                                      # directional light (vehicle truths)
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))  # logo
vis.AddSkyBox()                                                # sky box
vis.AttachVehicle(vehicle1.GetVehicle())                       # bind the first sedan visuals

render_step_size = 1.0 / 50.0                                   # 50 fps render cadence
render_steps = math.ceil(render_step_size / step_size)         # physics steps per rendered frame
render_every = render_steps                                    # untagged cadence constant

realtime_timer = chrono.ChRealtimeStepTimer()                  # wall-clock pacing
while vis.Run() and system.GetChTime() < sim_end:
    time = system.GetChTime()                                 # current sim time

    vis.BeginScene()                                         # draw the frame
    vis.Render()
    vis.EndScene()

    for _ in range(render_every):
        time = system.GetChTime()                            # step-local time
        driver_inputs1 = driver1.GetInputs()                 # first sedan inputs
        driver_inputs2 = driver2.GetInputs()                 # second sedan inputs

        driver1.Synchronize(time)                            # update first driver law
        driver2.Synchronize(time)                            # update second driver law
        vehicle1.Synchronize(time, driver_inputs1, terrain) # feed inputs to first sedan
        vehicle2.Synchronize(time, driver_inputs2, terrain) # feed inputs to second sedan
        terrain.Synchronize(time)                            # update terrain
        vis.Synchronize(time, driver_inputs1)               # sync the window to the first sedan

        driver1.Advance(step_size)                          # advance first driver
        driver2.Advance(step_size)                          # advance second driver
        vehicle1.Advance(step_size)                         # advance first sedan subsystems
        vehicle2.Advance(step_size)                         # advance second sedan subsystems
        terrain.Advance(step_size)                          # advance terrain
        vis.Advance(step_size)                              # advance the window

        system.DoStepDynamics(step_size)                    # step the shared world once for both sedans

        if system.GetChTime() >= sim_end:
            break

    realtime_timer.Spin(step_size)                          # pace to wall clock
