import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                  # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')             # locate vehicle data files

step_size = 2e-3                                                      # integration step
tire_step_size = 1e-3                                                 # tire substep
render_step_size = 1.0 / 50.0                                         # 50 FPS render cadence

init_loc = chrono.ChVector3d(0, 0, 0.5)                               # first vehicle spawn
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                           # identity heading

# --- First sedan: owns the shared ChSystem ---
car1 = veh.BMW_E90()                                                  # primary sedan (BMW E90)
car1.SetContactMethod(chrono.ChContactMethod_NSC)                    # NSC for rigid terrain
car1.SetChassisCollisionType(veh.CollisionType_NONE)                # no chassis collision mesh
car1.SetChassisFixed(False)                                          # MANDATORY — chassis must move
car1.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))        # spawn pose
car1.SetTireType(veh.TireModelType_TMEASY)                          # TMeasy tires on rigid road
car1.SetTireStepSize(tire_step_size)                               # tire integration step
car1.Initialize()                                                   # build the first vehicle

car1.SetChassisVisualizationType(veh.VisualizationType_MESH)     # mesh chassis
car1.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
car1.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
car1.SetWheelVisualizationType(veh.VisualizationType_MESH)       # mesh wheels
car1.SetTireVisualizationType(veh.VisualizationType_MESH)        # mesh tires

system = car1.GetSystem()                                            # shared system owned by car1
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET) # REQUIRED, after Initialize
print("VEHICLE MASS: ", car1.GetVehicle().GetMass())                # report first vehicle mass

# --- Second sedan: shares car1's system (NOT a fresh wrapper) ---
init_loc2 = chrono.ChVector3d(7, 3, 0.5)                             # second vehicle spawn
init_rot2 = chrono.ChQuaterniond(1, 0, 0, 0)                         # second vehicle heading
car2 = veh.BMW_E90(car1.GetSystem())                                # second sedan on the SAME system
car2.SetChassisCollisionType(veh.CollisionType_NONE)               # no chassis collision mesh
car2.SetChassisFixed(False)                                         # MANDATORY — chassis must move
car2.SetInitPosition(chrono.ChCoordsysd(init_loc2, init_rot2))     # second spawn pose
car2.SetTireType(veh.TireModelType_TMEASY)                         # TMeasy tires
car2.SetTireStepSize(tire_step_size)                              # tire integration step
car2.Initialize()                                                  # build the second vehicle

car2.SetChassisVisualizationType(veh.VisualizationType_MESH)    # mesh chassis
car2.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
car2.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
car2.SetWheelVisualizationType(veh.VisualizationType_MESH)      # mesh wheels
car2.SetTireVisualizationType(veh.VisualizationType_MESH)       # mesh tires
print("VEHICLE MASS: ", car2.GetVehicle().GetMass())               # report second vehicle mass

# --- Rigid terrain shared by both vehicles ---
terrain = veh.RigidTerrain(car1.GetSystem())                       # rigid ground on the shared system
patch_mat = chrono.ChContactMaterialNSC()                          # NSC patch material
patch_mat.SetFriction(0.9)                                         # road friction
patch_mat.SetRestitution(0.01)                                     # nearly inelastic
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 200.0, 200.0) # 200 x 200 m flat patch
patch.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 200, 200)  # concrete texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.8))                      # light grey tint
terrain.Initialize()                                               # finalize terrain

# --- Vehicle-specific Irrlicht visualization (bound to car1) ---
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                   # wheeled-vehicle Irrlicht view
vis.SetWindowTitle("Two Sedans")                                  # window title
vis.SetWindowSize(1280, 1024)                                     # window resolution
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)      # chase camera on car1
vis.Initialize()                                                  # build Irrlicht device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))  # add logo AFTER Initialize
vis.AddSkyBox()                                                   # sky box
vis.AddLightDirectional()                                        # directional light (vehicle truth style)
vis.AttachVehicle(car1.GetVehicle())                            # bind chase view to car1

# --- Drivers: one interactive driver per vehicle ---
driver1 = veh.ChInteractiveDriverIRR(vis)                         # interactive driver for car1
driver1.SetSteeringDelta(render_step_size / 1.0)                 # steering rate
driver1.SetThrottleDelta(render_step_size / 1.0)                # throttle rate
driver1.SetBrakingDelta(render_step_size / 0.3)               # braking rate
driver1.Initialize()                                            # init first driver

driver2 = veh.ChInteractiveDriver(car2.GetVehicle())            # driver for the second vehicle
driver2.SetSteeringDelta(render_step_size / 1.0)               # steering rate
driver2.SetThrottleDelta(render_step_size / 1.0)              # throttle rate
driver2.SetBrakingDelta(render_step_size / 0.3)            # braking rate
driver2.Initialize()                                          # init second driver

sim_end = 12.0                                                 # total simulated time (s)
render_steps = math.ceil(render_step_size / step_size)         # physics steps per render
realtime_timer = chrono.ChRealtimeStepTimer()                  # wall-clock pacing
step_number = 0                                                # physics step counter


while vis.Run() and car1.GetSystem().GetChTime() < sim_end:
    time = car1.GetSystem().GetChTime()                        # current sim time

    if step_number % render_steps == 0:                        # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    # Sinusoidal steering input for both vehicles
    steering1 = 0.4 * math.sin(2.0 * math.pi * 0.1 * time)     # car1 steering law
    steering2 = 0.4 * math.sin(2.0 * math.pi * 0.1 * time + math.pi)  # car2 steering law (out of phase)

    inputs1 = driver1.GetInputs()                              # car1 driver inputs
    inputs1.m_steering = steering1                             # apply sinusoidal steering
    inputs1.m_throttle = 0.3                                   # constant forward throttle
    inputs1.m_braking = 0.0                                    # no braking

    inputs2 = driver2.GetInputs()                              # car2 driver inputs
    inputs2.m_steering = steering2                             # apply sinusoidal steering
    inputs2.m_throttle = 0.3                                   # constant forward throttle
    inputs2.m_braking = 0.0                                    # no braking


    # Synchronize all subsystems for both vehicles
    driver1.Synchronize(time)
    driver2.Synchronize(time)
    terrain.Synchronize(time)
    car1.Synchronize(time, inputs1, terrain)                   # first vehicle
    car2.Synchronize(time, inputs2, terrain)                   # second vehicle
    vis.Synchronize(time, inputs1)                             # HUD follows car1

    # Advance all subsystems for both vehicles
    driver1.Advance(step_size)
    driver2.Advance(step_size)
    terrain.Advance(step_size)
    car1.Advance(step_size)                                    # advances the shared system
    car2.Advance(step_size)                                    # second vehicle subsystems
    vis.Advance(step_size)

    step_number += 1
    realtime_timer.Spin(step_size)                            # spin to wall-clock
