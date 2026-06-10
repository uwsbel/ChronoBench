import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                 # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')             # locate vehicle data files

init_loc = chrono.ChVector3d(0, 0, 0.5)                              # ARTcar spawn above terrain
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                          # QUNIT, no initial rotation

contact_method = chrono.ChContactMethod_NSC                          # NSC for rigid terrain
step_size = 1e-3                                                     # integration time step
tire_step_size = step_size                                          # tire model sub-step
render_step_size = 1.0 / 50.0                                        # 50 FPS visualization

terrainLength = 100.0                                               # terrain X size
terrainWidth = 100.0                                                # terrain Y size

car = veh.ARTcar()                                                  # ARTcar catalog wrapper
car.SetContactMethod(contact_method)                                # rigid-terrain contact method
car.SetChassisCollisionType(veh.CollisionType_NONE)                 # chassis has no collision shape
car.SetChassisFixed(False)                                          # MANDATORY — chassis must move
car.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))         # spawn pose
car.SetTireType(veh.TireModelType_TMEASY)                           # TMEASY tire on rigid terrain
car.SetTireStepSize(tire_step_size)                                 # tire integration step
car.SetMaxMotorVoltageRatio(0.16)                                   # ARTcar motor voltage ratio
car.SetStallTorque(0.3)                                             # ARTcar stall torque
car.SetTireRollingResistance(0.06)                                 # ARTcar rolling resistance
car.Initialize()                                                    # build the vehicle subsystems

system = car.GetSystem()                                            # the wrapper-owned ChSystem
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET) # REQUIRED for contact

car.SetChassisVisualizationType(veh.VisualizationType_MESH)         # chassis mesh visualization
car.SetSuspensionVisualizationType(veh.VisualizationType_MESH)      # suspension mesh visualization
car.SetSteeringVisualizationType(veh.VisualizationType_MESH)        # steering mesh visualization
car.SetWheelVisualizationType(veh.VisualizationType_MESH)           # wheel mesh visualization
car.SetTireVisualizationType(veh.VisualizationType_MESH)            # tire mesh visualization

print("VEHICLE MASS: ", car.GetVehicle().GetMass())                 # report total vehicle mass

terrain = veh.RigidTerrain(system)                                  # rigid terrain on the vehicle system
patch_mat = chrono.ChContactMaterialNSC()                           # NSC contact material for the patch
patch_mat.SetFriction(0.9)                                          # terrain friction
patch_mat.SetRestitution(0.01)                                      # terrain restitution
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, terrainLength, terrainWidth)  # flat patch at origin
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)          # custom tile texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                       # terrain color
terrain.Initialize()                                                # finalize terrain

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                    # vehicle-specific Irrlicht vis
vis.SetWindowTitle('dart')                                          # window title
vis.SetWindowSize(1280, 1024)                                       # window size
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 0.2), 6.0, 0.5)         # trackPoint, chase distance, height
vis.Initialize()                                                    # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))    # logo
vis.AddSkyBox()                                                     # sky box
vis.AddLightDirectional()                                          # directional light (vehicle truth style)
vis.AttachVehicle(car.GetVehicle())                                 # bind vehicle visual assets

driver = veh.ChInteractiveDriverIRR(vis)                            # interactive keyboard driver
steering_time = 1.0                                                 # seconds 0 -> +1 steering
throttle_time = 1.0                                                 # seconds 0 -> +1 throttle
braking_time = 0.3                                                  # seconds 0 -> +1 braking
driver.SetSteeringDelta(render_step_size / steering_time)           # steering response
driver.SetThrottleDelta(render_step_size / throttle_time)           # throttle response
driver.SetBrakingDelta(render_step_size / braking_time)             # braking response
driver.Initialize()                                                 # build the driver

render_steps = math.ceil(render_step_size / step_size)              # physics steps per rendered frame
realtime_timer = chrono.ChRealtimeStepTimer()                       # wall-clock pacing
step_number = 0                                                     # step counter

while vis.Run():
    time = system.GetChTime()                                      # current sim time


    if step_number % render_steps == 0:                            # throttled rendering at 50 FPS
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()                             # current driver commands

    driver.Synchronize(time)                                       # update driver
    terrain.Synchronize(time)                                      # update terrain
    car.Synchronize(time, driver_inputs, terrain)                  # update vehicle
    vis.Synchronize(time, driver_inputs)                           # update visualization

    driver.Advance(step_size)                                      # advance driver
    terrain.Advance(step_size)                                     # advance terrain
    car.Advance(step_size)                                         # advance vehicle (steps the system)
    vis.Advance(step_size)                                         # advance visualization


    step_number += 1                                               # advance step counter
    realtime_timer.Spin(step_size)                                 # pace to real time
