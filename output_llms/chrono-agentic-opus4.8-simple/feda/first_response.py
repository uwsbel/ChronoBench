import math                                                           # render-cadence math
import pychrono.core as chrono                                        # core PyChrono
import pychrono.vehicle as veh                                        # vehicle catalog
import pychrono.irrlicht as chronoirr                                 # Irrlicht renderer

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                 # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')             # locate vehicle data files

init_loc = chrono.ChVector3d(0, 0, 0.5)                               # vehicle spawn location
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                           # vehicle spawn orientation (identity)

step_size = 1e-3                                                      # integration step
render_step_size = 1.0 / 50.0                                        # 50 fps rendering

feda = veh.FEDA()                                                    # FED-Alpha catalog vehicle
feda.SetContactMethod(chrono.ChContactMethod_NSC)                    # NSC for rigid terrain
feda.SetChassisCollisionType(veh.CollisionType_NONE)                # no chassis collision shape
feda.SetChassisFixed(False)                                          # chassis must be free to move
feda.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))        # initial pose
feda.SetTireType(veh.TireModelType_PAC02)                           # FEDA tire model
feda.SetTireStepSize(step_size)                                     # tire integration step
feda.Initialize()                                                   # build the vehicle subsystems

feda.SetChassisVisualizationType(veh.VisualizationType_MESH)     # mesh chassis
feda.SetSuspensionVisualizationType(veh.VisualizationType_MESH)  # mesh suspension
feda.SetSteeringVisualizationType(veh.VisualizationType_MESH)    # mesh steering
feda.SetWheelVisualizationType(veh.VisualizationType_MESH)       # mesh wheels
feda.SetTireVisualizationType(veh.VisualizationType_MESH)        # mesh tires

system = feda.GetSystem()                                           # take the wrapper-owned system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET) # required for contact
print("VEHICLE MASS: ", feda.GetVehicle().GetMass())               # report total vehicle mass

terrain = veh.RigidTerrain(system)                                  # rigid terrain on the vehicle system
patch_mat = chrono.ChContactMaterialNSC()                          # NSC patch material
patch_mat.SetFriction(0.9)                                         # terrain friction
patch_mat.SetRestitution(0.01)                                     # terrain restitution
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 200.0, 200.0) # flat 200x200 m patch
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)  # custom terrain texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                     # patch tint
terrain.Initialize()                                              # build terrain

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                  # vehicle-specific Irrlicht window
vis.SetWindowTitle("FEDA on Rigid Terrain")                       # window title
vis.SetWindowSize(1280, 1024)                                     # window resolution
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5)  # follow camera behind the vehicle
vis.Initialize()                                                 # build the device first
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png")) # logo
vis.AddSkyBox()                                                  # sky box
vis.AddLightDirectional()                                       # directional light (vehicle truth)
vis.AttachVehicle(feda.GetVehicle())                            # bind chassis/wheel/tire visuals

driver = veh.ChInteractiveDriverIRR(vis)                        # interactive keyboard driver
steering_time = 1.0                                             # 0 -> 1 steering in 1 s
throttle_time = 1.0                                            # 0 -> 1 throttle in 1 s
braking_time = 0.3                                             # 0 -> 1 brake in 0.3 s
driver.SetSteeringDelta(render_step_size / steering_time)      # steering rate
driver.SetThrottleDelta(render_step_size / throttle_time)      # throttle rate
driver.SetBrakingDelta(render_step_size / braking_time)        # braking rate
driver.Initialize()                                           # build the driver

render_steps = math.ceil(render_step_size / step_size)        # physics steps per rendered frame
realtime_timer = chrono.ChRealtimeStepTimer()                # wall-clock pacing
step_number = 0                                              # physics step counter

while vis.Run():
    time = system.GetChTime()                              # current sim time

    if step_number % render_steps == 0:                   # throttled rendering at 50 fps
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()                    # current driver command

    driver.Synchronize(time)                              # sync driver
    terrain.Synchronize(time)                             # sync terrain
    feda.Synchronize(time, driver_inputs, terrain)        # sync vehicle with terrain
    vis.Synchronize(time, driver_inputs)                  # sync visualization


    driver.Advance(step_size)                             # advance driver
    terrain.Advance(step_size)                            # advance terrain
    feda.Advance(step_size)                               # advance vehicle (steps the system)
    vis.Advance(step_size)                                # advance visualization

    step_number += 1                                      # next step
    realtime_timer.Spin(step_size)                        # spin so wall-clock matches sim time
