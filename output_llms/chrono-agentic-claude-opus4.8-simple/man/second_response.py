import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                   # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')               # locate vehicle data files

init_loc = chrono.ChVector3d(-20, 0, 1.5)                              # initial chassis location (on the hills)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                            # QUNIT, no initial rotation

step_size = 1e-3                                                        # integration step
tire_step_size = step_size                                             # tire force model step
render_step_size = 1.0 / 50.0                                          # 50 FPS render cadence

vehicle = veh.MAN_5t()                                                 # MAN 5t catalog truck
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)                   # NSC for rigid terrain
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)                # no chassis collision shape
vehicle.SetChassisFixed(False)                                         # chassis must be free to move
vehicle.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))        # spawn pose
vehicle.SetTireType(veh.TireModelType_TMEASY)                          # TMEASY tire model
vehicle.SetTireStepSize(tire_step_size)                                # tire integration step
vehicle.Initialize()                                                   # build the vehicle subsystems

system = vehicle.GetSystem()                                           # the wrapper-owned ChSystem
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)    # REQUIRED for contact
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())                # report total vehicle mass

vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)        # mesh chassis
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_MESH)     # mesh suspension
vehicle.SetSteeringVisualizationType(veh.VisualizationType_MESH)       # mesh steering
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)          # mesh wheels
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)           # mesh tires

terrain = veh.RigidTerrain(system)                                     # rigid terrain
patch_mat = chrono.ChContactMaterialNSC()                              # NSC contact material
patch_mat.SetFriction(0.9)                                             # terrain friction
patch_mat.SetRestitution(0.01)                                         # terrain restitution
patch = terrain.AddPatch(                                              # rigid hills from a height map
    patch_mat,
    chrono.CSYSNORM,                                                   # centered at origin
    veh.GetDataFile("terrain/height_maps/bump64.bmp"),                 # height-map bitmap (hills)
    128.0,                                                             # patch length (X)
    128.0,                                                             # patch width (Y)
    0.0,                                                               # minimum height
    4.0                                                                # maximum height
)
patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 16, 16)  # grass texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                          # patch color
terrain.Initialize()                                                   # build the terrain

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                       # vehicle Irrlicht visual system
vis.SetWindowTitle('MAN 5t Demo')                                      # window title
vis.SetWindowSize(1280, 1024)                                          # window size
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 15.0, 0.5)       # chase camera trackpoint/dist/height
vis.Initialize()                                                       # create the Irrlicht device FIRST
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))       # logo (after Initialize)
vis.AddLightDirectional()                                              # directional light (vehicle style)
vis.AddSkyBox()                                                        # sky box
vis.AttachVehicle(vehicle.GetVehicle())                                # bind the vehicle to the chase cam

driver = veh.ChInteractiveDriverIRR(vis)                               # real-time interactive driver
driver.SetSteeringDelta(render_step_size / 1.0)                        # steering rate
driver.SetThrottleDelta(render_step_size / 1.0)                        # throttle rate
driver.SetBrakingDelta(render_step_size / 0.3)                         # braking rate
driver.Initialize()                                                    # build the driver

render_steps = math.ceil(render_step_size / step_size)                 # physics steps per render frame
realtime_timer = chrono.ChRealtimeStepTimer()                          # wall-clock pacing
step_number = 0                                                        # step counter


while vis.Run():                                                       # plain real-time loop (truth form)
    time = vehicle.GetSystem().GetChTime()                            # current sim time

    if step_number % render_steps == 0:                               # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()                                # current driver command

    driver.Synchronize(time)                                          # advance driver state
    terrain.Synchronize(time)                                         # advance terrain
    vehicle.Synchronize(time, driver_inputs, terrain)                 # feed inputs to the vehicle
    vis.Synchronize(time, driver_inputs)                              # sync the visual system

    driver.Advance(step_size)                                         # step driver
    terrain.Advance(step_size)                                        # step terrain
    vehicle.Advance(step_size)                                        # step the wrapper-owned system
    vis.Advance(step_size)                                            # step the visual system

    step_number += 1                                                  # advance counter
    realtime_timer.Spin(step_size)                                    # match wall-clock to sim time
