import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                 # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')             # locate vehicle data files

init_loc = chrono.ChVector3d(-20, 0, 1.5)                            # vehicle start (prompt: (-20,0,1.5))
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                          # identity heading
step_size = 1e-3                                                      # integration step
tire_step_size = 1e-3                                                 # tire substep

vehicle = veh.MAN_5t()                                               # prompt vehicle: MAN_5t
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)                 # NSC for rigid terrain
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)              # no chassis collision mesh
vehicle.SetChassisFixed(False)                                       # MANDATORY — fixed chassis won't move
vehicle.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))      # spawn pose
vehicle.SetTireType(veh.TireModelType_TMEASY)                        # TMeasy tires on rigid terrain
vehicle.SetTireStepSize(tire_step_size)                              # tire integration step
vehicle.Initialize()                                                 # build the vehicle subsystems

vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)   # chassis mesh
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)  # suspension links
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)    # steering links
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)     # rims
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)      # tire meshes

system = vehicle.GetSystem()                                         # wrapper-owned system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED for contact, after Initialize
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())             # report total vehicle mass

terrain = veh.RigidTerrain(system)                                   # rigid terrain on the vehicle system
patch_mat = chrono.ChContactMaterialNSC()                            # NSC patch material
patch_mat.SetFriction(0.9)                                           # tire grip
patch_mat.SetRestitution(0.01)                                       # nearly inelastic
terrain_length = 200.0                                               # X extent of the hills (m)
terrain_width = 200.0                                                # Y extent of the hills (m)
patch = terrain.AddPatch(                                            # heightmap patch -> rigid hills
    patch_mat,
    chrono.CSYSNORM,                                                 # centered at origin
    veh.GetDataFile("terrain/height_maps/test64.bmp"),              # bitmap of rolling hills
    terrain_length,                                                  # length (X)
    terrain_width,                                                   # width (Y)
    0.0,                                                             # min height
    4.0,                                                             # max height (hill amplitude)
)
patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 64, 64)  # prompt texture: grass.jpg
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.8))                       # neutral tint over the grass texture
terrain.Initialize()                                                 # build the terrain collision/visual

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                     # vehicle-specific Irrlicht system
vis.SetWindowTitle("MAN 5t on rigid hills")                         # window title
vis.SetWindowSize(1280, 1024)                                        # window pixels
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 2.0), 12.0, 0.5)         # follow point, distance, height
vis.Initialize()                                                     # create the device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))    # corner logo
vis.AddSkyBox()                                                      # sky box
vis.AddLightDirectional()                                           # vehicle truths use a directional light
vis.AttachVehicle(vehicle.GetVehicle())                            # bind chassis/wheel/tire assets

render_step_size = 1.0 / 50.0                                        # 50 FPS render cadence
render_every = max(1, round(render_step_size / step_size))          # untagged cadence constant

driver = veh.ChInteractiveDriverIRR(vis)                             # interactive driver (scored-core default)
steering_time = 1.0                                                  # s to full steering
throttle_time = 1.0                                                  # s to full throttle
braking_time = 0.3                                                   # s to full brake
driver.SetSteeringDelta(render_step_size / steering_time)           # steering ramp rate
driver.SetThrottleDelta(render_step_size / throttle_time)           # throttle ramp rate
driver.SetBrakingDelta(render_step_size / braking_time)             # brake ramp rate
driver.Initialize()                                                 # arm the driver

sim_end = 12.0                                                       # simulation length (s)

realtime_timer = chrono.ChRealtimeStepTimer()                       # wall-clock pacing
while vis.Run() and system.GetChTime() < sim_end:
    vis.BeginScene()                                                # start frame
    vis.Render()                                                    # draw scene
    vis.EndScene()                                                  # finish frame
    for _ in range(render_every):
        time = system.GetChTime()                                   # current sim time
        driver_inputs = driver.GetInputs()                          # steering/throttle/brake
        driver.Synchronize(time)                                    # update driver
        terrain.Synchronize(time)                                   # update terrain
        vehicle.Synchronize(time, driver_inputs, terrain)          # feed inputs + terrain
        vis.Synchronize(time, driver_inputs)                       # update HUD/view
        driver.Advance(step_size)                                  # advance driver
        terrain.Advance(step_size)                                 # advance terrain
        vehicle.Advance(step_size)                                 # advances the wrapper-owned system
        vis.Advance(step_size)                                     # advance visualization
        realtime_timer.Spin(step_size)                             # pace to wall-clock
        if system.GetChTime() >= sim_end:
            break
