import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                  # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')              # locate vehicle data files

step_size = 2e-3                                                       # integration step
init_loc = chrono.ChVector3d(-20, 0, 1.5)                             # initial vehicle location
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                           # no initial heading rotation

vehicle = veh.MAN_5t()                                                # MAN 5t catalog wrapper
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)                  # NSC for rigid terrain
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)              # no chassis collision shape
vehicle.SetChassisFixed(False)                                        # chassis must be free to move
vehicle.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))      # spawn pose
vehicle.SetTireType(veh.TireModelType_TMEASY)                        # TMEASY tires on rigid terrain
vehicle.SetTireStepSize(step_size)                                    # tire integration step
vehicle.Initialize()                                                  # build the vehicle subsystems

vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)   # chassis mesh
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)     # wheel mesh
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)      # tire mesh

system = vehicle.GetSystem()                                          # take the wrapper-owned system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # Bullet collision (required for contact)
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())              # report total vehicle mass

terrain = veh.RigidTerrain(system)                                    # rigid terrain on the vehicle system
patch_mat = chrono.ChContactMaterialNSC()                            # NSC contact material for the patch
patch_mat.SetFriction(0.9)                                            # terrain friction
patch_mat.SetRestitution(0.01)                                        # terrain restitution
patch = terrain.AddPatch(                                             # rigid hills from a height map
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),    # centered at origin
    veh.GetDataFile("terrain/height_maps/terrain3.bmp"),            # hilly height map
    128.0, 128.0,                                                    # terrain length, width (m)
    0.0, 4.0,                                                        # min, max hill height (m)
)
patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 64, 64)  # grass texture
terrain.Initialize()                                                  # build terrain collision/visual

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                      # vehicle-specific Irrlicht window
vis.SetWindowTitle("MAN 5t on Rigid Hills")                          # window title
vis.SetWindowSize(1280, 1024)                                         # window size
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)         # chase camera on the chassis
vis.Initialize()                                                     # create the device first
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))     # logo
vis.AddSkyBox()                                                      # sky box
vis.AddLightDirectional()                                            # directional light (vehicle truths use this)
vis.AttachVehicle(vehicle.GetVehicle())                             # bind chassis/wheel/tire visuals

driver = veh.ChInteractiveDriverIRR(vis)                            # interactive driver bound to the vis
render_step_size = 1.0 / 50.0                                        # render cadence (s)
driver.SetSteeringDelta(render_step_size / 1.0)                      # steering ramp rate
driver.SetThrottleDelta(render_step_size / 1.0)                      # throttle ramp rate
driver.SetBrakingDelta(render_step_size / 0.3)                       # braking ramp rate
driver.Initialize()                                                  # initialize the driver

render_every = max(1, round(render_step_size / step_size))           # untagged render-cadence constant
sim_end = 12.0                                                      # simulation duration (s)
realtime_timer = chrono.ChRealtimeStepTimer()                      # real-time pacing
while vis.Run() and system.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        time = system.GetChTime()                                  # current sim time
        driver_inputs = driver.GetInputs()                         # current driver inputs
        driver.Synchronize(time)                                   # driver update
        terrain.Synchronize(time)                                  # terrain update
        vehicle.Synchronize(time, driver_inputs, terrain)         # vehicle update with terrain
        vis.Synchronize(time, driver_inputs)                      # vis update

        driver.Advance(step_size)                                  # advance driver
        terrain.Advance(step_size)                                 # advance terrain
        vehicle.Advance(step_size)                                 # advance wrapper-owned system
        vis.Advance(step_size)                                     # advance vis
        realtime_timer.Spin(step_size)                            # spin to match wall clock
        if system.GetChTime() >= sim_end:
            break
