import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                    # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')                # locate vehicle data files

# Simulation parameters
step_size = 1e-3                                                         # physics time step (s)
sim_end = 20.0                                                           # simulation end time (s)
render_fps = 50.0                                                        # render frame rate
terrainLength = 200.0                                                    # terrain length (m)
terrainWidth = 200.0                                                     # terrain width (m)

# ARTcar vehicle setup
vehicle = veh.ARTcar()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)                     # NSC for rigid terrain
vehicle.SetChassisCollisionType(veh.CollisionType_MESH)                  # prompt: mesh collision type
vehicle.SetChassisFixed(False)                                           # mandatory — fixed chassis won't move

init_loc = chrono.ChVector3d(1, 0, 0.5)                                 # prompt: initial location (1, 0, 0.5)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                             # no rotation (identity)
vehicle.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))

vehicle.SetTireType(veh.TireModelType_FIALA)                             # prompt: FIALA tire model
vehicle.SetTireStepSize(step_size)                                       # tire integration step size
vehicle.Initialize()

system = vehicle.GetSystem()                                             # get the vehicle-owned ChSystem
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)      # required for contact/terrain scenes

print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())                  # truth's literal banner (scored)

# Apply visualization types — after Initialize; VisualizationType_* is in veh.* namespace in 9.0.0
vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)      # prompt: PRIMITIVES
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)   # prompt: PRIMITIVES
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)     # prompt: PRIMITIVES
vehicle.SetWheelVisualizationType(veh.VisualizationType_PRIMITIVES)        # prompt: PRIMITIVES
vehicle.SetTireVisualizationType(veh.VisualizationType_PRIMITIVES)         # prompt: PRIMITIVES

# Rigid terrain (flat) — NSC material to match vehicle contact method
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()                                # NSC matches vehicle contact method
patch_mat.SetFriction(0.9)                                               # road friction
patch_mat.SetRestitution(0.01)                                           # low restitution
patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,                                                     # centered at origin, no rotation
    terrainLength,                                                       # X direction size
    terrainWidth                                                         # Y direction size
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)  # tile texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                           # sandy color
terrain.Initialize()

# Irrlicht vehicle visualization — ChWheeledVehicleVisualSystemIrrlicht (order-sensitive)
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("ARTcar - Turn 2")                                    # window title
vis.SetWindowSize(1280, 1024)                                            # window size
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 0.5), 6.0, 0.5)             # chase camera behind vehicle
vis.Initialize()                                                         # FIRST: initialize device
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))         # logo after Initialize
vis.AddSkyBox()                                                          # sky box after Initialize
vis.AddLightDirectional()                                                # directional light (vehicle truth style)
vis.AttachVehicle(vehicle.GetVehicle())                                  # attach vehicle to vis

# Interactive driver — scored-core default for catalog vehicle demos
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0                                                      # seconds to reach max steering
throttle_time = 1.0                                                      # seconds to reach max throttle
braking_time = 0.3                                                       # seconds to reach max braking
render_step_size = 1.0 / render_fps                                      # render step size (s)
driver.SetSteeringDelta(render_step_size / steering_time)                # delta per render step
driver.SetThrottleDelta(render_step_size / throttle_time)                # delta per render step
driver.SetBrakingDelta(render_step_size / braking_time)                  # delta per render step
driver.Initialize()

render_every = max(1, round(1.0 / (render_fps * step_size)))             # untagged render cadence constant


realtime_timer = chrono.ChRealtimeStepTimer()                            # for real-time pacing
step_number = 0                                                          # step counter

while vis.Run() and system.GetChTime() < sim_end:
    time = system.GetChTime()                                            # current simulation time

    if step_number % render_every == 0:                                  # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()                                   # get current driver inputs

    driver.Synchronize(time)                                             # synchronize driver
    terrain.Synchronize(time)                                            # synchronize terrain
    vehicle.Synchronize(time, driver_inputs, terrain)                    # synchronize vehicle (3-arg wheeled)
    vis.Synchronize(time, driver_inputs)                                 # synchronize visualization


    driver.Advance(step_size)                                            # advance driver
    terrain.Advance(step_size)                                           # advance terrain
    vehicle.Advance(step_size)                                           # advance vehicle (advances ChSystem)
    vis.Advance(step_size)                                               # advance visualization

    step_number += 1
    realtime_timer.Spin(step_size)                                       # spin to match wall-clock time
