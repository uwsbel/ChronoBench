import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                 # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')             # locate vehicle data files

# Vehicle initial position and orientation
init_loc = chrono.ChVector3d(0, 0, 0.5)                             # initial position (flat terrain, z=0.5 above ground)
init_rot = chrono.QuatFromAngleZ(0)                                  # no rotation

# Simulation parameters
step_size = 1e-3                                                     # physics time step (1 ms)
render_fps = 50                                                      # target render / video frame rate
render_step_size = 1.0 / render_fps                                  # seconds per frame
render_steps = math.ceil(render_step_size / step_size)              # physics steps per render frame

# Create the Gator vehicle
gator = veh.Gator()
gator.SetContactMethod(chrono.ChContactMethod_NSC)                  # NSC for rigid terrain
gator.SetChassisCollisionType(veh.CollisionType_NONE)               # no chassis collision mesh
gator.SetChassisFixed(False)                                        # MANDATORY — fixed chassis won't move
gator.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))      # set spawn position
gator.SetTireType(veh.TireModelType_TMEASY)                         # TMEASY tire model (prompt requirement)
gator.SetTireStepSize(step_size)                                    # tire integrator step
gator.Initialize()                                                   # finalize vehicle initialization

system = gator.GetSystem()                                           # obtain vehicle-owned system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET) # REQUIRED after Initialize for contact

print("VEHICLE MASS: ", gator.GetVehicle().GetMass())               # truth's literal vehicle mass banner

# Set mesh visualization for all vehicle components
gator.SetChassisVisualizationType(veh.VisualizationType_MESH)       # chassis mesh
gator.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)  # suspension primitives
gator.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)    # steering primitives
gator.SetWheelVisualizationType(veh.VisualizationType_MESH)         # wheel mesh
gator.SetTireVisualizationType(veh.VisualizationType_MESH)          # tire mesh

# Create rigid terrain
terrain = veh.RigidTerrain(system)                                  # rigid terrain attached to vehicle system

patch_mat = chrono.ChContactMaterialNSC()                           # NSC material to match contact method
patch_mat.SetFriction(0.9)                                          # friction coefficient
patch_mat.SetRestitution(0.01)                                      # low restitution

terrainLength = 200.0                                               # terrain patch length (m)
terrainWidth = 200.0                                                # terrain patch width (m)

patch = terrain.AddPatch(                                            # add flat terrain patch
    patch_mat,
    chrono.CSYSNORM,                                                 # centered at origin
    terrainLength,
    terrainWidth
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)  # custom tile texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                      # terrain color

terrain.Initialize()                                                 # finalize terrain

# Build vehicle Irrlicht visualization — Initialize FIRST, then add scene elements
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                    # vehicle-specific vis system
vis.SetWindowTitle("Gator Vehicle Simulation")                      # window caption
vis.SetWindowSize(1280, 1024)                                        # window resolution
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)       # chase camera config
vis.Initialize()                                                     # FIRST — then add scene elements
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))    # Chrono logo
vis.AddSkyBox()                                                      # sky background
vis.AddLightDirectional()                                            # directional sun light
vis.AttachVehicle(gator.GetVehicle())                               # attach vehicle to vis

# Interactive driver (keyboard-controlled, matches truth for catalog vehicle demos)
driver = veh.ChInteractiveDriverIRR(vis)                            # interactive driver bound to vis

steering_time = 1.0                                                  # seconds to full steering
throttle_time = 1.0                                                  # seconds to full throttle
braking_time = 0.3                                                   # seconds to full brake

driver.SetSteeringDelta(render_step_size / steering_time)           # steering rate
driver.SetThrottleDelta(render_step_size / throttle_time)           # throttle rate
driver.SetBrakingDelta(render_step_size / braking_time)             # braking rate
driver.Initialize()                                                  # finalize driver

render_every = max(1, round(1.0 / (render_fps * step_size)))        # cadence: physics steps per rendered frame
sim_end = 20.0                                                       # simulation end time (s)


realtime_timer = chrono.ChRealtimeStepTimer()                       # real-time pacing timer
step_number = 0                                                      # step counter

while vis.Run() and system.GetChTime() < sim_end:
    time = system.GetChTime()                                        # current simulation time

    if step_number % render_steps == 0:                              # throttled rendering at 50 fps
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()                              # get current driver commands


    driver.Synchronize(time)                                         # update driver state
    terrain.Synchronize(time)                                        # update terrain state
    gator.Synchronize(time, driver_inputs, terrain)                 # update vehicle with driver + terrain
    vis.Synchronize(time, driver_inputs)                             # update visualization

    driver.Advance(step_size)                                        # advance driver
    terrain.Advance(step_size)                                       # advance terrain
    gator.Advance(step_size)                                         # advance vehicle + owned system
    vis.Advance(step_size)                                           # advance vis

    step_number += 1                                                 # increment step counter
    realtime_timer.Spin(step_size)                                   # spin to maintain real-time pace
