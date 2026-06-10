import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')            # locate vehicle data files

# Simulation parameters
step_size = 1e-3                  # physics time step (s)
render_fps = 50.0                 # rendering frequency (Hz)
sim_end = 20.0                    # simulation end time (s)

# Initial vehicle position and orientation
init_pos = chrono.ChVector3d(0, 0, 0.5)          # spawn location (x, y, z)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)      # no initial rotation

# Create Kraz tractor-trailer vehicle
kraz = veh.Kraz()
kraz.SetContactMethod(chrono.ChContactMethod_NSC)          # NSC for rigid terrain
kraz.SetChassisFixed(False)                                # allow chassis to move
kraz.SetInitPosition(chrono.ChCoordsysd(init_pos, init_rot))
kraz.SetTireStepSize(step_size)                            # tire sub-step size
kraz.Initialize()

system = kraz.GetSystem()                                  # get the wrapper-owned ChSystem
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize

# Set visualization types for all subsystems (Kraz takes tractor+trailer args for most)
kraz.SetChassisVisualizationType(veh.VisualizationType_MESH, veh.VisualizationType_MESH)
kraz.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES, veh.VisualizationType_PRIMITIVES)
kraz.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
kraz.SetWheelVisualizationType(veh.VisualizationType_MESH, veh.VisualizationType_MESH)
kraz.SetTireVisualizationType(veh.VisualizationType_MESH, veh.VisualizationType_MESH)

print("VEHICLE MASS: ", kraz.GetTractor().GetMass())       # report tractor mass

# Rigid terrain with NSC contact material
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()                  # NSC contact for rigid terrain
patch_mat.SetFriction(0.9)                                 # friction coefficient
patch_mat.SetRestitution(0.01)                             # low restitution
patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,                   # centered at origin, no rotation
    200.0,                             # terrain length (X) in meters
    200.0                              # terrain width (Y) in meters
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)   # road texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                              # terrain color
terrain.Initialize()                                       # finalize terrain

# Vehicle Irrlicht visualization (vehicle-specific visual system)
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Kraz Truck Simulation")                # window title
vis.SetWindowSize(1280, 1024)                              # window dimensions
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 12.0, 0.5)  # chase camera for truck
vis.Initialize()                                           # MUST be called first
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))   # logo
vis.AddSkyBox()                                            # sky environment
vis.AddLightDirectional()                                  # directional lighting
vis.AttachVehicle(kraz.GetTractor())                       # Kraz: attach vis to tractor

# Interactive driver (scored core default for catalog vehicles)
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0                          # time to reach max steering
throttle_time = 1.0                          # time to reach max throttle
braking_time = 0.3                           # time to reach max braking
render_step_size = 1.0 / render_fps          # render step size
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

render_every = max(1, round(1.0 / (render_fps * step_size)))   # steps per render frame


realtime_timer = chrono.ChRealtimeStepTimer()                  # wall-clock sync timer
step_number = 0

while vis.Run():
    time = system.GetChTime()                                  # current simulation time

    if step_number % render_every == 0:                        # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()                         # get current driver inputs

    # Synchronize all subsystems
    driver.Synchronize(time)
    terrain.Synchronize(time)
    kraz.Synchronize(time, driver_inputs, terrain)             # vehicle synchronize with terrain
    vis.Synchronize(time, driver_inputs)

    # Advance all subsystems
    driver.Advance(step_size)
    terrain.Advance(step_size)
    kraz.Advance(step_size)                                    # advances the wrapper-owned system
    vis.Advance(step_size)


    step_number += 1
    realtime_timer.Spin(step_size)                             # keep real-time pace
