import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh

chrono.SetChronoDataPath(chrono.GetChronoDataPath())           # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')       # locate vehicle data files

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(0, 0, 0.5)                        # vehicle spawn location
initRot = chrono.ChQuaterniond(1, 0, 0, 0)                    # no initial rotation

# Visualization type for vehicle parts
vis_type = veh.VisualizationType_PRIMITIVES                    # primitive shapes for all components

# Collision type for chassis
chassis_collision_type = veh.CollisionType_NONE                # no chassis collision mesh

# Tire model
tire_model = veh.TireModelType_TMEASY                          # TMEASY tire (as requested)

# Terrain dimensions
terrainLength = 100.0                                          # terrain X size (m)
terrainWidth = 100.0                                           # terrain Y size (m)

# Chase camera target point on chassis
trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)               # offset from chassis origin

# Contact method
contact_method = chrono.ChContactMethod_NSC                    # NSC for rigid terrain

# Simulation step sizes
step_size = 1e-3                                               # physics step (s)
tire_step_size = step_size                                     # tire step equals physics step

# Render interval
render_step_size = 1.0 / 50                                    # 50 FPS render cadence

# Create the HMMWV vehicle, set parameters, and initialize
vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(contact_method)                       # NSC contact
vehicle.SetChassisCollisionType(chassis_collision_type)        # no chassis collision
vehicle.SetChassisFixed(False)                                 # MANDATORY — chassis must be free
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)                                # TMEASY tire model
vehicle.SetTireStepSize(tire_step_size)
vehicle.Initialize()

vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize

# Create the terrain
patch_mat = chrono.ChContactMaterialNSC()                      # NSC contact material
patch_mat.SetFriction(0.9)                                     # friction coefficient
patch_mat.SetRestitution(0.01)                                 # low restitution
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat,
                         chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
                         terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)  # tile texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                 # terrain color
terrain.Initialize()

# Create the vehicle Irrlicht interface
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)                      # chase camera
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()                                      # directional light (vehicle truth style)
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Create the driver system
driver = veh.ChInteractiveDriverIRR(vis)                       # interactive keyboard driver

# Time response for keyboard inputs
steering_time = 1.0                                            # s to go 0 -> +1 steering
throttle_time = 1.0                                            # s to go 0 -> +1 throttle
braking_time = 0.3                                             # s to go 0 -> +1 braking
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# Output vehicle mass
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())        # truth's literal banner

# Steps between render events
render_steps = math.ceil(render_step_size / step_size)         # physics steps per render frame
render_every = render_steps                                     # untagged cadence constant

realtime_timer = chrono.ChRealtimeStepTimer()                  # real-time sync timer
step_number = 0
render_frame = 0


while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Render scene at 50 FPS
    if (step_number % render_steps == 0):
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Get driver inputs
    driver_inputs = driver.GetInputs()

    # Update modules (process inputs from other modules)
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)          # vehicle gets driver + terrain inputs
    vis.Synchronize(time, driver_inputs)


    # Advance simulation for one timestep for all modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)                                 # advances the wrapper-owned ChSystem
    vis.Advance(step_size)

    step_number += 1
    realtime_timer.Spin(step_size)                             # real-time sync
