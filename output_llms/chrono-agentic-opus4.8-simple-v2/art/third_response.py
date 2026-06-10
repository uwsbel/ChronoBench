import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                  # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')              # locate vehicle data files

initLoc = chrono.ChVector3d(0, 0, 0.5)                                # vehicle spawn location
initRot = chrono.ChQuaterniond(1, 0, 0, 0)                           # spawn orientation (identity)

chassis_vis_type = veh.VisualizationType_PRIMITIVES                  # chassis drawn as primitives
suspension_vis_type = veh.VisualizationType_PRIMITIVES              # suspension primitives
steering_vis_type = veh.VisualizationType_PRIMITIVES                # steering primitives
wheel_vis_type = veh.VisualizationType_PRIMITIVES                   # wheels primitives

chassis_collision_type = veh.CollisionType_NONE                      # no chassis collision geometry

tire_model = veh.TireModelType_TMEASY                               # TMeasy tire force model

terrainHeight = 0                                                    # terrain height
terrainLength = 100.0                                               # size in X direction
terrainWidth = 100.0                                                # size in Y direction

trackPoint = chrono.ChVector3d(0.0, 0.0, 0.2)                       # chase-camera track point on chassis

contact_method = chrono.ChContactMethod_NSC                          # NSC for rigid terrain
contact_vis = False                                                 # do not draw contacts

step_size = 1e-3                                                    # integration step
tire_step_size = step_size                                          # tire substep matches main step

render_step_size = 1.0 / 50                                         # render at 50 FPS

# Create the ARTcar vehicle, set parameters, and initialize
car = veh.ARTcar()                                                  # autonomy research testbed car
car.SetContactMethod(contact_method)                                # NSC contact
car.SetChassisCollisionType(chassis_collision_type)                 # CollisionType_NONE
car.SetChassisFixed(False)                                          # MANDATORY: chassis must move
car.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))           # place chassis in world
car.SetTireType(tire_model)                                         # TMEASY tire
car.SetTireStepSize(tire_step_size)                                 # tire integration step
car.SetMaxMotorVoltageRatio(0.26)                                   # faster: raised 0.16 -> 0.26
car.SetStallTorque(0.4)                                             # faster: raised 0.3 -> 0.4
car.SetTireRollingResistance(0.03)                                  # faster: lowered 0.06 -> 0.03

car.Initialize()                                                    # build the vehicle subsystems

tire_vis_type = veh.VisualizationType_PRIMITIVES                    # tires drawn as primitives

car.SetChassisVisualizationType(chassis_vis_type)                  # apply chassis visualization
car.SetSuspensionVisualizationType(suspension_vis_type)           # apply suspension visualization
car.SetSteeringVisualizationType(steering_vis_type)               # apply steering visualization
car.SetWheelVisualizationType(wheel_vis_type)                     # apply wheel visualization
car.SetTireVisualizationType(tire_vis_type)                       # apply tire visualization

car.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED: Bullet collisions

# Create the terrain
patch_mat = chrono.ChContactMaterialNSC()                          # NSC terrain contact material
patch_mat.SetFriction(0.9)                                         # high friction for traction
patch_mat.SetRestitution(0.01)                                     # nearly inelastic
terrain = veh.RigidTerrain(car.GetSystem())                       # rigid terrain on vehicle's system
patch = terrain.AddPatch(patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),  # flat patch centered at origin
    terrainLength, terrainWidth)                                  # patch extents

patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)  # tiled ground texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                     # sandy tone
terrain.Initialize()                                              # finalize terrain

# Create the vehicle Irrlicht interface and the driver system
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                  # vehicle-aware Irrlicht visual system
vis.SetWindowTitle('dart')                                        # window title
vis.SetWindowSize(1280, 1024)                                     # window resolution
vis.SetChaseCamera(trackPoint, 6.0, 0.5)                          # chase camera behind chassis
vis.Initialize()                                                 # build device FIRST
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png')) # branding logo
vis.AddLightDirectional()                                        # vehicle scenes use a directional light
vis.AddSkyBox()                                                  # sky backdrop
vis.AttachVehicle(car.GetVehicle())                             # bind chassis/wheel/tire visuals

driver_data = veh.vector_Entry([veh.DataDriverEntry(0.0, 0.0, 0.0, 0.0),   # t=0: idle
                                veh.DataDriverEntry(0.1, 1.0, 0.0, 0.0),   # t=0.1: full steer
                                veh.DataDriverEntry(0.5, 1.0, 0.7, 0.0),   # t=0.5: steer + throttle
                                 ])
driver = veh.ChDataDriver(car.GetVehicle(), driver_data)         # scripted open-loop driver
driver.Initialize()                                              # build driver

# output vehicle mass
print("VEHICLE MASS: ", car.GetVehicle().GetMass())              # truth's literal mass banner

# Number of simulation steps between render frames
render_steps = math.ceil(render_step_size / step_size)           # physics steps per rendered frame

realtime_timer = chrono.ChRealtimeStepTimer()                    # wall-clock pacing
step_number = 0                                                  # physics step counter
render_frame = 0                                                 # rendered-frame counter

sim_end = 10.0                                                   # simulation horizon (s)

while vis.Run() and car.GetSystem().GetChTime() < sim_end:
    time = car.GetSystem().GetChTime()                          # current sim time

    # Render scene
    if (step_number % render_steps == 0):                       # throttle rendering to render_steps
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1                                       # count drawn frames

    # Get driver inputs
    driver_inputs = driver.GetInputs()                         # scripted steering/throttle/brake

    # Update modules (process inputs from other modules)
    driver.Synchronize(time)                                    # advance driver schedule
    terrain.Synchronize(time)                                   # update terrain
    car.Synchronize(time, driver_inputs, terrain)              # feed inputs + terrain to vehicle
    vis.Synchronize(time, driver_inputs)                       # update HUD/view


    # Advance simulation for one timestep for all modules
    driver.Advance(step_size)                                  # advance driver
    terrain.Advance(step_size)                                 # advance terrain
    car.Advance(step_size)                                     # advances the wrapper-owned system
    vis.Advance(step_size)                                     # advance visualization

    step_number += 1                                           # next step

    realtime_timer.Spin(step_size)                             # spin to match wall-clock
