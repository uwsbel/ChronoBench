import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                 # core Chrono data
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')             # vehicle data subtree

# Highway mesh runs along +Y (x in [-11.6,11.6], y in [-75,76]); face vehicles along +Y
driveDir = chrono.QuatFromAngleZ(chrono.CH_PI / 2)                  # +90 deg about Z -> heading +Y
truckLoc = chrono.ChVector3d(0, -40, 0.8)                           # truck on the highway surface
truckRot = driveDir                                                # truck heading +Y
sedanLoc = chrono.ChVector3d(-3, -25, 0.8)                          # sedan in adjacent lane, ahead
sedanRot = driveDir                                                # sedan heading +Y

step_size = 1e-3                                                      # integration step
tire_step_size = step_size                                           # tire integration step
render_step_size = 1.0 / 50.0                                        # 50 FPS render cadence

truck = veh.Kraz()                                                   # Kraz tractor-trailer wrapper
truck.SetContactMethod(chrono.ChContactMethod_NSC)                   # rigid terrain -> NSC
truck.SetChassisFixed(False)                                         # chassis free to move
truck.SetInitPosition(chrono.ChCoordsysd(truckLoc, truckRot))        # truck spawn pose
# prompt: truck tires rigid — the Kraz wrapper builds its own fixed tire model and
# exposes no SetTireType; it ships rigid-style tires by default, so no setter call
truck.Initialize()                                                   # build truck subsystems

truck.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # contact scene needs BULLET

print("VEHICLE MASS: ", truck.GetTractor().GetMass())               # Kraz mass via tractor

vis_type = veh.VisualizationType_MESH                                # mesh visualization
truck.SetChassisVisualizationType(vis_type, vis_type)               # tractor + trailer chassis
truck.SetSuspensionVisualizationType(vis_type, vis_type)            # suspensions
truck.SetSteeringVisualizationType(vis_type)                        # steering
truck.SetWheelVisualizationType(vis_type, vis_type)                # wheels
truck.SetTireVisualizationType(vis_type, vis_type)                 # tires

# Second vehicle (BMW E90 sedan) shares the truck's system
sedan = veh.BMW_E90(truck.GetSystem())                              # shared system -> not orphan
sedan.SetChassisFixed(False)                                        # sedan free to move
sedan.SetInitPosition(chrono.ChCoordsysd(sedanLoc, sedanRot))       # sedan spawn pose
sedan.SetTireType(veh.TireModelType_TMEASY)                         # sedan tire model
sedan.SetTireStepSize(tire_step_size)                              # sedan tire step
sedan.Initialize()                                                  # build sedan subsystems
sedan.SetChassisVisualizationType(vis_type)                         # sedan chassis vis
sedan.SetSuspensionVisualizationType(vis_type)                      # sedan suspension vis
sedan.SetSteeringVisualizationType(vis_type)                        # sedan steering vis
sedan.SetWheelVisualizationType(vis_type)                          # sedan wheel vis
sedan.SetTireVisualizationType(vis_type)                           # sedan tire vis

terrain = veh.RigidTerrain(truck.GetSystem())                      # rigid terrain on the shared system
patch_mat = chrono.ChContactMaterialNSC()                          # NSC contact material
patch_mat.SetFriction(0.9)                                          # terrain friction
patch_mat.SetRestitution(0.01)                                     # terrain restitution
# Predefined highway mesh as the rigid terrain patch
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM,
                         veh.GetDataFile("terrain/meshes/Highway_col.obj"))   # highway collision mesh
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)    # patch texture
terrain.Initialize()                                               # finalize terrain

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                   # vehicle-specific Irrlicht vis
vis.SetWindowTitle('Kraz Demo')                                    # window title
vis.SetWindowSize(1280, 1024)                                      # window size
vis.SetChaseCamera(chrono.ChVector3d(3, 0, 2.1), 25.0, 10.5)       # trackPoint, chase dist, height
vis.Initialize()                                                   # Irrlicht init FIRST
vis.AddLogo()                                                      # logo
vis.AddLightDirectional()                                          # directional light
vis.AddSkyBox()                                                    # sky box
vis.AttachVehicle(truck.GetTractor())                              # attach tractor (Kraz)

# Truck driver: rigid-tire truck driving forward with mild steering
truck_data = veh.vector_Entry([
    veh.DataDriverEntry(0.0, 0.0, 0.0, 0.0),                       # start at rest
    veh.DataDriverEntry(0.5, 0.0, 0.6, 0.0),                       # accelerate
    veh.DataDriverEntry(20.0, 0.0, 0.6, 0.0),                      # cruise straight
])
truck_driver = veh.ChDataDriver(truck.GetTractor(), truck_data)     # scripted truck driver on tractor
truck_driver.Initialize()                                          # init truck driver

# Sedan driver: move forward with fixed throttle and zero steering
sedan_data = veh.vector_Entry([
    veh.DataDriverEntry(0.0, 0.0, 0.5, 0.0),                       # fixed throttle, no steer
    veh.DataDriverEntry(20.0, 0.0, 0.5, 0.0),                      # hold
])
sedan_driver = veh.ChDataDriver(sedan.GetVehicle(), sedan_data)     # scripted sedan driver
sedan_driver.Initialize()                                          # init sedan driver

render_steps = math.ceil(render_step_size / step_size)            # physics steps per rendered frame
realtime_timer = chrono.ChRealtimeStepTimer()                     # wall-clock pacing
step_number = 0                                                   # step counter
log_info = True                                                   # fire the state log once


while vis.Run():
    time = truck.GetSystem().GetChTime()                          # current sim time

    if step_number % render_steps == 0:                           # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    truck_inputs = truck_driver.GetInputs()                       # truck driver command
    sedan_inputs = sedan_driver.GetInputs()                       # sedan driver command

    truck_driver.Synchronize(time)                                # sync truck driver
    sedan_driver.Synchronize(time)                                # sync sedan driver
    terrain.Synchronize(time)                                     # sync terrain
    truck.Synchronize(time, truck_inputs, terrain)               # sync truck
    sedan.Synchronize(time, sedan_inputs, terrain)               # sync sedan
    vis.Synchronize(time, truck_inputs)                           # sync vis

    truck_driver.Advance(step_size)                               # advance truck driver
    sedan_driver.Advance(step_size)                               # advance sedan driver
    terrain.Advance(step_size)                                    # advance terrain
    truck.Advance(step_size)                                      # advance truck (steps the system)
    sedan.Advance(step_size)                                      # advance sedan
    vis.Advance(step_size)                                        # advance vis

    if log_info and time > 1:                                     # store truck tractor + trailer state once
        tractor_pos = truck.GetTractor().GetPos()                 # tractor position
        trailer_pos = truck.GetTrailer().GetChassis().GetPos()    # trailer chassis position
        print("t = ", time)
        print("     tractor ", tractor_pos.x, "  ", tractor_pos.y)
        print("     trailer ", trailer_pos.x, "  ", trailer_pos.y)
        log_info = False                                          # disable further logging

    step_number += 1                                              # increment step
    realtime_timer.Spin(step_size)                                # pace to real time
