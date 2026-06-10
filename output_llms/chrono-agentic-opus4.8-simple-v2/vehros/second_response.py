import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.ros as chros
from pychrono import irrlicht as chronoirr


def main():
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())             # locate bundled Chrono assets
    veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')         # locate vehicle data files

    init_loc = chrono.ChVector3d(0, 0, 0.5)                          # chassis spawn above the road
    init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                      # no initial yaw
    step_size = 1e-3                                                 # physics step
    tire_step_size = 1e-3                                            # tire integration step

    hmmwv = veh.HMMWV_Full()                                         # full HMMWV catalog wrapper
    hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)               # NSC for rigid terrain
    hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)           # no chassis collision shape
    hmmwv.SetChassisFixed(False)                                     # MANDATORY — fixed chassis won't move
    hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))   # spawn pose
    hmmwv.SetTireType(veh.TireModelType_TMEASY)                      # TMEASY tire force model
    hmmwv.SetTireStepSize(tire_step_size)                           # tire substep
    hmmwv.Initialize()                                              # build the vehicle

    hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)          # mesh chassis
    hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES) # primitive suspension
    hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)   # primitive steering
    hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)            # mesh wheels
    hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)             # mesh tires

    system = hmmwv.GetSystem()                                      # wrapper-owned system
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED for terrain contact
    print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())          # report total vehicle mass

    terrain = veh.RigidTerrain(system)                             # rigid ground
    patch_mat = chrono.ChContactMaterialNSC()                      # NSC patch material
    patch_mat.SetFriction(0.9)                                     # tire grip
    patch_mat.SetRestitution(0.01)                                 # nearly inelastic
    patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 200.0, 200.0)  # flat 200x200 m patch
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                  # sandy color
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)  # tiled road texture
    terrain.Initialize()                                          # build the terrain

    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()              # vehicle-specific Irrlicht window
    vis.SetWindowTitle("HMMWV ROS Demo")                          # window title
    vis.SetWindowSize(1280, 1024)                                 # window size
    vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)   # follow the chassis
    vis.Initialize()                                             # build the device FIRST
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))  # logo overlay
    vis.AddSkyBox()                                              # sky box
    vis.AddLightDirectional()                                    # vehicle demos use a directional light
    vis.AttachVehicle(hmmwv.GetVehicle())                        # bind chassis/wheel/tire visuals

    driver = veh.ChInteractiveDriverIRR(vis)                     # interactive driver bound to the window
    steering_time = 1.0                                          # s to reach full steering
    throttle_time = 1.0                                          # s to reach full throttle
    braking_time = 0.3                                           # s to reach full brake
    render_step_size = 1.0 / 50.0                                # 50 FPS render cadence
    driver.SetSteeringDelta(render_step_size / steering_time)   # steering rate
    driver.SetThrottleDelta(render_step_size / throttle_time)   # throttle rate
    driver.SetBrakingDelta(render_step_size / braking_time)     # braking rate
    driver.Initialize()                                         # build the driver

    hmmwv.GetChassisBody().SetName("chassis")                   # name the chassis body for ROS topics

    ros_manager = chros.ChROSPythonManager()                    # ROS2 bridge manager
    ros_manager.RegisterHandler(chros.ChROSClockHandler())      # /clock FIRST — time sync
    driver_inputs_handler = chros.ChROSDriverInputsHandler(     # SUBSCRIBES throttle/steer/brake over ROS
        25, driver, "~/input/driver_inputs")
    ros_manager.RegisterHandler(driver_inputs_handler)         # register driver-inputs handler
    body_handler = chros.ChROSBodyHandler(                     # publishes chassis pose/twist
        25, hmmwv.GetChassisBody(), "~/output/hmmwv/state")
    ros_manager.RegisterHandler(body_handler)                 # register body handler
    ros_manager.Initialize()                                  # ONCE, after all handlers

    sim_end = 20.0                                            # simulation duration (s)
    render_steps = math.ceil(render_step_size / step_size)   # physics steps per rendered frame

    realtime_timer = chrono.ChRealtimeStepTimer()            # wall-clock pacing
    step_number = 0                                          # physics step counter
    while vis.Run() and system.GetChTime() < sim_end:
        time = system.GetChTime()                           # current sim time

        if step_number % render_steps == 0:                 # throttled rendering
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()                  # current driver command (ROS may overwrite)

        driver.Synchronize(time)                            # update driver
        terrain.Synchronize(time)                           # update terrain
        hmmwv.Synchronize(time, driver_inputs, terrain)     # feed inputs to the vehicle
        vis.Synchronize(time, driver_inputs)                # update the HUD

        driver.Advance(step_size)                           # advance driver
        terrain.Advance(step_size)                          # advance terrain
        hmmwv.Advance(step_size)                            # advances the wrapper-owned system
        vis.Advance(step_size)                              # advance visualization


        if not ros_manager.Update(time, step_size):         # publish/subscribe ROS state LAST
            break

        step_number += 1                                    # advance step counter
        realtime_timer.Spin(step_size)                      # spin so wall-clock matches sim time


if __name__ == "__main__":
    main()
