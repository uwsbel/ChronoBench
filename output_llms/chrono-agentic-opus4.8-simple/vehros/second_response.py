import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.ros as chros
import pychrono.irrlicht as chronoirr


def main():
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())              # locate bundled Chrono assets
    veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')          # locate vehicle data files

    init_loc = chrono.ChVector3d(0, 0, 0.5)                           # chassis origin spawn (HMMWV center)
    init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                       # identity orientation
    step_size = 1e-3                                                  # integration step (s)
    tire_step_size = step_size                                       # tire substep matches step

    hmmwv = veh.HMMWV_Full()                                          # full HMMWV catalog wrapper
    hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)               # NSC for rigid terrain
    hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)           # no chassis collision mesh
    hmmwv.SetChassisFixed(False)                                     # MANDATORY — fixed chassis won't move
    hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))   # place chassis in world
    hmmwv.SetTireType(veh.TireModelType_TMEASY)                     # TMEASY tire on rigid terrain
    hmmwv.SetTireStepSize(tire_step_size)                          # tire integration substep
    hmmwv.Initialize()                                              # build the vehicle

    hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)        # chassis mesh
    hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)  # suspension prims
    hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)    # steering prims
    hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)             # wheel mesh
    hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)              # tire mesh

    system = hmmwv.GetSystem()                                       # take the wrapper-owned system
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED, after Initialize
    print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())          # report total vehicle mass

    terrain = veh.RigidTerrain(system)                              # rigid terrain on the shared system
    patch_mat = chrono.ChContactMaterialNSC()                      # NSC contact material for the patch
    patch_mat.SetFriction(0.9)                                     # ground friction
    patch_mat.SetRestitution(0.01)                                # ground restitution
    patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 100.0, 100.0)  # flat 100x100 m patch
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 100, 100)  # tile texture
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                 # patch color
    terrain.Initialize()                                          # finalize terrain

    driver = veh.ChDriver(hmmwv.GetVehicle())                     # base driver, inputs set over ROS
    driver.Initialize()                                          # initialize the driver

    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()             # vehicle-specific Irrlicht window
    vis.SetWindowTitle("HMMWV over ROS")                        # window title
    vis.SetWindowSize(1280, 1024)                              # window size (px)
    vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)  # chase camera on chassis
    vis.Initialize()                                           # build the device first
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))  # logo overlay
    vis.AddSkyBox()                                            # sky box
    vis.AddLightDirectional()                                 # directional light (vehicle truth)
    vis.AttachVehicle(hmmwv.GetVehicle())                     # bind chassis/wheel/tire visuals

    ros_manager = chros.ChROSPythonManager()                  # ROS2 manager (python handlers)
    ros_manager.RegisterHandler(chros.ChROSClockHandler())   # /clock first — sync sim time
    driver_handler = chros.ChROSDriverInputsHandler(25, driver, "~/input/driver_inputs")  # subscribe throttle/steer/brake
    ros_manager.RegisterHandler(driver_handler)              # register driver-inputs handler
    body_handler = chros.ChROSBodyHandler(25, hmmwv.GetChassisBody(), "~/output/hmmwv/state")  # publish chassis pose/twist
    ros_manager.RegisterHandler(body_handler)               # register body handler
    ros_manager.Initialize()                                # initialize ONCE after all handlers

    sim_end = 30.0                                          # simulation duration (s)
    render_fps = 50.0                                      # target render rate
    render_every = max(1, round(1.0 / (render_fps * step_size)))  # untagged cadence constant


    while vis.Run() and system.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            time = system.GetChTime()                      # current sim time
            driver_inputs = driver.GetInputs()             # inputs (driven over ROS)

            driver.Synchronize(time)                       # sync driver
            terrain.Synchronize(time)                      # sync terrain
            hmmwv.Synchronize(time, driver_inputs, terrain)  # sync vehicle
            vis.Synchronize(time, driver_inputs)           # sync visualization

            driver.Advance(step_size)                      # advance driver
            terrain.Advance(step_size)                     # advance terrain
            hmmwv.Advance(step_size)                       # advances the wrapper-owned system
            vis.Advance(step_size)                         # advance visualization


            if not ros_manager.Update(time, step_size):    # ROS update LAST — publish/subscribe
                break

            if system.GetChTime() >= sim_end:
                break


if __name__ == "__main__":
    main()
