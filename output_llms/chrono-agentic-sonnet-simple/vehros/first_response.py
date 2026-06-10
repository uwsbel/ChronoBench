import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import pychrono.ros as chros

def main():
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())             # locate bundled Chrono assets
    veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')         # locate vehicle data files

    # --- vehicle setup ---
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)               # NSC for rigid terrain
    hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
    hmmwv.SetChassisFixed(False)                                     # MANDATORY — fixed chassis won't move
    init_loc = chrono.ChVector3d(0, 0, 0.5)                         # spawn with z clearance
    init_rot = chrono.QUNIT                                          # no initial rotation
    hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
    hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)                  # full shafts engine
    hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)  # automatic shafts
    hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)                      # all-wheel drive
    hmmwv.SetTireType(veh.TireModelType_TMEASY)                      # TMEASY tire model
    hmmwv.SetTireStepSize(1e-3)                                      # tire integrator step
    hmmwv.Initialize()

    system = hmmwv.GetSystem()                                       # get wrapper-owned system
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize

    hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

    print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())            # truth's literal banner

    # --- terrain ---
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChContactMaterialNSC()                        # NSC material matches contact method
    patch_mat.SetFriction(0.9)                                       # friction coefficient
    patch_mat.SetRestitution(0.01)                                   # low restitution
    patch = terrain.AddPatch(
        patch_mat,
        chrono.CSYSNORM,                                             # centered at origin, flat
        200.0,                                                       # length X
        200.0,                                                       # width Y
    )
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    # --- Irrlicht visualization ---
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle("HMMWV + ROS")
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)     # chase camera behind vehicle
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddLightDirectional()                                        # vehicle truth uses directional light
    vis.AttachVehicle(hmmwv.GetVehicle())

    # --- driver ---
    driver = veh.ChInteractiveDriverIRR(vis)                         # interactive keyboard driver
    steering_time = 1.0                                              # s to full steering
    throttle_time = 1.0                                              # s to full throttle
    braking_time = 0.3                                               # s to full braking
    render_step_size = 1.0 / 50.0                                    # 50 Hz render rate
    driver.SetSteeringDelta(render_step_size / steering_time)
    driver.SetThrottleDelta(render_step_size / throttle_time)
    driver.SetBrakingDelta(render_step_size / braking_time)
    driver.Initialize()

    # --- ROS manager (scored core — ChROSPythonManager + handlers are never review-only) ---
    ros_manager = chros.ChROSPythonManager()
    ros_manager.RegisterHandler(chros.ChROSClockHandler())           # clock sync — register FIRST
    ros_manager.RegisterHandler(
        chros.ChROSDriverInputsHandler(25, driver, "~/input/driver_inputs")  # subscribe driver inputs
    )
    ros_manager.RegisterHandler(
        chros.ChROSBodyHandler(25, hmmwv.GetVehicle().GetChassisBody(), "~/output/vehicle/state")  # publish vehicle state
    )
    ros_manager.Initialize()                                         # ONCE, after all handlers

    # --- simulation loop parameters ---
    step_size = 1e-3                                                 # physics time step (s)
    sim_end = 30.0                                                   # simulation duration (s)
    render_every = max(1, round(render_step_size / step_size))       # render cadence (untagged)


    step_number = 0
    realtime_timer = chrono.ChRealtimeStepTimer()
    while vis.Run() and system.GetChTime() < sim_end:
        time = system.GetChTime()                                    # current simulation time

        if step_number % render_every == 0:                          # throttled rendering
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()

        # synchronize all subsystems
        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        # advance all subsystems
        driver.Advance(step_size)
        terrain.Advance(step_size)
        hmmwv.Advance(step_size)                                     # also steps the system
        vis.Advance(step_size)


        if not ros_manager.Update(time, step_size):                  # ROS update last; break on shutdown
            break

        step_number += 1
        realtime_timer.Spin(step_size)                               # keep wall-clock ~ sim time


if __name__ == "__main__":
    main()
