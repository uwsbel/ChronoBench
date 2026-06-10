import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import pychrono.ros as chros

def main():
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())           # locate bundled Chrono assets
    veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')       # locate vehicle data files

    step_size = 1e-3                # physics time step (s)
    sim_end = 20.0                  # simulation duration (s)
    render_fps = 50.0               # rendering frame rate

    # HMMWV vehicle setup
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)             # NSC for rigid terrain
    hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
    hmmwv.SetChassisFixed(False)                                   # MANDATORY - fixed chassis won't move
    hmmwv.SetInitPosition(chrono.ChCoordsysd(
        chrono.ChVector3d(0, 0, 0.5), chrono.QUNIT))               # spawn above terrain
    hmmwv.SetTireType(veh.TireModelType_TMEASY)                    # TMEASY for rigid terrain
    hmmwv.SetTireStepSize(step_size)
    hmmwv.Initialize()

    system = hmmwv.GetSystem()                                     # get wrapper-owned system
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED for contact scenes

    print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())          # truth's literal banner

    # Visualization types - set after Initialize() (VisualizationType lives in veh namespace in 9.0.0)
    hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

    # Rigid terrain setup
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChContactMaterialNSC()                      # NSC material for rigid terrain
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 200.0, 200.0)  # flat terrain 200x200 m
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 100, 100)  # tile texture
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    # Irrlicht visualization (vehicle-specific visual system)
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle("HMMWV ROS Demo")
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)   # chase camera behind vehicle
    vis.Initialize()                                                # MUST be called FIRST
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddLightDirectional()                                       # vehicle truths use directional light
    vis.AttachVehicle(hmmwv.GetVehicle())

    # Interactive driver (scored-core default for catalog vehicles)
    driver = veh.ChInteractiveDriverIRR(vis)
    steering_time = 1.0            # seconds to max steering
    throttle_time = 1.0            # seconds to max throttle
    braking_time = 0.3             # seconds to max braking
    render_step_size = 1.0 / render_fps
    driver.SetSteeringDelta(render_step_size / steering_time)
    driver.SetThrottleDelta(render_step_size / throttle_time)
    driver.SetBrakingDelta(render_step_size / braking_time)
    driver.Initialize()

    # ROS manager setup - ChROSPythonManager for Python handler support
    ros_manager = chros.ChROSPythonManager()
    ros_manager.RegisterHandler(chros.ChROSClockHandler())         # clock FIRST - time-syncs ROS graph
    ros_manager.RegisterHandler(chros.ChROSDriverInputsHandler(25, driver, "~/input/driver_inputs"))  # subscribe driver inputs
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(25, hmmwv.GetChassisBody(), "~/output/vehicle/state"))  # publish chassis state
    ros_manager.Initialize()                                       # ONCE, after all handlers registered

    render_every = max(1, round(1.0 / (render_fps * step_size)))   # untagged cadence constant
    step_number = 0
    realtime_timer = chrono.ChRealtimeStepTimer()

    while vis.Run():
        time = system.GetChTime()

        if step_number % render_every == 0:                        # throttled rendering
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()

        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)            # vehicle sync with terrain
        vis.Synchronize(time, driver_inputs)


        driver.Advance(step_size)
        terrain.Advance(step_size)
        hmmwv.Advance(step_size)                                   # advances wrapper-owned system
        vis.Advance(step_size)

        if not ros_manager.Update(time, step_size):                # ROS update LAST; break on shutdown
            break

        step_number += 1
        realtime_timer.Spin(step_size)                             # keep wall-clock ~ sim time


if __name__ == "__main__":
    main()
