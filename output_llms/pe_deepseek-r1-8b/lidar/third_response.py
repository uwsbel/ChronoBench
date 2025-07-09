import pychrono.core as chrono
import pychrono.sensor as sens

import numpy as np
import time
import math

def main():
    # -----------------
    # Define simulation parameters
    # -----------------
    step_size = 1e-3
    end_time = 40.0
    update_rate = 5.0
    horizontal_samples = 800
    vertical_samples = 300
    horizontal_fov = 2 * chrono.CH_PI  # 360 degrees
    max_vert_angle = chrono.CH_PI / 12
    min_vert_angle = -chrono.CH_PI / 6
    sample_radius = 2
    divergence_angle = 0.003
    return_mode = sens.LidarReturnMode_STRONGEST_RETURN
    lag = 0
    collection_time = 1. / update_rate

    # -----------------
    # Initialize the system with an ARTcar vehicle
    # -----------------
    vehicle = chrono.veh.ARTcar()
    vehicle.SetContactMethod(chrono.ChContactMaterialNSC())
    vehicle.SetChassisCollisionType(chrono.ChCollisionType.DYNAMIC)
    vehicle.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI / 2)))
    vehicle.Initialize()
    mphysicalSystem = vehicle.GetSystem()
    mphysicalSystem.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    # -----------------
    # Add a box as part of the vehicle's chassis
    # -----------------
    side = 4
    box = chrono.ChBodyEasyBox(side, side, side, 1000)
    box.SetPos(chrono.ChVector3d(0, 0, 0))
    box.GetVisualModel().GetShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
    box.SetFixed(True)
    vehicle.Add(box)

    # -----------------
    # Create and attach 3D lidar sensor to the vehicle
    # -----------------
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(1.0, 0, 1),  # Changed from (-12, 0, 1) to (1.0, 0, 1)
        chrono.QuatFromAngleAxis(ch_time * orbit_rate, chrono.ChVector3d(0, 0, 1))
    )
    lidar = sens.ChLidarSensor(
        vehicle,  # Lidar is attached to the vehicle
        update_rate,
        offset_pose,
        horizontal_samples,
        vertical_samples,
        horizontal_fov,
        max_vert_angle,
        min_vert_angle,
        100.0,
        sens.LidarBeamShape_RECTANGULAR,
        sample_radius,
        divergence_angle,
        divergence_angle,
        return_mode
    )
    lidar.SetName("3D Lidar Sensor")
    lidar.SetLag(lag)
    lidar.SetCollectionWindow(collection_time)
    
    # -----------------
    # Add visualizations
    # -----------------
    vis = True
    vis_system = chronoirr.ChVisualSystemIrrlicht()
    vis_system.AttachSystem(mphysicalSystem)
    vis_system.SetWindowSize(1024, 768)
    vis_system.SetWindowTitle('Simulation Window')
    vis_system.Initialize()
    vis_system.AddSkyBox()
    vis_system.AddCamera(chrono.ChVector3d(0, 3, 6))
    vis_system.AddTypicalLights()

    # -----------------
    # Create third person camera sensor
    # -----------------
    camera_offset = chrono.ChFramed(
        chrono.ChVector3d(5, 5, 10),
        chrono.QuatFromAngleX(chrono.CH_PI / 2)
    )
    camera = sens.ChSensorCamera(
        vehicle,  # Camera is attached to the vehicle
        60,  # Field of view
        2560,  # Width
        1440,  # Height
        camera_offset,
        45,  # View angle
        1.0  # Near plane
    )
    camera.SetName("Third Person Camera")
    mphysicalSystem.Add(camera)

    # -----------------
    # Create sensor manager
    # -----------------
    manager = sens.ChSensorManager(mphysicalSystem)
    manager.AddSensor(lidar)
    manager.AddSensor(camera)

    # -----------------
    # Simulate system
    # -----------------
    orbit_radius = 10
    orbit_rate = 0.1

    render_time = 0
    t1 = time.time()

    while ch_time < end_time:
        # Set lidar to orbit around the vehicle
        lidar.SetOffsetPose(
            chrono.ChFramed(
                chrono.ChVector3d(
                    -orbit_radius * math.cos(ch_time * orbit_rate),
                    -orbit_radius * math.sin(ch_time * orbit_rate),
                    1
                ),
                chrono.QuatFromAngleAxis(ch_time * orbit_rate, chrono.ChVector3d(0, 0, 1))
            )
        )

        # Access the XYZI buffer from lidar
        xyzi_buffer = lidar.GetMostRecentXYZIBuffer()
        if xyzi_buffer.HasData():
            xyzi_data = xyzi_buffer.GetXYZIData()
            print('XYZI buffer received from lidar. Lidar resolution: {0}x{1}'.format(xyzi_buffer.Width, xyzi_buffer.Height))
            print('Max Value: {0}'.format(np.max(xyzi_data)))

        # Update sensor manager (will render/save/filter automatically)
        manager.Update()

        # Perform step of dynamics
        mphysicalSystem.DoStepDynamics(step_size)

        # Get the current time of the simulation
        ch_time = mphysicalSystem.GetChTime()

        # Update vehicle, driver, and terrain
        vehicle.Update()
        vehicle.GetDriver().Update()

    print("Sim time:", end_time, "Wall time:", time.time() - t1)

# -----------------
# Run main function
# -----------------
if __name__ == '__main__':
    main()