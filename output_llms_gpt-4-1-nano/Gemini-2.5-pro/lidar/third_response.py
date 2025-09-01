import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.vehicle as veh

import numpy as np
import time
import os # Added for data path handling

# -----------------
# Global Parameters
# -----------------

# Path to Chrono data files (textures, meshes, etc.)
# Try to use environment variables, otherwise specify path directly
CHRONO_DATA_DIR = os.environ.get('CHRONO_DATA_DIR', '../../../chrono_data/')
CHRONO_VEHICLE_DATA_DIR = os.environ.get('CHRONO_VEHICLE_DATA_DIR', '../../../chrono_vehicle_data/')

# Lidar parameters
noise_model = "NONE"  # "CONST_NORMAL_XYZI" or "NONE"
return_mode = sens.LidarReturnMode_STRONGEST_RETURN
update_rate = 5.0
horizontal_samples = 800
vertical_samples_3d = 30 # Reduced for performance, original was 300
horizontal_fov = 2 * chrono.CH_PI  # 360 degrees
max_vert_angle = chrono.CH_PI / 12.0
min_vert_angle = -chrono.CH_PI / 6.0
lag = 0.0
collection_time = 1.0 / update_rate
sample_radius = 1 # Reduced for performance, original was 2
divergence_angle = 0.003

# Camera parameters (for new third-person camera)
camera_update_rate = 30.0
camera_fov = chrono.CH_PI / 3.0
camera_width = 1280
camera_height = 720

# Simulation parameters
step_size = 1e-3
end_time = 20.0 # Reduced for quicker testing, original was 40.0

# Visualization/Save options
vis = True # Enable/disable sensor visualization windows
# save = False # Unused in this script version
# out_dir = "SENSOR_OUTPUT/" # Unused in this script version

def main():
    # -----------------
    # Set data paths
    # -----------------
    chrono.SetChronoDataPath(CHRONO_DATA_DIR)
    veh.SetDataPath(CHRONO_VEHICLE_DATA_DIR)

    # ---------------------
    # Create ARTcar Vehicle
    # ---------------------
    my_vehicle = veh.ARTcar_Vehicle()
    my_vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
    my_vehicle.SetChassisFixed(False)
    # art_vehicle.SetChassisCollisionType(veh.ChassisCollisionType_NONE) # Optional: manage chassis collision
    my_vehicle.SetEngineType(veh.EngineModelType_SHAFTS)
    my_vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
    my_vehicle.SetBrakeType(veh.BrakeType_SHAFTS) # Ensure brake type is set
    my_vehicle.SetTireType(veh.TireModelType_TMEASY) # TMeasy is a good default
    my_vehicle.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, -2, 0.5), chrono.ChQuaterniond(1, 0, 0, 0))) # Initial position
    my_vehicle.Initialize()

    # Set visualization for vehicle components
    my_vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
    my_vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    my_vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    my_vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
    my_vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)

    mphysicalSystem = my_vehicle.GetSystem()
    mphysicalSystem.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))


    # ------------------
    # Create the driver
    # ------------------
    # Simple data driver: move forward slowly
    driver_data = veh.vector_ChDataDriverEntry()
    driver_data.push_back(veh.ChDataDriverEntry(0.0, 0.0, 0.2, 0.0))  # time, steering, throttle, braking
    driver_data.push_back(veh.ChDataDriverEntry(end_time, 0.0, 0.2, 0.0)) # Maintain throttle
    my_driver = veh.ChDataDriver(my_vehicle.GetVehicle(), driver_data)
    my_driver.Initialize()
    
    # ------------------
    # Create the terrain
    # ------------------
    terrain = veh.RigidTerrain(mphysicalSystem)
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    # Terrain patch at z=0, large enough
    patch = terrain.AddPatch(patch_mat, chrono.ChVector3d(0, 0, -0.1), chrono.ChVector3d(0,0,1), 200.0, 200.0, 0.1) # center, normal, size_x, size_y, thickness
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200) # texture file, repeat U, repeat V
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()


    # ----------------------------------
    # Add a mesh to be sensed by sensors
    # ----------------------------------
    side = 4.0
    box = chrono.ChBodyEasyBox(side, side, side, 1000) # density 1000
    box.SetPos(chrono.ChVector3d(10, 0, side / 2.0)) # Position it away from vehicle start, on the terrain
    # Corrected texture setting for ChVisualModel shapes
    if box.GetVisualModel() and len(box.GetVisualModel().GetShapes()) > 0:
        try:
            box.GetVisualModel().GetShapes()[0].SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
        except Exception as e:
            print(f"Warning: Could not set texture for box: {e}")
    else:
        # Fallback or alternative if GetVisualModel() is not set up as expected by EasyBox
        # For ChBodyEasyBox, visual shape is usually added. If specific material properties are needed:
        mat = chrono.ChVisualMaterial()
        mat.SetDiffuseColor(chrono.ChColor(0.2, 0.3, 0.8)) # Blueish
        mat.SetKdTexture(chrono.GetChronoDataFile("textures/blue.png"))
        if box.GetVisualModel() and len(box.GetVisualModel().GetShapes()) > 0:
             box.GetVisualModel().GetShapes()[0].SetMaterial(0,mat)


    box.SetFixed(True)
    mphysicalSystem.Add(box)

    # -----------------------
    # Create a sensor manager
    # -----------------------
    manager = sens.ChSensorManager(mphysicalSystem)
    intensity = 1.0 # Set a default intensity for the sensor manager's rendering
    manager.scene.AddPointLight(chrono.ChVector3d(100, 100, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddPointLight(chrono.ChVector3d(-100, 100, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddPointLight(chrono.ChVector3d(100, -100, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddPointLight(chrono.ChVector3d(-100, -100, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)


    # ------------------------------------------------
    # Create a 3D lidar and add it to the sensor manager
    # ------------------------------------------------
    # Modified offset pose for vehicle attachment
    lidar_offset_pose = chrono.ChFramed(
        chrono.ChVector3d(1.0, 0.0, 1.0), # x, y, z offset from chassis center
        chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )

    lidar_3d = sens.ChLidarSensor(
        my_vehicle.GetChassisBody(), # Attached to vehicle chassis
        update_rate,
        lidar_offset_pose,
        horizontal_samples,
        vertical_samples_3d, # Use specific vertical samples for 3D
        horizontal_fov,
        max_vert_angle,
        min_vert_angle,
        100.0,                          # Max range
        sens.LidarBeamShape_RECTANGULAR,
        sample_radius,
        divergence_angle,
        divergence_angle,
        return_mode
    )
    lidar_3d.SetName("3D Lidar Sensor")
    lidar_3d.SetLag(lag)
    lidar_3d.SetCollectionWindow(collection_time)

    if noise_model == "CONST_NORMAL_XYZI":
        lidar_3d.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    if vis:
        lidar_3d.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples_3d, "Raw 3D Lidar Depth Data"))
    lidar_3d.PushFilter(sens.ChFilterDIAccess())
    lidar_3d.PushFilter(sens.ChFilterPCfromDepth())
    if vis:
        lidar_3d.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "3D Lidar Point Cloud"))
    lidar_3d.PushFilter(sens.ChFilterXYZIAccess())
    manager.AddSensor(lidar_3d)

    # ------------------------------------------------
    # Create a 2D lidar and add it to the sensor manager
    # ------------------------------------------------
    # Using the same modified offset pose
    lidar_2d = sens.ChLidarSensor(
        my_vehicle.GetChassisBody(), # Attached to vehicle chassis
        update_rate,
        lidar_offset_pose, # Re-use or define new if different position needed
        horizontal_samples,
        1,                              # 1 vertical channel for 2D lidar
        horizontal_fov,
        0.0,                            # Max vertical angle (flat plane)
        0.0,                            # Min vertical angle (flat plane)
        100.0,                          # Max range
        sens.LidarBeamShape_RECTANGULAR,
        sample_radius,
        divergence_angle,
        divergence_angle,
        return_mode
    )
    lidar_2d.SetName("2D Lidar Sensor")
    lidar_2d.SetLag(lag)
    lidar_2d.SetCollectionWindow(collection_time)

    if noise_model == "CONST_NORMAL_XYZI":
        lidar_2d.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    if vis:
        # Corrected vertical samples for 2D lidar visualization
        lidar_2d.PushFilter(sens.ChFilterVisualize(horizontal_samples, 1, "Raw 2D Lidar Depth Data"))
    lidar_2d.PushFilter(sens.ChFilterDIAccess())
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())
    if vis: # Added point cloud visualization for 2D lidar as well
        lidar_2d.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 2.0, "2D Lidar Point Cloud"))
    lidar_2d.PushFilter(sens.ChFilterXYZIAccess())
    manager.AddSensor(lidar_2d)

    # ----------------------------------------------------
    # Create a Third Person Camera and add to manager
    # ----------------------------------------------------
    camera_offset_pose = chrono.ChFramed(
        chrono.ChVector3d(-6.0, 0.0, 2.0), # Behind, centered, above chassis
        chrono.QuatFromAngleAxis(chrono.CH_PI / 20.0, chrono.ChVector3d(0, 1, 0)) # Slight downward pitch
    )
    camera = sens.ChCameraSensor(
        my_vehicle.GetChassisBody(), # Attached to vehicle chassis
        camera_update_rate,
        camera_offset_pose,
        camera_width,
        camera_height,
        camera_fov
    )
    camera.SetName("Third Person Camera")
    camera.SetLag(0) # lag
    camera.SetCollectionWindow(0) # collection window
    if vis:
        camera.PushFilter(sens.ChFilterVisualize(camera_width, camera_height, "Third Person Camera View"))
    camera.PushFilter(sens.ChFilterRGBA8Access())
    manager.AddSensor(camera)
    
    # ---------------
    # Simulate system
    # ---------------
    ch_time = 0.0
    t1 = time.time()

    while ch_time < end_time:
        # Get driver inputs
        driver_inputs = my_driver.GetInputs()

        # Synchronize and Advance systems
        current_time = mphysicalSystem.GetChTime()
        my_driver.Synchronize(current_time)
        my_vehicle.Synchronize(current_time, driver_inputs.m_steering, driver_inputs.m_throttle, driver_inputs.m_braking, terrain)
        terrain.Synchronize(current_time)
        
        my_driver.Advance(step_size)
        my_vehicle.Advance(step_size)
        terrain.Advance(step_size)

        # Update sensor manager (will render/save/filter automatically)
        manager.Update()

        # Perform step of dynamics
        mphysicalSystem.DoStepDynamics(step_size)

        # Get the current time of the simulation
        ch_time = mphysicalSystem.GetChTime()

        # Optional: Access and print lidar data (example for 3D lidar)
        if lidar_3d.GetMostRecentXYZIBuffer().HasData():
            xyzi_buffer = lidar_3d.GetMostRecentXYZIBuffer()
            # xyzi_data = xyzi_buffer.GetXYZIData() # This can be large, print sparingly
            # print(f'Time {ch_time:.2f}: 3D Lidar XYZI buffer received. Resolution: {xyzi_buffer.Width}x{xyzi_buffer.Height}')
            # print(f'Max Value: {np.max(xyzi_data) if xyzi_data.size > 0 else "N/A"}')
            pass


    print("Sim time:", end_time, "Wall time:", time.time() - t1)

if __name__ == "__main__":
    main()