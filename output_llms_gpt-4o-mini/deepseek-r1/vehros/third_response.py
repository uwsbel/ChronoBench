import pychrono as ch
import pychrono.vehicle as veh
import pychrono.ros as chros
import pychrono.sensor as sens  # Instruction 1: Add sensor import
from pychrono import irrlicht as chronoirr
import math

def main():
    veh.SetDataPath(ch.GetChronoDataPath() + 'vehicle/')
    
    # Create the HMMWV vehicle and set its parameters
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(ch.ChContactMethod_NSC)
    hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
    hmmwv.SetChassisFixed(False)
    hmmwv.SetInitPosition(ch.ChCoordsysd(ch.ChVector3d(0, 0, 1.6), ch.ChQuaterniond(1, 0, 0, 0)))
    hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
    hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
    hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
    hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
    hmmwv.SetTireType(veh.TireModelType_TMEASY)
    hmmwv.SetTireStepSize(1e-3)
    hmmwv.Initialize()
    hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetSteeringVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

    # Create terrain
    terrain = veh.RigidTerrain(hmmwv.GetSystem())
    patch_mat = ch.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch = terrain.AddPatch(patch_mat, ch.CSYSNORM, 100.0, 100.0)
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 100, 100)
    terrain.Initialize()

    # Instruction 2: Add visualization box
    box = ch.ChBodyEasyBox(2, 2, 2, 1000, True, True)
    box.SetPos(ch.ChVector3d(5, 0, 1))
    box.SetFixed(True)
    box.SetVisualMaterial(ch.ChVisualMaterial().SetKdTexture(ch.GetChronoDataFile("textures/concrete.jpg")))
    hmmwv.GetSystem().Add(box)

    # Create run-time visualization
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(hmmwv.GetSystem())
    vis.SetCameraVertical(ch.CameraVerticalDir_Z)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle('HMMWV Simulation with Sensors')
    vis.Initialize()
    vis.AddLogo(ch.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    # Instruction 7: Update camera position
    vis.AddCamera(ch.ChVector3d(-5, 2.5, 1.5), ch.ChVector3d(0, 0, 1))
    vis.AddTypicalLights()
    vis.AddLightWithShadow(ch.ChVector3d(1.5, -2.5, 5.5), ch.ChVector3d(0, 0, 0.5), 3, 4, 10, 40, 512)

    # Instruction 3: Set up sensor manager
    sens_manager = sens.ChSensorManager(hmmwv.GetSystem())
    sens_manager.scene.AddPointLight(ch.ChVector3d(100, 100, 100), ch.ChColor(1, 1, 1), 5000)

    # Instruction 4: Add and configure lidar sensor
    lidar_offset = ch.ChFramed(ch.ChVector3d(0, 0, 1.5), ch.Q_from_AngAxis(0, ch.ChVector3d(0, 1, 0)))
    lidar = sens.ChLidarSensor(
        hmmwv.GetChassisBody(),  # Parent body
        10,                      # Update rate in Hz
        lidar_offset,            # Offset pose
        1000,                    # Horizontal samples
        50,                      # Vertical samples
        math.radians(360),       # Horizontal FOV
        math.radians(20),        # Vertical FOV
        0.1,                     # Max vertical angle
        100.0                    # Max distance
    )
    lidar.SetName("Lidar Sensor")
    lidar.PushFilter(sens.ChFilterLidarNoiseXYZ(0.01, 0.001))
    lidar.PushFilter(sens.ChFilterLidarIntensityNoise(0.1))
    lidar.PushFilter(sens.ChFilterVisualize(640, 360, "Lidar Data"))
    sens_manager.AddSensor(lidar)

    # Create driver system
    driver = veh.ChDriver(hmmwv.GetVehicle())
    driver.Initialize()

    # Create ROS manager
    ros_manager = chros.ChROSPythonManager()
    ros_manager.RegisterHandler(chros.ChROSClockHandler())
    ros_manager.RegisterHandler(chros.ChROSDriverInputsHandler(25, driver, "~/input/driver_inputs"))
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(25, hmmwv.GetChassisBody(), "~/output/hmmwv/state"))
    # Instruction 5: Register lidar handler
    ros_manager.RegisterHandler(chros.ChROSLidarHandler(25, lidar, "~/output/lidar"))
    ros_manager.Initialize()

    # Simulation loop
    time = 0
    time_step = 1e-3
    time_end = 30
    step_number = 0
    render_step_size = 1.0 / 25
    render_steps = math.ceil(render_step_size / time_step)
    hmmwv.GetVehicle().EnableRealtime(True)

    while vis.Run():
        time = hmmwv.GetSystem().GetChTime()
        if time > time_end:
            break

        # Render scene
        if step_number % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        # Get inputs and synchronize
        driver_inputs = driver.GetInputs()
        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)

        # Advance simulation
        driver.Advance(time_step)
        terrain.Advance(time_step)
        hmmwv.Advance(time_step)

        # Instruction 6: Update sensor manager
        sens_manager.Update()

        # Update ROS
        if not ros_manager.Update(time, time_step):
            break

        step_number += 1

if __name__ == "__main__":
    main()