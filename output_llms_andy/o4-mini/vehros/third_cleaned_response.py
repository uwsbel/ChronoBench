import pychrono as ch
import pychrono.vehicle as veh
import pychrono.sensor as sens
import pychrono.ros as chros
from pychrono import irrlicht as chronoirr
import math

def main():
    
    
    
    veh.SetDataPath(ch.GetChronoDataPath() + 'vehicle/')

    
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(ch.ChContactMethod_NSC)
    hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
    hmmwv.SetChassisFixed(False)
    hmmwv.SetInitPosition(ch.ChCoordsysd(ch.ChVector3d(0, 0, 1.6),
                                         ch.ChQuaterniond(1, 0, 0, 0)))
    hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
    hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
    hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
    hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
    hmmwv.SetTireType(veh.TireModelType_TMEASY)
    hmmwv.SetTireStepSize(1e-3)
    hmmwv.Initialize()
    
    for vis in ["Chassis", "Suspension", "Steering", "Wheel", "Tire"]:
        getattr(hmmwv, f"Set{vis}VisualizationType")(veh.VisualizationType_MESH)

    
    terrain = veh.RigidTerrain(hmmwv.GetSystem())
    patch_mat = ch.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    
    patch = terrain.AddPatch(patch_mat, veh.CSYSNORM, 100.0, 100.0)
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 100, 100)
    terrain.Initialize()

    
    
    
    box = ch.ChBodyEasyBox(1.0, 1.0, 1.0,    
                           1000.0,           
                           True, True)       
    box.SetPos(ch.ChVector3d(2.0, 0.0, 1.0))
    box.SetBodyFixed(False)
    hmmwv.GetSystem().Add(box)

    
    
    
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(hmmwv.GetSystem())
    vis.SetCameraVertical(ch.CameraVerticalDir_Z)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle('HMMWV + Lidar on Rigid Terrain')
    vis.Initialize()
    vis.AddLogo(ch.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    
    vis.AddCamera(ch.ChVector3d(-5, 2.5, 1.5),
                  ch.ChVector3d(0, 0, 1))
    vis.AddTypicalLights()
    vis.AddLightWithShadow(ch.ChVector3d(1.5, -2.5, 5.5),
                           ch.ChVector3d(0, 0, 0.5),
                           3, 4, 10, 40, 512)

    
    
    
    driver = veh.ChDriver(hmmwv.GetVehicle())
    driver.Initialize()

    
    
    
    ros_manager = chros.ChROSPythonManager()
    ros_manager.RegisterHandler(chros.ChROSClockHandler())
    ros_manager.RegisterHandler(
        chros.ChROSDriverInputsHandler(25, driver, "~/input/driver_inputs"))
    ros_manager.RegisterHandler(
        chros.ChROSBodyHandler(25, hmmwv.GetChassisBody(),
                               "~/output/hmmwv/state"))
    
    ros_manager.Initialize()

    
    
    
    sens_manager = sens.ChSensorManager(hmmwv.GetSystem())
    

    
    update_rate    = 10              
    horiz_samples  = 180
    horiz_fov      = (-math.pi/2, math.pi/2)
    vert_samples   = 32
    vert_fov       = (-math.pi/6, math.pi/6)
    max_dist       = 10.0

    
    offset_pose = sens.ChFrameD(
        ch.ChVector3d(0.5, 0.0, 1.2),
        ch.ChQuaterniond(1, 0, 0, 0))

    lidar = sens.ChLidarSensor(
        parent = hmmwv.GetChassisBody(),
        updateRate = update_rate,
        horizontalSamples = horiz_samples,
        horizontalFov = horiz_fov,
        verticalSamples = vert_samples,
        verticalFov = vert_fov,
        maxDistance = max_dist,
        offsetPose = offset_pose)

    
    lidar.PushFilter(sens.ChFilterLidarNoise(
        sens.ChNoiseGaussian, 0.01))
    lidar.PushFilter(sens.ChFilterLidarDropOffPoints(
        min_distance = 0.2, max_distance = max_dist))
    lidar.PushFilter(sens.ChFilterXYZtoRGB(0.0, 1.0, 0.0))
    lidar.PushFilter(sens.ChFilterPCtoROSPointCloud2(
        topic = "~/output/lidar_points",
        frame = "lidar_frame"))

    
    sens_manager.AddSensor(lidar)

    
    ros_manager.RegisterHandler(
        chros.ChROSLidarHandler(25, lidar, "~/output/lidar"))

    
    
    
    time      = 0.0
    time_end  = 30.0
    time_step = 1e-3
    render_fps      = 25
    render_step_sz  = 1.0/render_fps
    render_steps    = math.ceil(render_step_sz/time_step)
    step_number     = 0

    hmmwv.GetVehicle().EnableRealtime(True)

    while vis.Run():
        time = hmmwv.GetSystem().GetChTime()

        
        if (step_number % render_steps) == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        
        driver_inputs = driver.GetInputs()
        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)

        
        driver.Advance(time_step)
        terrain.Advance(time_step)
        hmmwv.Advance(time_step)

        
        sens_manager.Update()

        
        if not ros_manager.Update(time, time_step):
            break

        step_number += 1

if __name__ == "__main__":
    main()