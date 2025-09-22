import pychrono as chrono
import pychrono.sensor as sens
import numpy as np

def main():
    
    chrono.SetChronoDataPath('./chrono_data/')
    sys = chrono.ChSystemNSC()

    
    mesh = chrono.ChTriangleMeshConnected()
    mesh.LoadWavefrontMesh(chrono.GetChronoDataPath() + 'robot.obj')
    mesh.Transform(chrono.ChVectorD(0, 0, 0), chrono.ChMatrix33D(1))
    mesh_asset = chrono.ChTriangleMeshShape()
    mesh_asset.SetMesh(mesh)
    body = chrono.ChBody()
    body.SetBodyFixed(True)
    body.AddAsset(mesh_asset)
    sys.Add(body)

    
    offset_pose = chrono.ChFrameD(chrono.ChVectorD(-1, 0, 0), chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0)))
    cam = sens.ChCameraSensor(
        body, 
        10,  
        offset_pose, 
        640,  
        480,  
        90  
    )
    cam.SetName("Camera Sensor")
    cam.PushFilter(sens.ChFilterRGBA8())
    cam.PushFilter(sens.ChFilterGrayscale())
    cam.PushFilter(sens.ChFilterNoiseSaltPepper(0.1))
    cam.PushFilter(sens.ChFilterVisualize(640, 480, "Camera Output"))
    cam.PushFilter(sens.ChFilterSave(chrono.GetChronoOutputPath() + "cam_salt_pepper/"))

    
    manager = sens.ChSensorManager(sys)
    manager.AddSensor(cam)

    
    angle = 0
    while sys.GetChTime() < 10:
        
        angle += 0.01
        offset_pose = chrono.ChFrameD(chrono.ChVectorD(2 * np.cos(angle), 0, 2 * np.sin(angle)), chrono.Q_from_AngAxis(angle, chrono.ChVectorD(0, 1, 0)))
        cam.SetOffsetPose(offset_pose)

        
        manager.Update()

        
        buffer = cam.GetMostRecentBuffer()
        if buffer.HasData():
            print(buffer.GetRGBA8().shape)

        
        sys.DoStepDynamics(0.01)

if __name__ == "__main__":
    main()