import pychrono as chrono
import pychrono.sensor as sens
import math


sys = chrono.ChSystemNSC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


mesh = chrono.ChBodyEasyMesh()
mesh.SetMeshFilename(chrono.GetChronoDataFile("vehichles/meshes/triangulated_meshes/triangulated_mesh.obj"))
mesh.SetPos(chrono.ChVector3d(0, 0, 0))
mesh.SetFixed(True)
mesh.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
sys.Add(mesh)


manager = sens.ChSensorManager(sys)


cam = sens.ChCameraSensor(
    mesh,
    30,  
    chrono.ChFrameD(chrono.ChVector3d(0, 0, 2), chrono.QUNIT),
    1280,  
    720,   
    60,    
    0.01,  
    1000,  
    sens.CameraProjectionType_PERSPECTIVE
)
cam.SetName("Camera Sensor")
cam.SetLag(0.0)
cam.SetCollectionWindow(1.0 / 30)  
cam.SetWriteImage(True, "output/images/", "cam_%.6d.png")
cam.SetImageType(sens.ChCameraSensor.ImageType_RGB)
manager.AddSensor(cam)


if False:  
    cam.PushFilter(sens.ChFilterNoise(0.01, 0.01, 0.0))
    cam.PushFilter(sens.ChFilterLensDistortion(0.1, 0.1, 0.0))
    cam.PushFilter(sens.ChFilterRGB())
    cam.PushFilter(sens.ChFilterVisualize(640, 480, "Camera RGB"))


if False:  
    cam.PushFilter(sens.ChFilterDepth())
    cam.PushFilter(sens.ChFilterVisualizeDepth(640, 480, "Camera Depth"))


if True:  
    cam.PushFilter(sens.ChFilterBuffer(1))


if True:  
    cam.PushFilter(sens.ChFilterVisualize(640, 480, "Camera Buffer"))


manager.InitializeSensors()


orbit_radius = 5
orbit_speed = 0.1
orbit_angle = 0


while True:
    
    orbit_angle += orbit_speed
    cam.SetPosOffset(chrono.ChVector3d(orbit_radius * math.cos(orbit_angle), orbit_radius * math.sin(orbit_angle), 2))
    cam.SetLookAtOffset(chrono.ChVector3d(0, 0, 0))

    
    if cam.HasBuffers():
        print("Camera buffer data:")
        print(cam.GetMostRecentBuffer().GetRGBA8Data())
    
    
    manager.Update()

    
    sys.DoStepDynamics(1e-3)