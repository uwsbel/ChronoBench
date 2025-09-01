import pychrono.core as chrono
import pychrono.sensor as sensor
import math


sys = chrono.ChSystemSMC()
sys.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))


mesh = chrono.ChTriangleMeshConnected()
mesh.LoadWavefrontMesh("mesh.obj",   
                       False,        
                       True)         

fixed_body = chrono.ChBody()
fixed_body.SetBodyFixed(True)


mesh_asset = chrono.ChTriangleMeshShape()
mesh_asset.SetMesh(mesh)
mesh_asset.SetStatic(True)
fixed_body.AddAsset(mesh_asset)

sys.Add(fixed_body)


cam_body = chrono.ChBody()
cam_body.SetMass(1.0)
cam_body.SetInertiaXX(chrono.ChVectorD(1,1,1))
cam_body.SetNoGravity(True)
cam_body.SetBodyFixed(False)
sys.Add(cam_body)


manager = sensor.ChSensorManager(sys)


update_rate  = 30                
w, h         = 640, 480
fov          = chrono.CH_C_PI/3  

camera = sensor.ChCameraSensor(
    parent=cam_body,
    updateRate=update_rate,
    width=w,
    height=h,
    fov=fov)

camera.SetOffsetPose(chrono.ChFrameD())


camera.PushFilter(sensor.ChFilterNoiseRGB(0.02))

camera.PushFilter(sensor.ChFilterVisualize(w, h, "Camera View"))

manager.AddSensor(camera)


def look_at_q(fromP, toP, up=chrono.ChVectorD(0,0,1)):
    dir = toP - fromP
    dir.Normalize()
    right = up.Cross(dir)
    right.Normalize()
    new_up = dir.Cross(right)
    
    M = chrono.ChMatrix33D(
        right.x,   new_up.x,   dir.x,
        right.y,   new_up.y,   dir.y,
        right.z,   new_up.z,   dir.z)
    q = chrono.ChQuaternionD()
    q.Q_from_RotationMatrix(M)
    return q


dt        = 1.0/100.0
n_steps   = 300
radius    = 2.0
center     = chrono.ChVectorD(0,0,0)

for i in range(n_steps):
    theta = 2*math.pi * (i / n_steps)
    pos   = chrono.ChVectorD(radius*math.cos(theta),
                             radius*math.sin(theta),
                             1.0)
    
    cam_body.SetPos(pos)
    cam_body.SetRot(look_at_q(pos, center))

    
    sys.DoStepDynamics(dt)
    manager.Update()

    
    buf = camera.GetMostRecentRGBA8Buffer()
    
    print(f"Step {i:3d} t={sys.GetChTime():.3f}  buffer_len={len(buf)}  first 8 bytes={list(buf[:8])}")


while True:
    sys.DoStepDynamics(dt)
    manager.Update()