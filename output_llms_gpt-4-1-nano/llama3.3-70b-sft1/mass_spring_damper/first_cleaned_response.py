import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens



mphysicalSystem = chrono.ChSystemNSC()



mmesh = chrono.ChTriangleMeshConnected()
mmesh.LoadWavefrontMesh(chrono.GetChronoDataFile("vehicle/hmmwv/hmmwv_chassis.obj"), False, True)
mmesh.Transform(chrono.ChVector3d(0, 0, 0), chrono.ChMatrix33d(2))

trimesh_shape = chrono.ChVisualShapeTriangleMesh()
trimesh_shape.SetMesh(mmesh)
trimesh_shape.SetName("HMMWV Chassis Mesh")
trimesh_shape.SetMutable(False)

mesh_body = chrono.ChBody()
mesh_body.SetPos(chrono.ChVector3d(0, 0, 0))
mesh_body.AddVisualShape(trimesh_shape)
mesh_body.SetFixed(True)
mphysicalSystem.Add(mesh_body)



box = chrono.ChBodyEasyBox(1, 1, 1, 1000)
box.SetPos(chrono.ChVector3d(0, 1, 0))
box.SetFixed(True)
box.GetVisualShape(0).SetMutable(False)
mphysicalSystem.Add(box)


vis = sens.ChVisualSystemIrrlicht()
vis.AttachSystem(mphysicalSystem)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Mesh')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(4, 4, 4), chrono.ChVector3d(0, 1, 0))
vis.AddTypicalLights()


t1 = 0
t2 = 100

while vis.Run() :
    t = mphysicalSystem.GetChTime()

    if (t >= t2):
        break

    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(0.01)