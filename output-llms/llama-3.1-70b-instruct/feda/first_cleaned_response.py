import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


chrono.SetChronoDataPath('./data/')


mysystem = chrono.ChSystemNSC()


feda = veh.FE_Drivetrain()
feda.SetContactMethod(chrono.ChContactMethod_NSC)
feda.SetTireModel(chrono.ChTireModel_Rigid_Random)
feda.SetChassisFixed(False)
feda.SetInitPosition(chrono.ChVectorD(0, 0, 0.5))
feda.SetInitOrientation(chrono.ChQuaternionD(1, 0, 0, 0))


mysystem.Add(feda.GetChassisBody())
mysystem.Add(feda.GetPart(0))
mysystem.Add(feda.GetPart(1))
mysystem.Add(feda.GetPart(2))
mysystem.Add(feda.GetPart(3))


terrain = chrono.ChBodyEasyRigid()
terrain.SetMass(1000)
terrain.SetInertiaXX(chrono.ChVectorD(1, 1, 1))
terrain.SetPos(chrono.ChVectorD(0, 0, 0))
terrain.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))
terrain.SetMaterialSurface(chrono.ChMaterialSurfaceNSC())
terrain.SetVisualizationType(chrono.ChVisualizationType_MESH)
terrain.SetMesh(chrono.ChTriangleMesh())
terrain.GetMesh().LoadWavefrontMesh('data/terrain.obj')
terrain.SetTexture(chrono.ChTexture('data/terrain.jpg'))


mysystem.Add(terrain)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('FEDA Vehicle Simulation')
vis.SetCamera(chrono.ChVectorD(0, 0, 2), chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 1, 0))
vis.SetCameraFollow(feda.GetChassisBody(), chrono.ChVectorD(0, 0, 2), chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 1, 0))


driver = veh.ChIrrlichtDriver()
driver.SetVehicle(feda)
driver.SetSteering(0)
driver.SetThrottle(0)
driver.SetBraking(0)


while vis.Run():
    
    mysystem.DoStepDynamics(0.02)
    
    
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()
    
    
    driver.Synchronize(0.02)
    
    
    vis.SetCamera(chrono.ChVectorD(0, 0, 2), chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 1, 0))
    vis.SetCameraFollow(feda.GetChassisBody(), chrono.ChVectorD(0, 0, 2), chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 1, 0))