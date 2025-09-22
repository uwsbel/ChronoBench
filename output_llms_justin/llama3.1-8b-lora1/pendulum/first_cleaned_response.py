import pychrono as chrono
import pychrono.irrlicht as irr
import math







m1 = chrono.ChBodyEasyBox(0.5, 0.4, 1.0, 1000, True, True)
m1.SetPosChWorld(chrono.ChVector3d(0, 0, 2))
m1.SetAngleXY(0.2)
m1.SetRotAxis(chrono.ChVector3d(0, 0, 1))
m1.SetName('floor')
m1.GetVisualShape(0).SetMaterial(0.0, 0.0, 0.1)

sys = chrono.ChSystemNSC()
sys.Add(m1)


m2 = chrono.ChBody()
m2.SetMass(10)
m2.SetInertiaXXNNN(1,1,1)
m2.SetPos(chrono.ChVector3d(0,-0.5,0.5))
m2.AddCollisionShape(chrono.ChCollisionShapeBox(m2.GetHalfExtension(), m2.GetHalfExtension(), m2.GetHalfExtension()))
sys.Add(m2)


m3 = chrono.ChBody()
m3.SetName('pendulum')
m3.SetPos(chrono.ChVector3d(0,0,1))
m3.AddRevoluteConstraint(m2, chrono.ChFramed(chrono.ChVector3d(0,-0.5,0.5), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(1,0,0))))
sys.Add(m3)


motor = chrono.ChLinkLockRevolute()
motor.SetSpindleSpeed(1)
motor.Initialize(m2, m3)
sys.Add(motor)


shape = chrono.ChCollisionShapeBox(m3.GetHalfExtension(), m3.GetHalfExtension(), m3.GetHalfExtension())
shape.SetNegationMethod(chrono.ChCollisionBoxNegationMethod_BULLET)
m3.AddCollisionShape(shape)









vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024,768)
vis.SetWindowTitle('Pendulum')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.2,0.2,-0.2))
vis.AddTypicalLights()


driver = irr.ChDriverSimple(vis)
driver.SetChaseCamera(chrono.ChVector3d(0.2,0.2,-0.2), 6.0, 0.5)


while vis.Run() :
    vis.BeginScene()
    vis.GetSceneManager().getActiveCamera().setTarget(0.2,0.2,-0.2)
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.01)

    
    q = m3.GetRot()
    x = m3.GetPos()
    v = m3.GetVel()
    print ('time =', sys.GetChTime(), ' x =', x, ' v =', v, ' q =', q)