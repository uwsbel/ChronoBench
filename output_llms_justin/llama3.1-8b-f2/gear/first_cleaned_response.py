import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.cascade as cascade
import math as m







var_mesh = True


def my_mesh_callback(gear, path):
    print("Changed mesh to: " + path)
    gear.SetMesh(chrono.GetChronoDataFile(path))





sys = cascade.CascadeSystemNSC()


truss = cascade.CCascadeBodyNSC(sys)
truss.SetFixed(True)
truss.SetMass(0)
truss.SetInertiaXX(chrono.ChVector3d(1, 1, 1))
truss.SetName("Truss")
sys.Add(truss)



joint = cascade.CCascadeLinkLockRevoluteNSC(sys)
joint.Initialize(truss, cascade.ChFramed(cascade.ChVector3d(0, 0.2, 0), chrono.ChVector3d(0, 1, 0)))
joint.SetName("Revolute")
sys.Add(joint);



bar = cascade.CCascadeBarNSC()
bar.SetLength(1.5)
bar.SetRadius(0.02)
bar.SetChassisFixed(False)
bar.SetMass(0.2)
bar.SetInertiaXX(chrono.ChVector3d(0.1, 0.1, 0.1))
bar.SetName("Bar")
bar.Initialize(sys, cascade.ChFramed(cascade.ChVector3d(0, 0, 0), chrono.ChVector3d(1, 0, 0)))


gear = cascade.CCascadeGearNSC()
gear.SetOuterRadius(0.06)
gear.SetInnerRadius(0.03)
gear.SetChassis(bar.GetChassisBody())
gear.SetSunBody(truss)
gear.SetName("Gear")
gear.Initialize()


motor = cascade.CCascadeMotorSHA_NSVC()
motor.SetShaft1(gear.GetShaft())
motor.SetShaft2(joint.GetShaft())
motor.SetName("Motor")
motor.Initialize(100., 0.05);





vis = cascade.CCascadeVisualSystemIrrlichtNSC()
vis.SetWindowTitle('Epicyclic Gear')
vis.SetWindowSize(1024,768)
vis.SetChaseCamera(chrono.ChVector3d(0.0,0.0,0.75), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachSystem(sys)





sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

while vis.Run() :
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.01)