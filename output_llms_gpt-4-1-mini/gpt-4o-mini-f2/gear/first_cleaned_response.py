import pychrono as chrono
import pychrono.irrlicht as chronoirr


sys = chrono.ChSystemNSC()


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Epicyclic Gears')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 0.5, 1.5))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0), 3, 4, 10, 40, 512)


truss = chrono.ChBody()
sys.Add(truss)
truss.SetFixed(True)
truss.EnableCollision(False)


truss.AddVisualShape(chrono.ChVisualShapeSphere(0.05))


barA = chrono.ChBody()
sys.Add(barA)
barA.SetFixed(False)
barA.SetPos(chrono.ChVector3d(0, 0, 0))
barA.EnableCollision(False)


barA.AddVisualShape(chrono.ChVisualShapeCylinder(0.05, 0.2))


gearB = chrono.ChBody()
sys.Add(gearB)
gearB.SetFixed(False)
gearB.SetPos(chrono.ChVector3d(0.2, 0, 0))
gearB.EnableCollision(False)


gearB.AddVisualShape(chrono.ChVisualShapeCylinder(0.05, 0.1))


gearC = chrono.ChBody()
sys.Add(gearC)
gearC.SetFixed(False)
gearC.SetPos(chrono.ChVector3d(0.1, 0, 0))
gearC.EnableCollision(False)


gearC.AddVisualShape(chrono.ChVisualShapeCylinder(0.05, 0.15))


revA = chrono.ChLinkLockRevolute()
sys.Add(revA)
revA.Initialize(truss, barA, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 1, 0)))


revB = chrono.ChLinkLockRevolute()
sys.Add(revB)
revB.Initialize(barA, gearB, chrono.ChFramed(chrono.ChVector3d(0.2, 0, 0), chrono.ChVector3d(0, 1, 0)))


revC = chrono.ChLinkLockRevolute()
sys.Add(revC)
revC.Initialize(gearB, gearC, chrono.ChFramed(chrono.ChVector3d(0.1, 0, 0), chrono.ChVector3d(0, 1, 0)))


gearBC = chrono.ChLinkGear()
sys.Add(gearBC)
gearBC.Initialize(gearB, gearC, chrono.ChFramed(chrono.ChVector3d(0.1, 0, 0), chrono.ChVector3d(0, 1, 0)))
gearBC.SetName('GBC')
gearBC.SetTimestep(1e-3)
gearBC.SetGearRatio(2)


gearAB = chrono.ChLinkGear()
sys.Add(gearAB)
gearAB.Initialize(gearA, gearB, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 1, 0)))
gearAB.SetName('GAB')
gearAB.SetTimestep(1e-3)
gearAB.SetGearRatio(1)


motorA = chrono.ChLinkMotorRotationSpeed()
sys.Add(motorA)
motorA.Initialize(gearA, gearB, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 1, 0)))
motorA.SetSpeed(chrono.CH_PI_2)  


sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED)


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(1e-3)