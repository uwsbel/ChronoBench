import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.engine as chronoengine


sys = chrono.ChSystemNSC()


chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.001)
chrono.ChMaterialSurfaceNSC.SetDefaultFriction(0.5)


ground = chrono.ChBodyEasyBox(sys, 10, 1, 10, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -5, 0))
ground.SetBodyFixed(True)


truss = chrono.ChBodyEasyBox(sys, 1, 0.1, 1, 1000, True, True)
truss.SetPos(chrono.ChVectorD(0, 0, 0))
truss.SetBodyFixed(True)


bar = chrono.ChBodyEasyBox(sys, 2, 0.1, 0.1, 1000, True, True)
bar.SetPos(chrono.ChVectorD(0, 1, 0))
bar.AddConstraint(chrono.ChLinkLockRotation(ground, bar))


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(ground, bar)
motor.SetSpeed(10)  


gear1 = chrono.ChBodyEasyCylinder(sys, 0.5, 0.1, 1000, True, True)
gear1.SetPos(chrono.ChVectorD(-1, 1, 0))
gear1.AddConstraint(chrono.ChLinkLockRotation(ground, gear1))

gear2 = chrono.ChBodyEasyCylinder(sys, 0.5, 0.1, 1000, True, True)
gear2.SetPos(chrono.ChVectorD(1, 1, 0))
gear2.AddConstraint(chrono.ChLinkLockRotation(ground, gear2))


chrono.ChLinkMateGeneric(gear1, bar, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngX(chrono.CH_C_PI_2)), chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngX(chrono.CH_C_PI_2)))
chrono.ChLinkMateGeneric(gear2, bar, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngX(-chrono.CH_C_PI_2)), chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngX(-chrono.CH_C_PI_2)))


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AddCamera(chrono.ChVectorD(0, 5, -10))
vis.AddCamera(chrono.ChVectorD(0, 5, 10))
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Epicyclic Gears Simulation")
vis.AddTypicalLights()


mat_ground = chrono.ChMaterialSurfaceNSC()
mat_ground.SetFriction(0.5)
mat_ground.SetRestitution(0.5)
mat_ground.SetColor(chrono.ChColor(0.5, 0.5, 0.5))
ground.GetCollisionModel().AddMaterial(mat_ground)

mat_truss = chrono.ChMaterialSurfaceNSC()
mat_truss.SetFriction(0.5)
mat_truss.SetRestitution(0.5)
mat_truss.SetColor(chrono.ChColor(0.8, 0.8, 0.8))
truss.GetCollisionModel().AddMaterial(mat_truss)

mat_bar = chrono.ChMaterialSurfaceNSC()
mat_bar.SetFriction(0.5)
mat_bar.SetRestitution(0.5)
mat_bar.SetColor(chrono.ChColor(0.2, 0.2, 0.2))
bar.GetCollisionModel().AddMaterial(mat_bar)

mat_gear1 = chrono.ChMaterialSurfaceNSC()
mat_gear1.SetFriction(0.5)
mat_gear1.SetRestitution(0.5)
mat_gear1.SetColor(chrono.ChColor(0.5, 0, 0))
gear1.GetCollisionModel().AddMaterial(mat_gear1)

mat_gear2 = chrono.ChMaterialSurfaceNSC()
mat_gear2.SetFriction(0.5)
mat_gear2.SetRestitution(0.5)
mat_gear2.SetColor(chrono.ChColor(0, 0.5, 0))
gear2.GetCollisionModel().AddMaterial(mat_gear2)


while vis.Run():
    sys.DoStepDynamics(0.01)
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()