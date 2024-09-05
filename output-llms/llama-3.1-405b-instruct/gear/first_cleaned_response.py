import pychrono.core as chrono
import pychrono.irrlicht as chronoirr


chrono.SetChronoDataPath('/path/to/chrono/data/')


sys = chrono.ChSystemNSC()


ground = chrono.ChBodyEasyBox(sys, 10, 1, 10, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -5, 0))
sys.Add(ground)


truss = chrono.ChBodyEasyBox(sys, 2, 2, 2, 1000, True, True)
truss.SetPos(chrono.ChVectorD(0, 0, 0))
truss.SetBodyFixed(True)
sys.Add(truss)


bar = chrono.ChBodyEasyCylinder(sys, 1, 1, 1000, True, True)
bar.SetPos(chrono.ChVectorD(0, 2, 0))
sys.Add(bar)


gear1 = chrono.ChBodyEasyCylinder(sys, 0.5, 1, 1000, True, True)
gear1.SetPos(chrono.ChVectorD(-1, 2, 0))
sys.Add(gear1)

gear2 = chrono.ChBodyEasyCylinder(sys, 0.5, 1, 1000, True, True)
gear2.SetPos(chrono.ChVectorD(1, 2, 0))
sys.Add(gear2)


rev1 = chrono.ChLinkRevolute()
rev1.Initialize(bar, gear1, chrono.ChFrameD(chrono.ChVectorD(-1, 2, 0)))
sys.Add(rev1)


rev2 = chrono.ChLinkRevolute()
rev2.Initialize(bar, gear2, chrono.ChFrameD(chrono.ChVectorD(1, 2, 0)))
sys.Add(rev2)


gear_constraint = chrono.ChLinkGear()
gear_constraint.Initialize(gear1, gear2, 2, 1, chrono.ChFrameD(chrono.ChVectorD(0, 2, 0)))
sys.Add(gear_constraint)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(bar, gear1, chrono.ChFrameD(chrono.ChVectorD(-1, 2, 0)))
motor.SetMotorFunction(chrono.ChFunction_Const(chrono.CH_C_PI / 2))
sys.Add(motor)


app = chronoirr.ChIrrApp(sys, 'Epicyclic Gears', chronoirr.dimension2du(800, 600))


app.AddCamera(chronoirr.vector3df(0, 5, -10))


app.AddLight(chronoirr.vector3df(2, 2, 2), chronoirr.vector3df(0, 0, 0), 3, 0, 3, 7)


app.AssetBindAll()
app.AssetUpdateAll()
ground.GetVisualShape().SetMaterial(app.GetMaterial('ground'))


truss.GetVisualShape().SetMaterial(app.GetMaterial('metal'))


bar.GetVisualShape().SetMaterial(app.GetMaterial('metal'))


gear1.GetVisualShape().SetMaterial(app.GetMaterial('metal'))
gear2.GetVisualShape().SetMaterial(app.GetMaterial('metal'))


app.SetTimestep(0.01)
app.SetTryRealtime(True)
while app.GetDevice().run():
    app.BeginScene()
    app.DrawAll()
    app.DoStep()
    app.EndScene()