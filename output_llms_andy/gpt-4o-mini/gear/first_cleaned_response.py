import pychrono.core as chrono
import pychrono.irrlicht as chronoirr


chrono.SetChronoDataPath('path/to/chrono/data/')  


system = chrono.ChSystemNSC()


fixed_truss = chrono.ChBodyAuxRef()
fixed_truss.SetBodyFixed(True)
fixed_truss.SetPos(chrono.ChVectorD(0, 0, 0))
fixed_truss.GetCollisionModel().ClearModel()
fixed_truss.GetCollisionModel().AddBox(1, 0.1, 1)  
fixed_truss.GetCollisionModel().BuildModel()
fixed_truss.SetCollide(True)
system.Add(fixed_truss)


rotating_bar = chrono.ChBody()
rotating_bar.SetMass(1.0)
rotating_bar.SetPos(chrono.ChVectorD(0, 0.5, 0))
rotating_bar.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))
rotating_bar.SetCollide(True)
rotating_bar.GetCollisionModel().ClearModel()
rotating_bar.GetCollisionModel().AddBox(0.1, 0.1, 0.5)  
rotating_bar.GetCollisionModel().BuildModel()
system.Add(rotating_bar)


gear1 = chrono.ChBody()
gear2 = chrono.ChBody()
gear1.SetMass(0.5)
gear2.SetMass(0.5)


gear1.SetPos(chrono.ChVectorD(0, 0.5, 0))
gear2.SetPos(chrono.ChVectorD(0.2, 0.5, 0))
gear1.SetInertiaXX(chrono.ChVectorD(0.01, 0.01, 0.01))
gear2.SetInertiaXX(chrono.ChVectorD(0.01, 0.01, 0.01))


gear1.GetCollisionModel().ClearModel()
gear2.GetCollisionModel().ClearModel()
gear1.GetCollisionModel().AddCylinder(0.05, 0.05, 0.1)  
gear2.GetCollisionModel().AddCylinder(0.05, 0.05, 0.1)  
gear1.GetCollisionModel().BuildModel()
gear2.GetCollisionModel().BuildModel()

system.Add(gear1)
system.Add(gear2)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(rotating_bar, gear1, chrono.ChFrameD(chrono.ChVectorD(0, 0.5, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
motor.SetSpeed(chrono.ChFunction_Const(1.0))  
system.AddLink(motor)


app = chronoirr.ChIrrApp(system, 'Epicyclic Gears Simulation', chronoirr.dimension2d(800, 600))
app.AddLogo(chrono.GetChronoDataPath() + 'logo.png')
app.AddSkyBox()
app.AddCamera(chrono.ChVectorD(2, 2, 2), chrono.ChVectorD(0, 0, 0))
app.AddLight(chrono.ChVectorD(10, 10, 10), chrono.ChVectorD(1, 1, 1))


time_step = 0.01
app.SetTimestep(time_step)


while app.GetDevice().run():
    app.BeginScene()
    app.DrawAll()
    app.EndScene()
    system.DoStepDynamics(time_step)