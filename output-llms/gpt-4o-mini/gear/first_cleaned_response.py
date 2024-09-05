import pychrono.core as chrono
import pychrono.irrlicht as chronoirr


chrono.SetChronoDataPath('/path/to/chrono/data/')  
system = chrono.ChSystemNSC()


application = chronoirr.ChIrrApp(system, 'Epicyclic Gears Simulation', chronoirr.dimension2du(800, 600))
application.SetSkyBox(chrono.GetChronoDataPath() + 'skybox/skybox.jpg')
application.AddLogo(chrono.GetChronoDataPath() + 'chrono_logo_alpha.png')
application.AddCamera(chrono.ChVectorD(0, 2, -10), chrono.ChVectorD(0, 1, 0))
application.AddTypicalLights()


truss = chrono.ChBodyAuxRef()
truss.SetBodyFixed(True)
truss.SetPos(chrono.ChVectorD(0, 0, 0))
truss.GetMaterialSurface().SetFriction(0.4)
truss.GetMaterialSurface().SetRestitution(0.1)
system.Add(truss)


bar = chrono.ChBody()
bar.SetMass(1.0)
bar.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))
bar.SetPos(chrono.ChVectorD(0, 1, 0))
bar.SetRot(chrono.Q_from_AngZ(chrono.CH_C_PI / 4))  
system.Add(bar)


gear1 = chrono.ChBody()
gear1.SetMass(0.5)
gear1.SetInertiaXX(chrono.ChVectorD(0.01, 0.01, 0.01))
gear1.SetPos(chrono.ChVectorD(0, 1, 0))
gear1.SetRot(chrono.Q_from_AngZ(0))
system.Add(gear1)

gear2 = chrono.ChBody()
gear2.SetMass(0.5)
gear2.SetInertiaXX(chrono.ChVectorD(0.01, 0.01, 0.01))
gear2.SetPos(chrono.ChVectorD(0.5, 1, 0))
gear2.SetRot(chrono.Q_from_AngZ(0))
system.Add(gear2)


gear_motor = chrono.ChLinkMotorRotation()
gear_motor.Initialize(gear1, gear2, chrono.ChFrameD(chrono.ChVectorD(0, 1, 0)))
gear_motor.SetMotorType(chrono.ChLinkMotor::eChLinkMotorType::MOTOR);
gear_motor.SetSpeed(chrono.CH_C_PI / 2)  
system.Add(gear_motor)


truss.GetMaterialSurface().SetFriction(0.4)
truss.GetMaterialSurface().SetRestitution(0.1)

bar.GetMaterialSurface().SetFriction(0.4)
bar.GetMaterialSurface().SetRestitution(0.1)

gear1.GetMaterialSurface().SetFriction(0.4)
gear1.GetMaterialSurface().SetRestitution(0.1)

gear2.GetMaterialSurface().SetFriction(0.4)
gear2.GetMaterialSurface().SetRestitution(0.1)


application.AssetBindAll()
application.AssetUpdateAll()


time_step = 0.01
application.SetTimestep(time_step)

while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.EndScene()
    
    
    system.DoStepDynamics(time_step)


application.GetDevice().drop()