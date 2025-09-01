import chrono as chrono
import chrono.irrlicht as irrlicht


chrono.Initialize()


truss = chrono.ChBody()
truss.SetName('Truss')
truss.SetMass(1.0)
truss.SetInertiaXX([1.0, 1.0])
truss.SetPos(chrono.ChVectorD(0, 0, 0))
truss.SetRot(chrono.Q_from_AngX(0))
truss.SetBodyFixed(True)
truss.SetBodyMaterial(chrono.ChMaterialSurfaceNSC3D(0.9, 0.9, 0.9, 0.0, 0.0, 0.0))
chrono.Add(truss)


bar = chrono.ChBody()
bar.SetName('Bar')
bar.SetMass(1.0)
bar.SetInertiaXX([1.0, 1.0])
bar.SetPos(chrono.ChVectorD(0, 0, 0))
bar.SetRot(chrono.Q_from_AngX(0))
bar.SetBodyFixed(False)
bar.SetBodyMaterial(chrono.ChMaterialSurfaceNSC3D(0.9, 0.9, 0.9, 0.0, 0.0, 0.0))
chrono.Add(bar)


motor = chrono.ChGearMotor()
motor.SetName('Motor')
motor.SetMass(1.0)
motor.SetInertiaXX([1.0, 1.0])
motor.SetPos(chrono.ChVectorD(0, 0, 0))
motor.SetRot(chrono.Q_from_AngX(0))
motor.SetBodyFixed(False)
motor.SetBodyMaterial(chrono.ChMaterialSurfaceNSC3D(0.9, 0.9, 0.9, 0.0, 0.0, 0.0))
chrono.Add(motor)


gear = chrono.ChGear()
gear.SetName('Gear')
gear.SetMass(1.0)
gear.SetInertiaXX([1.0, 1.0])
gear.SetPos(chrono.ChVectorD(0, 0, 0))
gear.SetRot(chrono.Q_from_AngX(0))
gear.SetBodyFixed(False)
gear.SetBodyMaterial(chrono.ChMaterialSurfaceNSC3D(0.9, 0.9, 0.9, 0.0, 0.0, 0.0))
chrono.Add(gear)


gear2 = chrono.ChGear()
gear2.SetName('Gear2')
gear2.SetMass(1.0)
gear2.SetInertiaXX([1.0, 1.0])
gear2.SetPos(chrono.ChVectorD(0, 0, 0))
gear2.SetRot(chrono.Q_from_AngX(0))
gear2.SetBodyFixed(False)
gear2.SetBodyMaterial(chrono.ChMaterialSurfaceNSC3D(0.9, 0.9, 0.9, 0.0, 0.0, 0.0))
chrono.Add(gear2)


motor.SetGear(gear)
motor.SetGear2(gear2)


gear.SetGearMotor(motor)
gear2.SetGearMotor(motor)


bar.SetRot(chrono.Q_from_AngX(0))
bar.SetRotSpeed(1.0)


gear.SetRot(chrono.Q_from_AngX(0))
gear.SetRotSpeed(1.0)
gear2.SetRot(chrono.Q_from_AngX(0))
gear2.SetRotSpeed(1.0)


visual = irrlicht.Irrlicht()
visual.CreateWindow(800, 600, "Epicyclic Gears")
visual.AddSceneNode()
visual.AddLightSource(irrlicht.EYEPOS, irrlicht.EYEPOS + chrono.ChVectorD(0, 0, 10))
visual.AddLightSource(irrlicht.EYEPOS, irrlicht.EYEPOS + chrono.ChVectorD(0, 0, -10))
visual.AddLightSource(irrlicht.EYEPOS, irrlicht.EYEPOS + chrono.ChVectorD(0, 10, 0))
visual.AddLightSource(irrlicht.EYEPOS, irrlicht.EYEPOS + chrono.ChVectorD(0, -10, 0))
visual.AddLightSource(irrlicht.EYEPOS, irrlicht.EYEPOS + chrono.ChVectorD(10, 0, 0))
visual.AddLightSource(irrlicht.EYEPOS, irrlicht.EYEPOS + chrono.ChVectorD(-10, 0, 0))


truss.SetMaterial(chrono.ChMaterialSurfaceNSC3D(0.9, 0.9, 0.9, 0.0, 0.0, 0.0))
bar.SetMaterial(chrono.ChMaterialSurfaceNSC3D(0.9, 0.9, 0.9, 0.0, 0.0, 0.0))
motor.SetMaterial(chrono.ChMaterialSurfaceNSC3D(0.9, 0.9, 0.9, 0.0, 0.0, 0.0))
gear.SetMaterial(chrono.ChMaterialSurfaceNSC3D(0.9, 0.9, 0.9, 0.0, 0.0, 0.0))
gear2.SetMaterial(chrono.ChMaterialSurfaceNSC3D(0.9, 0.9, 0.9, 0.0, 0.0, 0.0))


while visual.IsRunning():
    visual.BeginScene()
    visual.DrawAll()
    visual.EndScene()
    chrono.DoStepDynamics(0.01)


chrono.Clear()
chrono.Destroy()
visual.Close()