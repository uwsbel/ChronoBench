import chrono as chrono
import chrono.irrlicht as irrlicht


chrono.Initialize()
irrlicht.Initialize()


truss = chrono.ChBodyBox(name='truss')
truss.SetPos(chrono.ChVectorD(0, 0, 0))
truss.SetMass(1)
truss.SetBodyFixed(True)


bar = chrono.ChBodyBox(name='bar')
bar.SetPos(chrono.ChVectorD(1, 0, 0))
bar.SetMass(1)
bar.SetRot(chrono.ChQuaternionD(chrono.VECT_X, 0))
bar.SetBodyFixed(False)


gear1 = chrono.ChBodyGear(name='gear1')
gear1.SetPos(chrono.ChVectorD(2, 0, 0))
gear1.SetMass(1)
gear1.SetRot(chrono.ChQuaternionD(chrono.VECT_X, 0))
gear1.SetBodyFixed(False)

gear2 = chrono.ChBodyGear(name='gear2')
gear2.SetPos(chrono.ChVectorD(3, 0, 0))
gear2.SetMass(1)
gear2.SetRot(chrono.ChQuaternionD(chrono.VECT_X, 0))
gear2.SetBodyFixed(False)


motor = chrono.ChMotorGear(name='motor')
motor.SetPos(chrono.ChVectorD(2.5, 0, 0))
motor.SetMass(1)
motor.SetRot(chrono.ChQuaternionD(chrono.VECT_X, 0))
motor.SetBodyFixed(False)
motor.SetGearRatio(10)
motor.SetTorque(10)


gear1.AddForce(chrono.ChVectorD(0, 0, 0))
gear2.AddForce(chrono.ChVectorD(0, 0, 0))
motor.AddForce(chrono.ChVectorD(0, 0, 0))


irrlicht.SetLightAmbient(chrono.ChVectorD(0.2, 0.2, 0.2))
irrlicht.SetLightDiffuse(chrono.ChVectorD(0.8, 0.8, 0.8))
irrlicht.SetLightSpecular(chrono.ChVectorD(0.9, 0.9, 0.9))
irrlicht.SetLightPosition(chrono.ChVectorD(0, 5, 0))


irrlicht.SetMaterial(chrono.ChMaterialSurface(name='material', texture=chrono.ChTexture(name='texture', filename='texture.png')))
irrlicht.SetScene(truss, bar, gear1, gear2, motor)


chrono.Run()