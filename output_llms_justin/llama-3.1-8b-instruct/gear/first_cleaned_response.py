import chrono as ch
import irrrlichtpython as irr


ch.InitChSystem()
irr.InitIrrlicht()


truss = ch.ChBody()
truss.SetBodyType(ch.ChBodyType.STATIC)
truss.SetMass(0.0)
truss.SetPos(ch.Vec3(0, 0, 0))
truss.SetRot(ch.Vec3(0, 0, 0))


bar = ch.ChBody()
bar.SetBodyType(ch.ChBodyType.KINEMATIC)
bar.SetMass(1.0)
bar.SetPos(ch.Vec3(0, 0, 1))
bar.SetRot(ch.Vec3(0, 0, 0))


gear1 = ch.ChBody()
gear1.SetBodyType(ch.ChBodyType.KINEMATIC)
gear1.SetMass(1.0)
gear1.SetPos(ch.Vec3(-1, 0, 0))
gear1.SetRot(ch.Vec3(0, 0, 0))

gear2 = ch.ChBody()
gear2.SetBodyType(ch.ChBodyType.KINEMATIC)
gear2.SetMass(1.0)
gear2.SetPos(ch.Vec3(1, 0, 0))
gear2.SetRot(ch.Vec3(0, 0, 0))


motor = ch.ChMotor()
motor.SetType(ch.ChMotorType.ROTATIONAL)
motor.SetSpeed(10.0)
motor.SetPos(ch.Vec3(0, 0, 0))


gear1.AddForce(ch.Vec3(0, 0, -1))
gear2.AddForce(ch.Vec3(0, 0, 1))


gear1.AddForce(motor.GetForce())


ch.AddBody(truss)
ch.AddBody(bar)
ch.AddBody(gear1)
ch.AddBody(gear2)


irr.SetMaterial(truss, irr.Material(1, 1, 1, 0.5, 0.5, 0.5))
irr.SetMaterial(bar, irr.Material(1, 0, 0, 0.5, 0.5, 0.5))
irr.SetMaterial(gear1, irr.Material(0, 1, 0, 0.5, 0.5, 0.5))
irr.SetMaterial(gear2, irr.Material(0, 0, 1, 0.5, 0.5, 0.5))

irr.SetLight(irr.Light(1, 1, 1, 10, 10, 10))


ch.StartSimulation()


while True:
    ch.StepSimulation(0.01)
    irr.Update()