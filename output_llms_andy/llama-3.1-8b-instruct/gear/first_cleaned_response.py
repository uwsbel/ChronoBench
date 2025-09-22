import chrono as ch
import numpy as np


ch.InitCh chrono
chrono.SetChVisualize(True)


truss = ch.ChBodyEasyBox(1, 1, 0.1, 1)
truss.SetPos(ch.Vec3(0, 0, 0))
truss.SetRot(ch.Mat33(ch.Vec3(0, 0, 1)))
truss.SetMass(1)
truss.SetInertia(ch.MassInertia(1, 0, 0, 0))
truss.SetBodyFixed(True)


bar = ch.ChBodyEasyBox(0.1, 0.1, 1, 1)
bar.SetPos(ch.Vec3(0, 0, 1))
bar.SetRot(ch.Mat33(ch.Vec3(0, 0, 1)))
bar.SetMass(1)
bar.SetInertia(ch.MassInertia(1, 0, 0, 0))
bar.SetRotSpeed(ch.Vec3(0, 0, 1))
bar.SetRotAxis(ch.Vec3(0, 0, 1))


gear1 = ch.ChBodyEasyCylinder(0.1, 0.1, 1, 1)
gear1.SetPos(ch.Vec3(0, 0, 2))
gear1.SetRot(ch.Mat33(ch.Vec3(0, 0, 1)))
gear1.SetMass(1)
gear1.SetInertia(ch.MassInertia(1, 0, 0, 0))

gear2 = ch.ChBodyEasyCylinder(0.1, 0.1, 1, 1)
gear2.SetPos(ch.Vec3(0, 0, 3))
gear2.SetRot(ch.Mat33(ch.Vec3(0, 0, 1)))
gear2.SetMass(1)
gear2.SetInertia(ch.MassInertia(1, 0, 0, 0))


motor = ch.ChGearMotor()
motor.SetGear1(gear1)
motor.SetGear2(gear2)
motor.SetSpeed(1)
motor.SetTorque(1)


chrono.SetTimeStep(0.01)
chrono.SetTolerance(1e-6)


vis = ch.ChVisualizeIrrlicht()
vis.SetWindowTitle("Epicyclic Gears Simulation")
vis.SetWindowSize(800, 600)
vis.SetCameraPos(ch.Vec3(0, 0, 5))
vis.SetCameraRot(ch.Vec3(0, 0, 1))


vis.AddMaterial(ch.ChMaterialTextureDiffuse(ch.Vec3(1, 0, 0), ch.Vec3(1, 1, 1)))
vis.AddLight(ch.ChLightVolumetric(ch.Vec3(0, 0, 0), ch.Vec3(1, 1, 1), 1))


while chrono.GetSimulationTime() < 10:
    chrono.DoStep()
    vis.Draw()


chrono.Destroy()