import pychrono as chrono
import numpy as np


def create_beam_system():
    
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

    
    mat_vis = chrono.ChVisualMaterial()
    mat_vis.SetDiffuseColor(chrono.ChColor(0.3, 0.6, 0.9))

    
    beam = chrono.ChBodyEasyBox(0.02, 0.4, 0.02, 1000, True, True)
    beam.SetName("Buckling beam")
    beam.SetMass(1.0)
    beam.SetInertiaXX(chrono.ChVector3d(0.001, 0.001, 0.001))
    beam.SetPos(chrono.ChVector3d(0, 0, 0))
    beam.SetFixed(True)
    beam.GetVisualShape(0).SetColor(mat_vis.GetDiffuseColor())
    system.Add(beam)

    
    support = chrono.ChBodyEasyBox(0.2, 0.2, 0.2, 1000, True, True)
    support.SetName("Buckling support")
    support.SetMass(10.0)
    support.SetInertiaXX(chrono.ChVector3d(0.01, 0.01, 0.01))
    support.SetPos(chrono.ChVector3d(0, -0.5, 0))
    support.SetFixed(True)
    support.GetVisualShape(0).SetColor(chrono.ChColor(0.8, 0.8, 0.8))
    system.Add(support)

    
    spring = chrono.ChLinkTSDA()
    spring.SetName("Buckling spring-damper")
    spring.Initialize(beam, support, True, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 0, 0))
    spring.SetRestLength(0.4)
    spring.SetSpringCoefficient(10000)
    spring.SetDampingCoefficient(100)
    system.Add(spring)

    
    motor = chrono.ChLinkMotorRotationSpeed()
    motor.SetName("Buckling motor")
    motor.Initialize(beam, support, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
    motor.SetMotorFunction(chrono.ChFunctionConst(0.0))
    system.Add(motor)

    
    beam_helper = chrono.ChVisualShapeBox(0.02, 0.4, 0.02)
    beam_helper.SetColor(mat_vis.GetDiffuseColor())
    beam_helper.SetOpacity(0.4)
    beam.AddVisualShape(beam_helper)

    return system, beam, support, spring, motor


def custom_motor_function(time):
    
    return 0.1 * np.sin(2 * np.pi * time)


def run_simulation(duration=5.0, step=0.001):
    system, beam, support, spring, motor = create_beam_system()
    motor.SetMotorFunction(chrono.ChFunctionConst(0.0))
    motor.SetMotorFunction(chrono.ChFunctionCustom(10, 0, 0, custom_motor_function))

    while system.GetChTime() < duration:
        system.DoStepDynamics(step)
        print(f"Time: {system.GetChTime():.3f} | Beam position: {beam.GetPos():.3f} | Spring force: {spring.GetForce():.3f}")


if __name__ == "__main__":
    run_simulation()