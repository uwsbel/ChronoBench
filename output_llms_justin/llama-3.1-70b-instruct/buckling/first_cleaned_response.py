import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea
import numpy as np


chrono.SetChronoDataPath("./")


system = chrono.ChSystemNSC()


visualizer = chronoirr.ChVisualSystemIrrlicht()
visualizer.SetWindowSize(800, 600)
visualizer.SetWindowTitle("Beam Buckling Simulation")
visualizer.SetCameraDistance(2)
visualizer.SetCameraAzimuth(45)
visualizer.SetCameraElevation(30)


beam_length = 10  
beam_width = 0.1  
beam_height = 0.1  
beam_density = 7850  
beam_youngs_modulus = 2.1e11  
beam_poissons_ratio = 0.3  

beam = fea.ChBeamSectionEulerBernoulli()
beam.SetDensity(beam_density)
beam.SetYoungModulus(beam_youngs_modulus)
beam.SetPoissonsRatio(beam_poissons_ratio)
beam.SetWidth(beam_width)
beam.SetHeight(beam_height)


beam_mesh = fea.ChBeamMesh(beam_length, 10, beam)
beam_mesh.SetSection(beam)


beam_body = chrono.ChBodyEasyMesh(beam_mesh)
beam_body.SetMass(beam_density * beam_length * beam_width * beam_height)
beam_body.SetInertiaXX(chrono.ChVectorD(1, 1, 1))
beam_body.SetPos(chrono.ChVectorD(0, 0, 0))
beam_body.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))


system.Add(beam_body)


def motor_function(t):
    return 10 * np.sin(2 * np.pi * t)


motor = chrono.ChLinkMotorRotationSpeed()
motor.SetMotorFunction(chrono.ChFunction_Sine(motor_function))
motor.Initialize(beam_body, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
system.Add(motor)


constraint = chrono.ChLinkMateGeneric()
constraint.Initialize(beam_body, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
system.Add(constraint)


system.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED)
system.SetSolverType(chrono.ChSolver.Type_PSSOR)
system.SetStep(0.01)


while visualizer.Run():
    system.DoStepDynamics(0.01)
    visualizer.BeginScene()
    visualizer.DrawAll()
    visualizer.EndScene()