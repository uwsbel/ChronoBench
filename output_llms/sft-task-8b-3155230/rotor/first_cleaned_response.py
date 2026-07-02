import pychrono as chrono
from pychrono import irrlicht
import numpy as np


system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))


visual_system = irrlicht.ChVisualSystemIrrlicht()
visual_system.SetWindowSize(1024, 768)
visual_system.SetWindowTitle("Jeffcott Rotor IGA Beam Simulation")
visual_system.Initialize()
visual_system.AddSkyBox()
visual_system.AddCamera(chrono.ChFogMode.NOFOG, 0.1, 0.9, 0.9, 0.9, 0.9, 0.9)
visual_system.AddLightDirectional(chrono.ChVector3d(0, -1, 0), chrono.ChColor(0.8, 0.8, 0.8))


beam_length = 2.0
beam_radius = 0.02
young_modulus = 210e9  
poisson_ratio = 0.3
density = 7800  
num_elements = 10
num_control_points = num_elements + 1


beam = chrono.ChIgaBeam()
beam.SetName("Jeffcott Rotor Beam")
beam.SetYoungModulus(young_modulus)
beam.SetPoissonRatio(poisson_ratio)
beam.SetDensity(density)
beam.SetCrossSection(chrono.ChVector3d(beam_radius * 2, beam_radius * 2, 0.01))


knots = np.linspace(0, 1, num_control_points)
beam.SetKnotVector(knots)


control_points = []
for i in range(num_control_points):
    angle = 2 * np.pi * i / num_control_points
    x = beam_length * np.cos(angle)
    y = beam_length * np.sin(angle)
    control_points.append(chrono.ChVector3d(x, y, 0))
beam.SetControlPoints(control_points)


system.AddBody(beam)


flywheel = chrono.ChBodyEasySphere(0.15, 1000, True, False)
flywheel.SetName("Jeffcott Rotor Flywheel")
flywheel.SetMass(10.0)
flywheel.SetInertiaXX(chrono.ChVector3d(0.05, 0.05, 0.05))
flywheel.SetPos(chrono.ChVector3d(beam_length / 2, 0, 0))
flywheel.SetRot(chrono.QuatFromAngleZ(0))
system.AddBody(flywheel)


motor = chrono.ChLinkMotorRotationAngle()
motor.SetName("Jeffcott Rotor Motor")
motor.Initialize(beam, chrono.ChFramed(chrono.ChVector3d(beam_length, 0, 0), chrono.QuatFromAngleZ(0)), chrono.ChFramed(chrono.ChVector3d(beam_length, 0, 0), chrono.QuatFromAngleZ(0)))
motor.SetAngleFunction(chrono.ChFunctionConst(0))
system.AddLink(motor)


visual_system.AddSphere(flywheel, chrono.ChColor(0.8, 0.2, 0.2))


time_step = 1e-3
simulation_time = 5.0
num_steps = int(simulation_time / time_step)


for step in range(num_steps):
    time = step * time_step
    motor.SetAngleFunction(chrono.ChFunctionConst(time * 20))  

    system.DoStepDynamics(time_step)
    visual_system.BeginScene()
    visual_system.Render()
    visual_system.EndScene()

visual_system.Close()