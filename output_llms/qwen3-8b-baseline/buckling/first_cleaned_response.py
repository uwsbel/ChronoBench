import pychrono as chrono
from pychrono import irrlicht
import numpy as np


system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVectorD(0, -9.81, 0))


beam_length = 2.0
beam_width = 0.1
beam_height = 0.1
young_modulus = 210e9  
poisson_ratio = 0.3


beam_section = chrono.ChBeamSectionRectangular(beam_width, beam_height, young_modulus, poisson_ratio)


beam_body = chrono.ChFlexibleBody()
beam_body.SetSection(beam_section)
beam_body.SetMass(beam_section.GetMass(beam_length))  
beam_body.SetPos(chrono.ChVectorD(0, 0, 0))
beam_body.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))


system.Add(beam_body)


constraint = chrono.ChPointConstraint(beam_body, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 0))
system.AddConstraint(constraint)


motor_force = chrono.ChFunctionLinear(0, 0, 1000)  


force = chrono.ChForce()
force.SetForce(chrono.ChVectorD(-1000, 0, 0))  
beam_body.AddForce(force)


vis = irrlicht.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraPosition(chrono.ChVectorD(0, 2, 5))
vis.SetCameraLookAt(chrono.ChVectorD(0, 0, 0))
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('Beam Buckling Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_small.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 2, 5), chrono.ChVectorD(0, 0, 0))
vis.AddLight(chrono.ChVectorD(0, 5, 5), 2.0)


system.SetTimestep(1e-3)
system.SetMaxIter(100)


beam_body.GetVisualShape().SetType(chrono.ChVisualShape.Type_MESH)
beam_body.GetVisualShape().SetMeshFile(chrono.GetChronoDataFile('beam.obj'))


time_end = 5.0
time_step = 1e-3
time = 0.0

while time < time_end:
    system.DoStepDynamics(time_step)
    
    force.SetForce(chrono.ChVectorD(motor_force.GetValue(time), 0, 0))
    beam_body.AddForce(force)
    vis.Render()
    time = system.GetChTime()