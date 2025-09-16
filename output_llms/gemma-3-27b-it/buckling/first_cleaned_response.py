import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea


chrono.SetChronoDataPath("../chrono_data/")  


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


beam_length = 1.0  
beam_height = 0.1  
beam_width = 0.05  
youngs_modulus = 2.0e11  
poissons_ratio = 0.3
density = 7850  
area = beam_height * beam_width
moment_of_inertia = (beam_width * beam_height**3) / 12.0


beam = fea.ChBeamSection(youngs_modulus, poissons_ratio, density, area, moment_of_inertia)
beam.Set_Plasticity(False)  


beam_body = chrono.ChBodyEasy()
beam_body.SetBodyFixed(False)
beam_body.SetMass(density * beam_length * area)
beam_body.AddAsset(chrono.ChVisualShapeEasy(chrono.ChBoxShape(), beam_width, beam_height, beam_length))
beam_body.SetPos(chrono.ChVectorD(0, 0.5, 0))
system.Add(beam_body)


beam_mesh = fea.ChBodyAuxiliary()
beam_mesh.AddSection(beam)
beam_mesh.CreateNodesAndElements(10, beam_length)  
beam_mesh.AttachBody(beam_body)
system.Add(beam_mesh)


support_left = chrono.ChBodyEasy()
support_left.SetBodyFixed(True)
support_left.SetPos(chrono.ChVectorD(-beam_length / 2, 0, 0))
support_left.AddAsset(chrono.ChVisualShapeEasy(chrono.ChBoxShape(), 0.1, 0.1, 0.1))
system.Add(support_left)

support_right = chrono.ChBodyEasy()
support_right.SetBodyFixed(True)
support_right.SetPos(chrono.ChVectorD(beam_length / 2, 0, 0))
support_right.AddAsset(chrono.ChVisualShapeEasy(chrono.ChBoxShape(), 0.1, 0.1, 0.1))
system.Add(support_right)



link_left = chrono.ChLinkRevolute()
link_left.Initialize(support_left, beam_body, chrono.ChCoordsys(chrono.ChVectorD(-beam_length/2, 0, 0)))
system.Add(link_left)

link_right = chrono.ChLinkRevolute()
link_right.Initialize(support_right, beam_body, chrono.ChCoordsys(chrono.ChVectorD(beam_length/2, 0, 0)))
system.Add(link_right)


force_magnitude = 100  
force_point = chrono.ChVectorD(0, beam_height/2, 0)
force_body = chrono.ChBodyEasy()
force_body.SetBodyFixed(True)
force_body.SetPos(force_point)
force_body.SetMass(0.01)
system.Add(force_body)

force_link = chrono.ChLinkForce()
force_link.Initialize(beam_body, force_body, False, force_point, chrono.ChVectorD(0, -force_magnitude, 0))
system.Add(force_link)


system.SetSolverType(chrono.ChSolver.Type_SOR)
system.SetSolverMaxIterations(100)
system.SetSolverTolerance(1e-6)

timestepper = chrono.ChTimestepper.Type_EULER_IMPLICIT
system.SetTimestepper(timestepper)
system.SetStep(0.001)  


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Beam Buckling Simulation')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(0, 2, -3))
vis.AddTypicalLights()


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics()