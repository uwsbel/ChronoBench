import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np






system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  







beam_length = 2.0
num_segments = 20
segment_length = beam_length / num_segments
beam_radius = 0.02
beam_mass = 0.1


cable = chrono.ChCableANCF()
cable.Set_Num_Segments(num_segments)
cable.Set_Segment_Length(segment_length)
cable.Set_Radius(beam_radius)
cable.Set_Mass(beam_mass)


system.Add(cable)







fixed_body = chrono.ChBodyEasy()
fixed_body.SetBodyFixed(True)
fixed_body.SetPos(chrono.ChVectorD(0, 0, 0))
system.Add(cable)
system.Add(fixed_body)


connection = chrono.ChLinkMateRevolute()
connection.Initialize(fixed_body, cable, chrono.ChCoordsys(chrono.ChVectorD(0, 0, 0)))
system.Add(connection)







system.SetTimestep(0.005)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('ANCF Cable Beam')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(beam_length/2, beam_length/2, -beam_length))
vis.AddTypicalLights()






while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(0.005)