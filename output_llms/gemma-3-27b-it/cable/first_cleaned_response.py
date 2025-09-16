import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math






system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  







beam_length = 2.0
num_segments = 20
segment_length = beam_length / num_segments
mass_per_segment = 0.1
young_modulus = 1e7
cross_section_area = 1e-4


body = chrono.ChBodyEasy()
body.SetBodyFixed(False)
system.Add(body)

prev_pos = chrono.ChVectorD(0, 0, 0)
for i in range(num_segments):
    
    cable = chrono.ChLinkCableANCF()
    cable.Initialize(body, body, chrono.ChCoordsys(prev_pos, chrono.ChQuaternionD(1, 0, 0, 0)),
                     chrono.ChCoordsys(prev_pos + chrono.ChVectorD(0, segment_length, 0), chrono.ChQuaternionD(1, 0, 0, 0)))

    cable.Set_YoungModulus(young_modulus)
    cable.Set_CrossSectionArea(cross_section_area)
    cable.Set_RestLength(segment_length)
    cable.Set_Mass(mass_per_segment)

    system.Add(cable)
    prev_pos += chrono.ChVectorD(0, segment_length, 0)


body.SetPos(chrono.ChVectorD(0,0,0))
body.SetTransform(chrono.ChCoordsys(chrono.ChVectorD(0,0,0), chrono.ChQuaternionD(1,0,0,0)))
body.Set_Fixed(True)







vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('ANCF Beam Simulation')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(beam_length/2, beam_length/2, -beam_length))
vis.AddTypicalLights()






while vis.Run():
    vis.BeginScene(True, True, chrono.ChColor(0.2, 0.2, 0.3))
    vis.Render()
    vis.EndScene()

    system.DoStepDynamics(0.001)