import pychrono as chrono
import pychrono.irrlicht as chronoirr






system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  






beam_length = 2.0
num_segments = 20
segment_length = beam_length / num_segments
mass_per_segment = 0.1
stiffness = 1000.0
damping = 10.0


body = chrono.ChBodyEasy()
body.SetBodyFixed(False)
system.Add(body)


prev_pos = chrono.ChVectorD(0, 0, 0)
body1 = chrono.ChBodyEasy()
body1.SetBodyFixed(True)  
body1.SetPos(prev_pos)
system.Add(body1)


link1 = chrono.ChLinkMateRevolute()
link1.Initialize(body1, body, chrono.ChCoordsys(prev_pos))
system.AddLink(link1)


for i in range(num_segments):
    current_pos = prev_pos + chrono.ChVectorD(segment_length, 0, 0)
    body = chrono.ChBodyEasy()
    body.SetMass(mass_per_segment)
    body.SetPos(current_pos)
    system.Add(body)

    
    link = chrono.ChLinkMateRevolute()
    link.Initialize(body1, body, chrono.ChCoordsys(current_pos))
    system.AddLink(link)
    body1 = body  
    prev_pos = current_pos






vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('ANCF Beam Simulation')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(beam_length / 2, beam_length / 2, -beam_length))
vis.AddTypicalLights()






time_step = 0.01
simulation_time = 10.0

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    system.DoStepDynamics(time_step)

    if system.GetChTime() > simulation_time:
        break