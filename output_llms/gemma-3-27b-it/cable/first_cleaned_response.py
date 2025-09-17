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


node = chrono.ChBodyEasy()
node.SetBodyFixed(True)
node.SetPos(chrono.ChVectorD(0, 0, 0))
system.Add(node)


prev_node = node
for i in range(num_segments):
    next_node = chrono.ChBodyEasy()
    next_node.SetMass(mass_per_segment)
    next_node.SetPos(chrono.ChVectorD(0, segment_length * (i + 1), 0))

    
    cable = chrono.ChLinkANCF()
    cable.Initialize(prev_node, next_node, False, False)
    cable.Set_Spring_K(stiffness)
    cable.Set_Spring_R(damping)
    cable.Set_Rest_Length(segment_length)
    system.Add(cable)

    prev_node = next_node






vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('ANCF Beam Simulation')
vis.Initialize()
vis.AddTypicalCamera(chrono.ChVectorD(0, beam_length / 2, -3))
vis.AddTypicalLights()






time_step = 0.005
simulation_time = 10.0

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    system.DoStepDynamics(time_step)

    
    
    

    if system.GetChTime() > simulation_time:
        break





vis.Close()