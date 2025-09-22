import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np


chrono.SetChronoDataPath('PATH_TO_CHRONO_DATA')  


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  



beam_length = 1.0  
num_elements = 10
beam_radius = 0.02  


density = 7800  
E = 200e9  
nu = 0.3  
G = E / (2 * (1 + nu))  


nodes = []
for i in range(num_elements + 1):
    x = i * beam_length / num_elements
    node = chrono.ChNodeFEAxyzrot()
    node.SetPos(chrono.ChVectorD(x, 0, 0))
    node.SetMass(0)  
    system.Add(node)
    nodes.append(node)


beam_elements = []
for i in range(num_elements):
    node1 = nodes[i]
    node2 = nodes[i+1]

    
    length = beam_length / num_elements
    volume = np.pi * beam_radius**2 * length
    mass = volume * density
    Ixx = np.pi * beam_radius**4 / 4
    Iyy = Ixx
    Izz = Ixx * 2

    
    beam = chrono.ChElementBeamANCF()
    beam.SetNodes(node1, node2)
    beam.SetBeamGeometry(chrono.ChVectorD(beam_radius, beam_radius, beam_radius))
    beam.SetMaterialProperties(density, E, G)

    
    beam.SetInertiaProperties(chrono.ChVectorD(Ixx, Iyy, Izz), mass)

    system.Add(beam)
    beam_elements.append(beam)


system.AddConstraint(chrono.ChLinkLockLock(),
                   chrono.ChFrameD(nodes[0].GetPos()),
                   chrono.ChFrameD(nodes[0].GetPos()))


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('Beam FEA Simulation')
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0.5, 0.5, 2))
vis.AddTypicalLights()


for beam in beam_elements:
    vis.AddFEMmesh(beam.GetBeamGeometry(), chrono.ChColor(0.5, 0.5, 1.0))
    vis.AddFEMnodes(beam.GetNodes(), 0.05, chrono.ChColor(1.0, 0.0, 0.0))


time_step = 0.01
vis.SetTimeStep(time_step)

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    system.DoStepDynamics(time_step)
    vis.SetTime(system.GetChTime())