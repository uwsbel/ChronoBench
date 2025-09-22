import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np


chrono.SetChronoDataPath('PATH_TO_CHRONO_DATA')  
sys = chrono.ChSystemNSC()  
sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  



num_elements = 5
beam_length = 0.5  
beam_width = 0.05  
beam_height = 0.05  
density = 7800     


nodes = []
for i in range(num_elements + 1):
    x_pos = i * beam_length / num_elements
    node = chrono.ChNodeFEAxyzrot()
    node.SetPos(chrono.ChVectorD(x_pos, 0, 0))
    node.SetMass(beam_width * beam_height * beam_length/num_elements * density / (num_elements + 1))
    node.SetInertiaXX(chrono.ChVectorD(
        (beam_width**2 + beam_height**2) * beam_length/num_elements * density / 12,
        beam_width**2 * beam_length/num_elements * density / 12,
        beam_height**2 * beam_length/num_elements * density / 12
    ))
    sys.Add(node)
    nodes.append(node)


for i in range(num_elements):
    beam = chrono.ChElementBeamEuler()
    beam.SetNodes(nodes[i], nodes[i+1])

    
    beam.SetBeamSection(chrono.ChBeamSectionAdvanced())
    beam.GetBeamSection().SetAsRectangularSection(beam_width, beam_height)
    beam.GetBeamSection().SetYoungModulus(2.1e11)  
    beam.GetBeamSection().SetShearModulus(7.9e10)  
    beam.GetBeamSection().SetPoissonRatio(0.3)     
    beam.GetBeamSection().SetDensity(density)

    sys.Add(beam)


fix_constraint = chrono.ChLinkLockLock()
fix_constraint.Initialize(nodes[0], chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
sys.Add(fix_constraint)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('Beam FEA Simulation')
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(1, 0.5, 1), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVectorD(1.5, 5, 1.5), chrono.ChVectorD(0, 0, 0), 3, 0.5, 5, 30, 512)


time_step = 0.001
vis.SetStep(0.01)  
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    sys.DoStepDynamics(time_step)