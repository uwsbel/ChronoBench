import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea
import numpy as np


chrono.SetChronoDataPath("PATH_TO_CHRONO_DATA")  


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  



beam_length = 2.0
num_elements = 10
diameter = 0.1


beam = fea.ChBeamANCF()
beam.SetNumElements(num_elements)
beam.SetElementsFromFile("beam.txt")  


material = fea.ChMaterialShellANCF()
material.SetYoungModulus(2.1e11)
material.SetPoissonRatio(0.3)
material.SetDensity(7800)
beam.SetMaterial(material)


beam.SetDiameter(diameter)


fixed_node = beam.GetNode(0)
fixed_node.SetFixed(True)


system.Add(beam)



vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(800, 600)
vis.SetWindowTitle("ANCF Beam Simulation")
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 0, 5))
vis.AddTypicalLights()
vis.AttachSystem(system)


beam_asset = chronoirr.ChBeamVisualizationIrrlicht(beam)
vis.AddVisualModel(beam_asset)


time_step = 0.01
sim_time = 5.0
current_time = 0

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    system.DoStepDynamics(time_step)
    current_time += time_step

    
    if int(current_time % 1) == 0:
        print("Time:", current_time)
        print("Node 0 position:", beam.GetNode(0).GetPos())
        print("Node -1 position:", beam.GetNode(-1).GetPos())

    if current_time > sim_time:
        break


vis.Close()