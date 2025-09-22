import numpy as np
from chrono import chrono_types
from chrono import chrono_sys
from chrono import chrono_fea
from chrono import chrono_irrlicht
from chrono import chrono_animation


chrono_types.InitChrono()


system = chrono_sys.ChSystem()


irr_app = chrono_irrlicht.ChIrrApp(system, "ANCF Cable Beam Simulation")


system.Set_G_acc(chrono_types.ChVector3d(0, -9.81, 0))


beam_mesh = chrono_fea.ChFEAmesh()


beam_length = 10.0  
beam_diameter = 0.1  
young_modulus = 2.1e9  
density = 7850  


num_elements = 10
num_nodes = num_elements + 1
for i in range(num_elements):
    node0 = chrono_fea.ChNodeFEAbase(chrono_types.ChVector3d(i * beam_length / num_elements, 0, 0))
    node1 = chrono_fea.ChNodeFEAbase(chrono_types.ChVector3d((i + 1) * beam_length / num_elements, 0, 0))
    
    
    element = chrono_fea.ChElementBeamANCF(beam_diameter, young_modulus, density)
    element.SetNodes(node0, node1)
    beam_mesh.AddElement(element)


for i in range(num_nodes):
    node = chrono_fea.ChNodeFEAbase(chrono_types.ChVector3d(i * beam_length / num_elements, 0, 0))
    beam_mesh.AddNode(node)


fix_node = beam_mesh.GetNode(0)
fix_node.SetFixed(True)


system.Add(beam_mesh)


irr_app.AssetBindAll()
irr_app.AssetUpdateAll()


while irr_app.Run():
    system.DoStepDynamics(0.01)


chrono_types.ChronoFinalize()