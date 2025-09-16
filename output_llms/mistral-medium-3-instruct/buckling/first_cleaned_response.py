import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np


chrono.SetChronoDataPath('path_to_chrono_data')  


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  




beam_length = 2.0
beam_width = 0.1
beam_height = 0.1
num_elements = 10


beam_mesh = chrono.ChMesh()
beam_mesh.SetAutomaticGravity(False)


nodes = []
for i in range(num_elements + 1):
    x = i * beam_length / num_elements
    node = chrono.ChNodeFEAxyzrot()
    node.SetPos(chrono.ChVectorD(x, 0, 0))
    node.SetMass(0.1)  
    beam_mesh.AddNode(node)
    nodes.append(node)


for i in range(num_elements):
    element = chrono.ChElementBeamANCF()
    element.SetNodes(nodes[i], nodes[i+1])
    element.SetDimensions(beam_width, beam_height)
    element.SetYoungModulus(2.1e11)  
    element.SetPoissonRatio(0.3)
    element.SetDensity(7850)  
    beam_mesh.AddElement(element)


fix_constraint = chrono.ChLinkLockLock()
fix_constraint.Initialize(nodes[0], chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
system.AddLink(fix_constraint)


force = chrono.ChForce()
force.SetMode(chrono.ChForce.FORCE)
force.SetF_force(chrono.ChVectorD(0, 0, -1000))  
nodes[-1].SetForce(force)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('Beam Buckling Simulation')
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AddCamera(chrono.ChVectorD(0, 1, 3), chrono.ChVectorD(0, 0, 0))
vis.SetCameraMoveScale(0.01)


beam_asset = chrono.ChVisualizationFEAmesh(beam_mesh)
beam_asset.SetFEMdataType(chrono.ChVisualizationFEAmesh.E_PLOT_NODE_DISPLACEMENT)
beam_asset.SetFEMglyphType(chrono.ChVisualizationFEAmesh.E_GLYPH_NODE_DOT_POS)
beam_asset.SetFEMscaling(0.1)
beam_asset.SetWireframe(True)
beam_mesh.AddAsset(beam_asset)


system.SetSolverType(chrono.ChSolver.Type_PSSOR)
system.SetTimestepperType(chrono.ChTimestepper.Type_HHT)
system.SetMaxItersSolverSpeed(100)
system.SetMaxItersSolverStab(100)
system.SetTol(1e-10)
system.SetMaxPenetrationRecoverySpeed(1.0)


time_step = 0.001
simulation_time = 5.0
output_fps = 30
output_step = 1.0 / output_fps


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    system.DoStepDynamics(time_step)

    current_time = system.GetChTime()
    if current_time >= simulation_time:
        break

    
    if int(current_time % 1) == 0:
        print(f"Time: {current_time:.2f}s")
        print(f"Tip displacement: {nodes[-1].GetPos().z:.4f}m")