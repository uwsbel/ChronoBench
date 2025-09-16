import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as irr


sys = chrono.ChSystemNSC()


mesh = fea.ChMesh()
sys.Add(mesh)


E = 210e9  
nu = 0.3   
density = 7800  


length = 1.0
height = 0.02
width = 0.01
area = height * width
Iyy = (width * height**3) / 12
Izz = (height * width**3) / 12
J = Iyy + Izz


section = fea.ChBeamSectionAdvanced()
section.SetArea(area)
section.SetIyy(Iyy)
section.SetIzz(Izz)
section.SetJ(J)
section.SetE(E)
section.SetG(E / (2 * (1 + nu)))
section.SetDensity(density)


num_elements = 10
nodes = []
for i in range(num_elements + 1):
    pos = chrono.ChVectorD(i * length / num_elements, 0, 0)
    node = fea.ChNodeFEAxyzrot(chrono.ChFrameD(pos))
    mesh.AddNode(node)
    nodes.append(node)

for i in range(num_elements):
    element = fea.ChElementBeamEuler()
    element.SetNodes(nodes[i], nodes[i+1])
    element.SetSection(section)
    mesh.AddElement(element)


nodes[0].SetFixed(True)


load = fea.ChLoadNodeForce(nodes[-1])
mesh.AddLoad(load)


sys.SetSolverType(chrono.ChSolver.Type_MINRES)
sys.SetSolverMaxIterations(100)
sys.SetTimestepperType(chrono.ChTimestepper.Type_HHT)


vis = irr.ChIrrApp(sys, 'Beam Buckling Simulation', irr.dimension2du(1024, 768))
vis.AddTypicalSky()
vis.AddTypicalLights()
vis.CameraSetPosition(chrono.ChVectorD(0, 1, -2))


mesh_vis = fea.ChVisualizationFEAmesh(mesh)
mesh_vis.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_SURFACE)
mesh_vis.SetColorscaleMinMax(-0.1, 0.1)
mesh.AddAsset(mesh_vis)

vis.AssetBindAll()
vis.AssetUpdateAll()


while vis.GetDevice().run():
    
    time = sys.GetChTime()
    force_magnitude = 1000 * min(time, 1.0)  
    load.SetForce(chrono.ChVectorD(-force_magnitude, 0, 0))
    
    sys.DoStepDynamics(0.001)
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()