import pychrono as chrono
from pychrono import fea, chronoirr


system = chrono.ChSystemSMC()
mesh = fea.ChMesh()



node0 = fea.ChNodeFEAxyz(chrono.ChVectorD(0, 0, 0))
node1 = fea.ChNodeFEAxyz(chrono.ChVectorD(1, 0, 0))
node2 = fea.ChNodeFEAxyz(chrono.ChVectorD(2, 0, 0))
mesh.AddNode(node0)
mesh.AddNode(node1)
mesh.AddNode(node2)


section = fea.ChBeamSectionCircular()
section.SetYoungModulus(2e11)  
section.SetGshearModulus(0.75 * 2e11 / (1 + 0.3))  
section.SetDensity(7800)  
section.SetRadius(0.05)  


element1 = fea.ChElementBeamEuler()
element1.SetNodes(node0, node1)
element1.SetSection(section)
mesh.AddElement(element1)

element2 = fea.ChElementBeamEuler()
element2.SetNodes(node1, node2)
element2.SetSection(section)
mesh.AddElement(element2)


system.Add(mesh)


ground = chrono.ChBody()
system.AddBody(ground)
ground.SetFixed(True)
link = chrono.ChLinkLockPointPoint()
link.Initialize(node0, ground, chrono.ChCoordsysD(node0.GetPos()))
system.AddLink(link)


node2.SetForce(chrono.ChVectorD(0, -1000, 0))


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("PyChrono Beam FEA Simulation")
vis.Initialize()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(3, 3, 3), chrono.ChVectorD(1, 0, 0))
vis.AddTypicalLights()


mesh_shape = fea.ChVisualShapeFEA(mesh)
mesh_shape.SetFEMdataType(fea.ChVisualShapeFEA.DataType_ELEM_BEAM_MID)
mesh_shape.SetColorscaleMinMax(-0.1, 0.1)
mesh_shape.SetSmoothFaces(True)
mesh.AddVisualShape(mesh_shape)


coord_system = chrono.ChVisualShapeBox(0.1, 0.1, 0.1)
coord_system.SetMutable(False)
coord_system.SetColor(chrono.ChColor(0.8, 0.8, 0.8))
system.AddVisualShape(coord_system, chrono.ChFramed(chrono.ChVectorD(0, 0, 0)))


while vis.Run():
    vis.BeginScene()
    vis.Render()
    system.DoStepDynamics(0.001)
    vis.EndScene()