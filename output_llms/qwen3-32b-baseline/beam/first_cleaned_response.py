import pychrono as chrono
import pychrono.irrlicht as irr
import math


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, 0))  


mesh = chrono.fea.ChMesh()


material = chrono.fea.ChMaterialBeamEuler()
material.SetYoungModulus(2e11)  
material.SetGshearModulus(0.769e11)  
material.SetDensity(7800)  


d = 0.1
area = (math.pi * d**2) / 4
Iyy = (math.pi * d**4) / 64
Izz = Iyy
J = (math.pi * d**4) / 32  

section = chrono.fea.ChBeamSectionEuler()
section.SetArea(area)
section.SetIyy(Iyy)
section.SetIzz(Izz)
section.SetGJ(J)  


node0 = chrono.fea.ChNodeFEAxyz(chrono.ChVectorD(0, 0, 0))
node1 = chrono.fea.ChNodeFEAxyz(chrono.ChVectorD(1, 0, 0))
node2 = chrono.fea.ChNodeFEAxyz(chrono.ChVectorD(2, 0, 0))

mesh.AddNode(node0)
mesh.AddNode(node1)
mesh.AddNode(node2)


element0 = chrono.fea.ChElementBeamEuler()
element0.SetNodes(node0, node1)
element0.SetMaterial(material)
element0.SetSection(section)
mesh.AddElement(element0)

element1 = chrono.fea.ChElementBeamEuler()
element1.SetNodes(node1, node2)
element1.SetMaterial(material)
element1.SetSection(section)
mesh.AddElement(element1)


fixed_link = chrono.ChLinkDirFrame()
fixed_link.Initialize(node0, chrono.ChFrameD())
fixed_link.SetConstrained(True)
system.AddLink(fixed_link)


node2.AddForce(chrono.ChVectorD(0, -1000, 0))  


system.Add(mesh)


vis = chrono.fea.ChVisualShapeFEA(mesh)
vis.SetFEMdataType(chrono.fea.ChVisualShapeFEA.DataType.ELEMENTS)
vis.SetColorscaleMinMax(-1, 1)
vis.SetSmoothFaces(True)
vis.SetThickness(0.1)  
mesh.AddVisualShape(vis)


app = irr.ChIrrApp(system, 'Beam FEA Simulation', irr.dimension2du(800, 600))
app.SetSkyBox()
app.SetCamera(chrono.ChVectorD(0, 5, -10), chrono.ChVectorD(1, 0, 0), chrono.ChVectorD(0, 1, 0))
app.SetAmbientLight(chrono.ChColor(0.4, 0.4, 0.4))
app.AddTypicalLights()
app.AssetBindAll()
app.AssetUpdateAll()


app.SetTimestep(0.001)
while app.GetDevice().run():
    app.BeginScene()
    app.DrawAll()
    system.DoStepDynamics(app.GetTimestep())
    app.EndScene()