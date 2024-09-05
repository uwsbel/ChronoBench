import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as irr
import math


sys = chrono.ChSystemSMC()


mesh = chrono.ChTriangleMeshConnected()
mesh.setFaceShareVertex(True)


beam = fea.ChBeamANCF_333()
beam.SetDensity(1000)
beam.SetYoungModulus(2e11)
beam.SetShearModulus(8e10)
beam.SetRotInertia(1e-6)
beam.SetSectionThickness(0.01)
beam.SetSectionWidth(0.01)
beam.SetSectionHeight(0.01)


nodeA = fea.ChNodeFEAxyzrot()
nodeB = fea.ChNodeFEAxyzrot()
nodeC = fea.ChNodeFEAxyzrot()
nodeD = fea.ChNodeFEAxyzrot()


nodeA.SetPos(chrono.ChVector3d(0, 0, 0))
nodeA.SetRot(chrono.ChQuaterniond(1, 0, 0, 0))
nodeB.SetPos(chrono.ChVector3d(1, 0, 0))
nodeB.SetRot(chrono.ChQuaterniond(1, 0, 0, 0))
nodeC.SetPos(chrono.ChVector3d(2, 0, 0))
nodeC.SetRot(chrono.ChQuaterniond(1, 0, 0, 0))
nodeD.SetPos(chrono.ChVector3d(3, 0, 0))
nodeD.SetRot(chrono.ChQuaterniond(1, 0, 0, 0))


beam.AddNode(nodeA)
beam.AddNode(nodeB)
beam.AddNode(nodeC)
beam.AddNode(nodeD)


sectionAB = fea.ChBeamSectionCable()
sectionBC = fea.ChBeamSectionCable()
sectionCD = fea.ChBeamSectionCable()


sectionAB.SetDiameter(0.01)
sectionAB.SetYoungModulus(2e11)
sectionAB.SetShearModulus(8e10)
sectionBC.SetDiameter(0.01)
sectionBC.SetYoungModulus(2e11)
sectionBC.SetShearModulus(8e10)
sectionCD.SetDiameter(0.01)
sectionCD.SetYoungModulus(2e11)
sectionCD.SetShearModulus(8e10)


beam.AddSection(sectionAB, 0, 1)
beam.AddSection(sectionBC, 1, 2)
beam.AddSection(sectionCD, 2, 3)


beam_mesh = fea.ChMesh()
beam_mesh.AddBeamSection(beam)


sys.Add(beam_mesh)


nodeHinge = fea.ChNodeFEAxyzrot()
nodeHinge.SetPos(chrono.ChVector3d(0, 0, 0))
nodeHinge.SetRot(chrono.ChQuaterniond(1, 0, 0, 0))


hinge = fea.ChLinkNodeFEAxyzrot()
hinge.Initialize(nodeHinge, nodeA, False)


beam_mesh.Add(hinge)


force = fea.ChLoadBeamSelfWeight(beam, chrono.ChVector3d(0, -9.81, 0))
beam_mesh.Add(force)


vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(chrono.ChVector2i(1024, 768))
vis.SetWindowTitle('Visualization of ANCF beam')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 1, 3), chrono.ChVector3d(1, 0, 0))
vis.AddTypicalLights()


material = chrono.ChVisualMaterial()
material.SetDiffuseColor(chrono.ChColor(0.6, 0.6, 0.6))
material.SetSpecularColor(chrono.ChColor(0.3, 0.3, 0.3))
mesh.AddTriangleMesh(material, True, True)


while vis.Run():
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    time = sys.GetChTime()

    
    mesh.Clear()
    for i in range(beam.GetNnodes()):
        node = beam.GetNode(i)
        mesh.AddVertex(node.GetPos())
    for i in range(beam.GetNsections()):
        section = beam.GetSection(i)
        for j in range(section.GetNvertices()):
            mesh.AddTriangle(
                section.GetNodeA() + j,
                section.GetNodeB() + j,
                section.GetNodeC() + j
            )
    mesh.SetVisible(True)
    mesh.UpdateNormales()

    
    sys.DoStepDynamics(1e-3)

    
    for i in range(beam.GetNnodes()):
        node = beam.GetNode(i)
        vis.DrawPoint(node.GetPos(), chrono.ChColor(1, 0, 0), 0.01)

    
    print('Time:', time)