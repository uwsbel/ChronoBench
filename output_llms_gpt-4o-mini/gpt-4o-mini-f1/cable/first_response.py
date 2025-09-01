import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as irr
import math

# Create the Chrono simulation system
sys = chrono.ChSystemSMC()

# Create a mesh object to visualize the beam
mesh = chrono.ChTriangleMeshConnected()
mesh.setFaceShareVertex(True)

# Create a beam object using ANCF cable elements
beam = fea.ChBeamANCF_333()
beam.SetDensity(1000)
beam.SetYoungModulus(2e11)
beam.SetShearModulus(8e10)
beam.SetRotInertia(1e-6)
beam.SetSectionThickness(0.01)
beam.SetSectionWidth(0.01)
beam.SetSectionHeight(0.01)

# Create nodes for the beam
nodeA = fea.ChNodeFEAxyzrot()
nodeB = fea.ChNodeFEAxyzrot()
nodeC = fea.ChNodeFEAxyzrot()
nodeD = fea.ChNodeFEAxyzrot()

# Set positions and orientations for the nodes
nodeA.SetPos(chrono.ChVector3d(0, 0, 0))
nodeA.SetRot(chrono.ChQuaterniond(1, 0, 0, 0))
nodeB.SetPos(chrono.ChVector3d(1, 0, 0))
nodeB.SetRot(chrono.ChQuaterniond(1, 0, 0, 0))
nodeC.SetPos(chrono.ChVector3d(2, 0, 0))
nodeC.SetRot(chrono.ChQuaterniond(1, 0, 0, 0))
nodeD.SetPos(chrono.ChVector3d(3, 0, 0))
nodeD.SetRot(chrono.ChQuaterniond(1, 0, 0, 0))

# Add nodes to the beam
beam.AddNode(nodeA)
beam.AddNode(nodeB)
beam.AddNode(nodeC)
beam.AddNode(nodeD)

# Create sections for the beam
sectionAB = fea.ChBeamSectionCable()
sectionBC = fea.ChBeamSectionCable()
sectionCD = fea.ChBeamSectionCable()

# Set properties for the sections
sectionAB.SetDiameter(0.01)
sectionAB.SetYoungModulus(2e11)
sectionAB.SetShearModulus(8e10)
sectionBC.SetDiameter(0.01)
sectionBC.SetYoungModulus(2e11)
sectionBC.SetShearModulus(8e10)
sectionCD.SetDiameter(0.01)
sectionCD.SetYoungModulus(2e11)
sectionCD.SetShearModulus(8e10)

# Add sections to the beam
beam.AddSection(sectionAB, 0, 1)
beam.AddSection(sectionBC, 1, 2)
beam.AddSection(sectionCD, 2, 3)

# Create a mesh for the beam
beam_mesh = fea.ChMesh()
beam_mesh.AddBeamSection(beam)

# Add the mesh to the system
sys.Add(beam_mesh)

# Create a node for the hinge constraint
nodeHinge = fea.ChNodeFEAxyzrot()
nodeHinge.SetPos(chrono.ChVector3d(0, 0, 0))
nodeHinge.SetRot(chrono.ChQuaterniond(1, 0, 0, 0))

# Create a hinge constraint
hinge = fea.ChLinkNodeFEAxyzrot()
hinge.Initialize(nodeHinge, nodeA, False)

# Add the hinge to the mesh
beam_mesh.Add(hinge)

# Create a force to apply to the beam
force = fea.ChLoadBeamSelfWeight(beam, chrono.ChVector3d(0, -9.81, 0))
beam_mesh.Add(force)

# Create the Irrlicht visualization
vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(chrono.ChVector2i(1024, 768))
vis.SetWindowTitle('Visualization of ANCF beam')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 1, 3), chrono.ChVector3d(1, 0, 0))
vis.AddTypicalLights()

# Create a material for the mesh
material = chrono.ChVisualMaterial()
material.SetDiffuseColor(chrono.ChColor(0.6, 0.6, 0.6))
material.SetSpecularColor(chrono.ChColor(0.3, 0.3, 0.3))
mesh.AddTriangleMesh(material, True, True)

# Simulation loop
while vis.Run():
    # Update the visualization
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Get the current time
    time = sys.GetChTime()

    # Update the mesh with the current node positions
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

    # Update the system
    sys.DoStepDynamics(1e-3)

    # Visualize the nodal positions
    for i in range(beam.GetNnodes()):
        node = beam.GetNode(i)
        vis.DrawPoint(node.GetPos(), chrono.ChColor(1, 0, 0), 0.01)

    # Print the current time
    print('Time:', time)