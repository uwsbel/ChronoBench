import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

# --------------------------------------------------------------------
# Create the Chrono system and set default parameters
# --------------------------------------------------------------------
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# Set simulation step size
system.SetTimestepperType(chrono.ChTimestepper.Type_HHT)
system.SetStep(0.01)

# --------------------------------------------------------------------
# Create the ANCF beam
# --------------------------------------------------------------------
beam_length = 10
num_elements = 20
element_length = beam_length / num_elements

# Create the ANCF cable material
material = fea.ChMaterialCableANCF(system)
material.SetDensity(7850)  # Steel density
material.SetYoungModulus(200e9)  # Steel Young's modulus
material.SetBeamRaleyghDamping(0.001)

# Create the ANCF beam section
section = fea.ChBeamSectionCable(material, 0.01)  # Radius of 0.01m

# Create the ANCF beam element
element = fea.ChElementCableANCF()
element.SetSection(section)

# Create the mesh and add the nodes and elements
mesh = fea.ChMesh()
nodes = []
for i in range(num_elements + 1):
    node = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD(i * element_length, 0, 0)))
    nodes.append(node)
    mesh.AddNode(node)

for i in range(num_elements):
    element = fea.ChElementCableANCF()
    element.SetNodes(nodes[i], nodes[i + 1])
    element.SetSection(section)
    mesh.AddElement(element)

# Fix the first node to create a hinge
nodes[0].SetFixed(True)

# Add gravity to the nodes
for node in nodes:
    node.SetMass(0.1)  # Small mass for visualization purposes

# Add the mesh to the system
system.Add(mesh)

# --------------------------------------------------------------------
# Create the Irrlicht visualization
# --------------------------------------------------------------------
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('ANCF Beam Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(beam_length / 2, beam_length / 4, -beam_length), chrono.ChVectorD(beam_length / 2, 0, 0))
vis.AddTypicalLights()

# --------------------------------------------------------------------
# Simulation loop
# --------------------------------------------------------------------
while vis.Run():
    system.DoStepDynamics(system.GetStep())

    # Render the simulation
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()

# Close the Irrlicht window
vis.Close()