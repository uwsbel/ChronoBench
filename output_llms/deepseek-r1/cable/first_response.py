import chrono
import chrono.fea as fea
import chrono.irrlicht as irr
import math

# System setup
system = chrono.ChSystemSMC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Gravity in -Y direction

# Create FEA mesh
mesh = fea.ChMesh()
system.Add(mesh)

# Beam parameters
length = 2.0
n_elements = 10
radius = 0.05
area = math.pi * radius**2
I_yy = math.pi * radius**4 / 4  # Second moment of area
E = 1e7                         # Young's modulus
density = 1000                  # Material density
damping = 0.01                  # Rayleigh damping

# Create cable section
section = fea.ChBeamSectionCable()
section.SetArea(area)
section.SetI(I_yy)
section.SetYoungModulus(E)
section.SetDensity(density)
section.SetRayleighDamping(damping)

# Create nodes
nodes = []
for i in range(n_elements + 1):
    x = i * (length / n_elements)
    pos = chrono.ChVectorD(x, 0, 0)
    dir = chrono.ChVectorD(1, 0, 0)  # Initial direction vector
    node = fea.ChNodeFEAxyzD(pos, dir)
    node.SetMass(0)
    if i == 0:
        node.SetFixed(True)  # Fix first node (hinged support)
    mesh.AddNode(node)
    nodes.append(node)

# Create ANCF cable elements
for i in range(n_elements):
    element = fea.ChElementCableANCF()
    element.SetNodes(nodes[i], nodes[i + 1])
    element.SetSection(section)
    element.SetVisualize(True)
    element.SetColor(chrono.ChColor(0.9, 0.9, 0.9))
    mesh.AddElement(element)

# Create Irrlicht visualization
application = irr.ChIrrApp(system, "ANCF Cable Beam", irr.dimension2du(1024, 768))
application.AddTypicalSky()
application.AddTypicalLights(irr.vector3df(30, 100, 100), irr.vector3df(30, -100, 80), 200, 130)
application.AddTypicalCamera(irr.vector3df(1.5, 0.5, -2), irr.vector3df(1.5, 0.5, 0))

application.AssetBindAll()
application.AssetUpdateAll()

# Simulation loop settings
application.SetTimestep(0.001)
application.SetTryRealtime(True)

# Simulation loop
while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()