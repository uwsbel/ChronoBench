import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as irr

# -----------------------------------------------------------------------------
# 1) SETUP THE PHYSICS SYSTEM
# -----------------------------------------------------------------------------
system = chrono.ChSystemSMC()
# Gravity in –Y
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# -----------------------------------------------------------------------------
# 2) CREATE THE FEA MESH AND MATERIAL FOR ANCF CABLE ELEMENTS
# -----------------------------------------------------------------------------
mesh = fea.ChMesh()

# ANCF cable section properties
young_modulus = 2e11     # [Pa]
density       = 7800     # [kg/m^3]
area          = 1e-4     # [m^2]
# Create a straight cable section with one Gauss point
section = fea.ChCableSectionAdvanced()
section.SetDensity(density)
section.SetYoungModulus(young_modulus)
section.SetDiameter(2.0 * chrono.sqrt(area / chrono.CH_C_PI))
section.SetAlphaDamp(0.0)
section.SetGamma(0.0)

# -----------------------------------------------------------------------------
# 3) BUILD NODES AND ELEMENTS FOR THE BEAM
# -----------------------------------------------------------------------------
L      = 1.0       # total beam length [m]
nSeg   = 20        # number of ANCF elements
dL     = L / nSeg  # element length

nodes = []
for i in range(nSeg + 1):
    # initial position along the X axis
    x = dL * i
    pos = chrono.ChVectorD(x, 0, 0)
    # initial slope vector D0: beam initially straight in X
    D0 = chrono.ChVectorD(1, 0, 0)
    node = fea.ChNodeFEAxyzD(pos, D0)
    # Fix the very first node to emulate a clamp/hinge at the left end:
    if i == 0:
        node.SetFixed(True)
    mesh.AddNode(node)
    nodes.append(node)

# create the cable elements connecting consecutive nodes
for i in range(nSeg):
    elem = fea.ChElementCableANCF()
    elem.SetNodes(nodes[i], nodes[i+1])
    elem.SetSection(section)
    # optional: specify internal damping, etc.
    mesh.AddElement(elem)

# -----------------------------------------------------------------------------
# 4) ADD VISUALIZATION ASSETS TO THE MESH
# -----------------------------------------------------------------------------
# Draw the deformed cable as a smooth line and show the nodes
vis_mesh = fea.ChVisualizationFEAmesh(mesh)
vis_mesh.SetWireframe(True)
vis_mesh.SetDrawNodes(True)
mesh.AddAsset(vis_mesh)

# -----------------------------------------------------------------------------
# 5) ADD THE MESH TO THE PHYSICS SYSTEM
# -----------------------------------------------------------------------------
system.Add(mesh)

# -----------------------------------------------------------------------------
# 6) CREATE AN IRRLICHT APPLICATION FOR REAL-TIME RENDERING
# -----------------------------------------------------------------------------
application = irr.ChIrrApp(
    system,
    "ANCF Cable Beam Example",
    irr.dimension2du(800, 600),
    irr.DO_NOT_SHARE_CONTEXT)

# Set up the camera, lights, sky, etc.
application.AddTypicalLogo(irr.vector2di(10, 10))
application.AddTypicalSky()
application.AddTypicalLights()
application.AddTypicalCamera(irr.vector3df(0.5, 0.5, 1.5))
application.AssetBindAll()
application.AssetUpdateAll()

# -----------------------------------------------------------------------------
# 7) RUN THE SIMULATION LOOP
# -----------------------------------------------------------------------------
application.SetTimestep(1e-3)

while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    # Advance simulation by one step
    application.DoStep()
    application.EndScene()