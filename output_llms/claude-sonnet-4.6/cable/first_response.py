import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

# -------------------------------------
# 1. Initialize the PyChrono system
# -------------------------------------
system = chrono.ChSystemSMC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# -------------------------------------
# 2. Create the FEA mesh
# -------------------------------------
mesh = fea.ChMesh()
system.Add(mesh)

# -------------------------------------
# 3. Define beam material and properties
# -------------------------------------
beam_length = 1.2        # Total length of the beam (m)
num_elements = 8         # Number of ANCF cable elements
density     = 1000.0     # kg/m^3
E_modulus   = 0.01e9     # Young's modulus (Pa) — flexible beam
beam_radius = 0.01       # Cross-section radius (m)
section_area = chrono.CH_C_PI * beam_radius**2
I_yy = chrono.CH_C_PI * beam_radius**4 / 4.0  # Second moment of area

# Create the beam section
section = fea.ChBeamSectionCable()
section.SetDiameter(beam_radius * 2)
section.SetYoungModulus(E_modulus)
section.SetDensity(density)
section.SetBeamRaleyghDamping(0.01)

# -------------------------------------
# 4. Build nodes and elements
# -------------------------------------
element_length = beam_length / num_elements
nodes = []

for i in range(num_elements + 1):
    # Position nodes along the X axis
    pos = chrono.ChVectorD(i * element_length, 0, 0)
    node = fea.ChNodeFEAxyzD(pos, chrono.ChVectorD(1, 0, 0))
    node.SetMass(0)
    mesh.AddNode(node)
    nodes.append(node)

# Create ANCF cable elements connecting the nodes
elements = []
for i in range(num_elements):
    element = fea.ChElementCableANCF()
    element.SetNodes(nodes[i], nodes[i + 1])
    element.SetSection(section)
    mesh.AddElement(element)
    elements.append(element)

# -------------------------------------
# 5. Apply boundary condition: hinge the first node to the ground
# -------------------------------------
# Fix position (but allow rotation) — truss/hinge constraint
hinge_body = chrono.ChBody()
hinge_body.SetPos(chrono.ChVectorD(0, 0, 0))
hinge_body.SetBodyFixed(True)
system.Add(hinge_body)

# Constrain the first node to the fixed body (pin/hinge)
constraint_hinge = fea.ChLinkPointFrame()
constraint_hinge.Initialize(nodes[0], hinge_body)
system.Add(constraint_hinge)

# Also fix the direction (gradient) of the first node to simulate a fixed end
# (comment the next two lines to allow rotation at the hinge)
# constraint_dir = fea.ChLinkDirFrame()
# constraint_dir.Initialize(nodes[0], hinge_body)
# system.Add(constraint_dir)

# -------------------------------------
# 6. Visualize the FEA mesh
# -------------------------------------
# Visual shape for the cable elements
vis_beam = fea.ChVisualizationFEAmesh(mesh)
vis_beam.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_ELEM_BEAM_MZ)
vis_beam.SetColorscaleMinMax(-0.4, 0.4)
vis_beam.SetSmoothFaces(True)
vis_beam.SetWireframe(False)
mesh.AddAsset(vis_beam)

# Visual shape for the nodes
vis_nodes = fea.ChVisualizationFEAmesh(mesh)
vis_nodes.SetFEMglyphType(fea.ChVisualizationFEAmesh.E_GLYPH_NODE_DOT_POS)
vis_nodes.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_NONE)
vis_nodes.SetSymbolsThickness(0.006)
mesh.AddAsset(vis_nodes)

# -------------------------------------
# 7. Set up the Irrlicht visualizer
# -------------------------------------
application = chronoirr.ChIrrApp(
    system,
    "ANCF Cable Beam - Hinged to Ground",
    chronoirr.dimension2du(1024, 768)
)

application.AddTypicalSky()
application.AddTypicalLights()
application.AddTypicalCamera(chronoirr.vector3df(0.5, 0.3, 1.5),
                              chronoirr.vector3df(0.5, 0.0, 0.0))
application.AddLightWithShadow(
    chronoirr.vector3df(1.5, 5.5, -2.5),
    chronoirr.vector3df(0.5, 0.0, 0.0),
    3, 2.2, 7.2, 40, 512
)

application.AssetBindAll()
application.AssetUpdateAll()

# -------------------------------------
# 8. Solver and integrator settings
# -------------------------------------
solver = chrono.ChSolverMINRES()
solver.SetMaxIterations(200)
solver.SetTolerance(1e-10)
solver.EnableWarmStart(True)
system.SetSolver(solver)

# Use HHT integrator for accuracy with FEA
integrator = chrono.ChTimestepperHHT(system)
integrator.SetAlpha(-0.2)
integrator.SetMaxiters(5)
integrator.SetAbsTolerances(1e-5)
integrator.SetMode(chrono.ChTimestepperHHT.POSITION)
system.SetTimestepper(integrator)

# -------------------------------------
# 9. Simulation loop
# -------------------------------------
timestep = 0.005
application.SetTimestep(timestep)
application.SetTryRealtime(False)

print("=" * 55)
print("  ANCF Cable Beam Simulation")
print("  Beam length  : {:.2f} m".format(beam_length))
print("  Num elements : {}".format(num_elements))
print("  Timestep     : {:.4f} s".format(timestep))
print("=" * 55)

step_count = 0

while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()

    # Print nodal positions every 50 steps
    if step_count % 50 == 0:
        print(f"\n--- Time = {system.GetChTime():.3f} s ---")
        for i, node in enumerate(nodes):
            pos = node.GetPos()
            print(f"  Node {i:2d}: x={pos.x:.4f}  y={pos.y:.4f}  z={pos.z:.4f}")

    application.DoStep()
    application.EndScene()
    step_count += 1

    # Stop after 5 seconds of simulation
    if system.GetChTime() > 5.0:
        break

print("\nSimulation complete.")