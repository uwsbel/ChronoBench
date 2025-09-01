import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

print("Example: PyChrono using beam finite elements")

# Create the physical system that will be simulated.
sys = chrono.ChSystemSMC()

# Create a mesh, which is a container for elements and their referenced nodes.
mesh = fea.ChMesh()

# --- Add a Section on Euler-Bernoulli Beams ---
# Use the ChBuilderBeamEuler helper object for beam creation.
builder = fea.ChBuilderBeamEuler()

# Create a beam from point A to point B using BuildBeam.
point_A = chrono.ChVectorD(0, 0, -0.1)
point_B = chrono.ChVectorD(0.2, 0, -0.1)
builder.BuildBeam(mesh,                   # The mesh to add the beam to
                 point_A, point_B,       # Start and end points of the beam
                  5,                      # Number of elements
                 chrono.ChVectorD(0, 1, 0)) # 'Y' up direction

# Fix the last node of the created beam
builder.GetLastBeamNodes().back().SetFixed(True)

# Apply a force to the first node of the created beam section
builder.GetLastBeamNodes()[0].SetForce(chrono.ChVectorD(0, -1, 0))

# --- Modify Existing Node-Fixing Approach ---
# Replace the direct setting of a node as fixed with constraints to fix node 1
constr_a = chrono.ChLinkMateGeneric()
constr_a.Initialize(builder.GetLastBeamNodes()[0], sys.GetGroundBody(), False, builder.GetLastBeamNodes()[0].Frame(), builder.GetLastBeamNodes()[0].Frame())
sys.Add(constr_a)
constr_a.SetConstrainedCoords(True, True, True,  # Constrain x, y, z translations
                              True, True, True)  # Constrain Rx, Ry, Rz rotations

# --- Maintain Visualization and Solvers ---
# Add visualization for the beams in the mesh.
visualizebeamA = chrono.ChVisualShapeFEA(mesh)
visualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
visualizebeamA.SetColorscaleMinMax(-0.4, 0.4)
visualizebeamA.SetSmoothFaces(True)
visualizebeamA.SetWireframe(False)
mesh.AddVisualShapeFEA(visualizebeamA)

# Add visualization for the nodes in the mesh.
visualizebeamC = chrono.ChVisualShapeFEA(mesh)
visualizebeamC.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
visualizebeamC.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
visualizebeamC.SetSymbolsThickness(0.006)
visualizebeamC.SetSymbolsScale(0.01)
visualizebeamC.SetZbufferHide(False)
mesh.AddVisualShapeFEA(visualizebeamC)

# Create an Irrlicht visualization window.
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('FEA beams')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0.1, 0.1, 0.2))
vis.AddTypicalLights()

# Change the default solver to the MKL Pardiso solver.
msolver = mkl.ChSolverPardisoMKL()
sys.SetSolver(msolver)

# Simulation loop.
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.001)