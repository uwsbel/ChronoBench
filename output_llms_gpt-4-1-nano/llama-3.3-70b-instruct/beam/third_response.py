import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr
import sys

print("Example: PyChrono using beam finite elements")

# Create a system
system = chrono.ChSystemSMC()

# Create a mesh
mesh = fea.ChMesh()

# Create a section
msection = fea.ChBeamSectionEulerAdvanced()

beam_wy = 0.012
beam_wz = 0.025
msection.SetAsRectangularSection(beam_wy, beam_wz)
msection.SetYoungModulus(0.01e9)
msection.SetShearModulus(0.01e9 * 0.3)
msection.SetRayleighDamping(0.000)
msection.SetCentroid(0, 0.02)
msection.SetShearCenter(0, 0.1)
msection.SetSectionRotation(45 * chrono.CH_RAD_TO_DEG)

# Add beam elements
beam_L = 0.1

hnode1 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
hnode2 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(beam_L, 0, 0)))
hnode3 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(beam_L * 2, 0, 0)))

mesh.AddNode(hnode1)
mesh.AddNode(hnode2)
mesh.AddNode(hnode3)

belement1 = fea.ChElementBeamEuler()
belement1.SetNodes(hnode1, hnode2)
belement1.SetSection(msection)
mesh.AddElement(belement1)

belement2 = fea.ChElementBeamEuler()
belement2.SetNodes(hnode2, hnode3)
belement2.SetSection(msection)
mesh.AddElement(belement2)

# Apply forces and constraints
hnode2.SetForce(chrono.ChVector3d(4, 2, 0))
hnode3.SetTorque(chrono.ChVector3d(0, -0.04, 0))

mtruss = chrono.ChBody()
mtruss.SetFixed(True)
system.Add(mtruss)

constr_bc = chrono.ChLinkMateGeneric()
constr_bc.Initialize(hnode3, mtruss, False, hnode3.Frame(), hnode3.Frame())
system.Add(constr_bc)
constr_bc.SetConstrainedCoords(True, True, True,  # x, y, z
                               True, True, True)  # Rx, Ry, Rz

constr_d = chrono.ChLinkMateGeneric()
constr_d.Initialize(hnode1, mtruss, False, hnode1.Frame(), hnode1.Frame())
system.Add(constr_d)
constr_d.SetConstrainedCoords(False, True, True,  # x, y, z
                              False, False, False)  # Rx, Ry, Rz

# Create a builder for beam elements
builder = fea.ChBuilderBeamEuler()

# Build a beam segment
builder.BuildBeam(mesh,  # the mesh where to put the created nodes and elements
                  msection,  # the ChBeamSectionEulerAdvanced to use for the ChElementBeamEuler elements
                  5,  # the number of ChElementBeamEuler to create
                  chrono.ChVector3d(0, 0, -0.1),  # the 'A' point in space (beginning of beam)
                  chrono.ChVector3d(0.2, 0, -0.1),  # the 'B' point in space (end of beam)
                  chrono.ChVector3d(0, 1, 0))  # the 'Y' up direction of the section for the beam

# Build another beam segment
builder.BuildBeam(mesh,  # the mesh where to put the created nodes and elements
                  msection,  # the ChBeamSectionEulerAdvanced to use for the ChElementBeamEuler elements
                  5,  # the number of ChElementBeamEuler to create
                  builder.GetLastBeamNodes().back().GetFrame().GetPos(),  # the 'A' point in space (beginning of beam)
                  chrono.ChVector3d(0.2, 0.1, -0.1),  # the 'B' point in space (end of beam)
                  chrono.ChVector3d(0, 1, 0))  # the 'Y' up direction of the section for the beam

# Retrieve nodes and apply forces
builder.GetLastBeamNodes().back().SetFixed(True)
builder.GetLastBeamNodes().front().SetForce(chrono.ChVector3d(0, -1, 0))

# Add the mesh to the system
system.Add(mesh)

# Set automatic gravity to False
mesh.SetAutomaticGravity(False)

# Visualize the FEM mesh
visualizebeamA = chrono.ChVisualShapeFEA(mesh)
visualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
visualizebeamA.SetColorscaleMinMax(-0.4, 0.4)
visualizebeamA.SetSmoothFaces(True)
visualizebeamA.SetWireframe(False)
mesh.AddVisualShapeFEA(visualizebeamA)

visualizebeamC = chrono.ChVisualShapeFEA(mesh)
visualizebeamC.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
visualizebeamC.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
visualizebeamC.SetSymbolsThickness(0.006)
visualizebeamC.SetSymbolsScale(0.01)
visualizebeamC.SetZbufferHide(False)
mesh.AddVisualShapeFEA(visualizebeamC)

# Create the Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('FEA beams')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.1, 0.1, 0.2))
vis.AddTypicalLights()

# Change the solver to the MKL Pardiso solver
msolver = mkl.ChSolverPardisoMKL()
system.SetSolver(msolver)

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(0.001)