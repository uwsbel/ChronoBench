import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

print("Example: PyChrono using beam finite elements")

# -------------------------------------------------------------------------------
# 1. Create the Chrono physical system
# -------------------------------------------------------------------------------
sys = chrono.ChSystemSMC()

# -------------------------------------------------------------------------------
# 2. Create a mesh that will contain the finite elements
# -------------------------------------------------------------------------------
mesh = fea.ChMesh()

# -------------------------------------------------------------------------------
# 3. Define a common beam section
# -------------------------------------------------------------------------------
msection = fea.ChBeamSectionEulerAdvanced()

beam_wy = 0.012
beam_wz = 0.025
msection.SetAsRectangularSection(beam_wy, beam_wz)
msection.SetYoungModulus(0.01e9)
msection.SetShearModulus(0.01e9 * 0.3)
msection.SetRayleighDamping(0.000)
msection.SetCentroid(0, 0.02)
msection.SetShearCenter(0, 0.10)
# SetSectionRotation wants radians, so convert degrees -> radians
msection.SetSectionRotation(45 * chrono.CH_C_DEG_TO_RAD)

# -------------------------------------------------------------------------------
# 4. Manually add three nodes and two Euler-Bernoulli beam elements
# -------------------------------------------------------------------------------
beam_L = 0.1

hnode1 = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD(0.0,        0.0, 0.0)))
hnode2 = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD(beam_L,      0.0, 0.0)))
hnode3 = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD(beam_L * 2., 0.0, 0.0)))

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

# -------------------------------------------------------------------------------
# 5. Apply loads to the nodes
# -------------------------------------------------------------------------------
hnode2.SetForce(chrono.ChVectorD(4.0, 2.0, 0.0))
hnode3.SetTorque(chrono.ChVectorD(0.0, -0.04, 0.0))

# -------------------------------------------------------------------------------
# 6. Create the ground (truss) and constraints
# -------------------------------------------------------------------------------
mtruss = chrono.ChBody()
mtruss.SetFixed(True)
sys.Add(mtruss)

# Full six-dof lock of node 3 to ground
constr_bc = chrono.ChLinkMateGeneric()
constr_bc.Initialize(hnode3, mtruss, False, hnode3.Frame(), hnode3.Frame())
constr_bc.SetConstrainedCoords(True, True, True,  True, True, True)
sys.Add(constr_bc)

# Partially constrain node 1 (only Y and Z translations)
constr_d = chrono.ChLinkMateGeneric()
constr_d.Initialize(hnode1, mtruss, False, hnode1.Frame(), hnode1.Frame())
constr_d.SetConstrainedCoords(False, True, True,  False, False, False)
sys.Add(constr_d)

# -------------------------------------------------------------------------------
# 7. Add further beams with the ChBuilderBeamEuler helper
# -------------------------------------------------------------------------------
builder = fea.ChBuilderBeamEuler()

# First automatically-built beam (already present in original script)
builder.BuildBeam(mesh,                       # mesh to populate
                  msection,                   # section to use
                  5,                          # number of FEM elements
                  chrono.ChVectorD(0.0, 0.0, -0.1),      # start point  A
                  chrono.ChVectorD(0.2, 0.0, -0.1),      # end   point  B
                  chrono.ChVectorD(0.0, 1.0, 0.0))       # Y up direction

# Fix the last node of this first automatic beam and load its first node
builder.GetLastBeamNodes().back().SetFixed(True)
builder.GetLastBeamNodes().front().SetForce(chrono.ChVectorD(0.0, -1.0, 0.0))

# -------------------------------------------------------------------------------
# *****  REQUIREMENT FROM THE ASSIGNMENT  ***************************************
# 8. Add a second beam segment that starts from the last node generated above
#    and ends at (0.2, 0.1, -0.1), ‘Y’ up direction = (0,1,0)
# -------------------------------------------------------------------------------
# Retrieve the last node created by the previous BuildBeam()
start_node = builder.GetLastBeamNodes().back()        # this is the ‘A’ node
end_point  = chrono.ChVectorD(0.2, 0.1, -0.1)         # ‘B’ point

builder.BuildBeam(mesh,               # mesh to populate
                  msection,           # section
                  5,                  # number of elements
                  start_node,         # start from existing node
                  end_point,          # end point
                  chrono.ChVectorD(0.0, 1.0, 0.0))     # Y up direction

# -------------------------------------------------------------------------------
# 9. Finalise the mesh and add it to the system
# -------------------------------------------------------------------------------
# In this demo we do not want gravitational forces on FEA elements
mesh.SetAutomaticGravity(False)

sys.Add(mesh)

# -------------------------------------------------------------------------------
# 10. Visualisation assets for the mesh
# -------------------------------------------------------------------------------
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

# -------------------------------------------------------------------------------
# 11. Irrlicht run-time visualisation
# -------------------------------------------------------------------------------
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('FEA beams')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0.1, 0.1, 0.2))
vis.AddTypicalLights()

# -------------------------------------------------------------------------------
# 12. Use the MKL Pardiso solver (better for FEA problems)
# -------------------------------------------------------------------------------
msolver = mkl.ChSolverPardisoMKL()
sys.SetSolver(msolver)

# -------------------------------------------------------------------------------
# 13. Simulation loop
# -------------------------------------------------------------------------------
timestep = 0.001
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(timestep)