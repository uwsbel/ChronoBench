import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

print("Example: PyChrono using beam finite elements")

sys = chrono.ChSystemSMC()

## Create a mesh, that is a container for groups
## of elements and their referenced nodes.
mesh = fea.ChMesh()

## Create a section, i.e. thickness and material properties
## for beams. This will be shared among some beams.

msection = fea.ChBeamSectionEulerAdvanced()

beam_wy = 0.012
beam_wz = 0.025
msection.SetAsRectangularSection(beam_wy, beam_wz)
msection.SetYoungModulus(0.01e9)
msection.SetShearModulus(0.01e9 * 0.3)
msection.SetRayleighDamping(0.000)
msection.SetCentroid(0, 0.02)
msection.SetShearCenter(0, 0.1)
msection.SetSectionRotation(45 * chrono.CH_C_DEG_TO_RAD)  # Use CH_C_DEG_TO_RAD instead of CH_RAD_TO_DEG

# Create nodes with corrected ChFrameD and ChVectorD
hnode1 = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
hnode2 = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD(0.1, 0, 0)))
hnode3 = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD(0.2, 0, 0)))

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
mesh.AddElement(belement2)  # Removed semicolon

# Apply forces/torques and fix nodes as before...

# Add Euler-Bernoulli beams using the builder
builder = fea.ChBuilderBeamEuler()

# Existing beam
builder.BuildBeam(
    mesh,
    msection,
    5,
    chrono.ChVectorD(0, 0, -0.1),
    chrono.ChVectorD(0.2, 0, -0.1),
    chrono.ChVectorD(0, 1, 0)
)

# New beam as per instruction
builder.BuildBeam(
    mesh,
    msection,
    5,
    chrono.ChVectorD(0.2, 0, -0.1),  # Starting at previous beam's endpoint
    chrono.ChVectorD(0.2, 0.1, -0.1),
    chrono.ChVectorD(0, 1, 0)
)

# Retrieve and fix nodes if needed (optional)
# builder.GetLastBeamNodes().back().SetFixed(True)
# ...

# Rest of the code remains as before with corrections:
mesh.SetAutomaticGravity(False)
sys.Add(mesh)

# Visualization and simulation setup...
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

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('FEA beams')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0.1, 0.1, 0.2))
vis.AddTypicalLights()

sys.SetSolver(mkl.ChSolverPardisoMKL())

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.001)

import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

print("Example: PyChrono using beam finite elements")

sys = chrono.ChSystemSMC()

## Create a mesh, that is a container for groups
## of elements and their referenced nodes.
mesh = fea.ChMesh()

## Create a section, i.e. thickness and material properties
## for beams. This will be shared among some beams.

msection = fea.ChBeamSectionEulerAdvanced()

beam_wy = 0.012
beam_wz = 0.025
msection.SetAsRectangularSection(beam_wy, beam_wz)
msection.SetYoungModulus(0.01e9)
msection.SetShearModulus(0.01e9 * 0.3)
msection.SetRayleighDamping(0.000)
msection.SetCentroid(0, 0.02)
msection.SetShearCenter(0, 0.1)
msection.SetSectionRotation(45 * chrono.CH_C_DEG_TO_RAD)  # Fixed rotation conversion

# Create nodes with corrected ChFrameD and ChVectorD
hnode1 = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
hnode2 = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD(0.1, 0, 0)))
hnode3 = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD(0.2, 0, 0)))

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
mesh.AddElement(belement2)  # Removed semicolon

# Apply forces/torques and fix nodes as before...

# Add Euler-Bernoulli beams using the builder
builder = fea.ChBuilderBeamEuler()

# Existing beam
builder.BuildBeam(
    mesh,
    msection,
    5,
    chrono.ChVectorD(0, 0, -0.1),
    chrono.ChVectorD(0.2, 0, -0.1),
    chrono.ChVectorD(0, 1, 0)
)

# New beam as per instruction
builder.BuildBeam(
    mesh,
    msection,
    5,
    chrono.ChVectorD(0.2, 0, -0.1),  # Starting at previous beam's endpoint
    chrono.ChVectorD(0.2, 0.1, -0.1),
    chrono.ChVectorD(0, 1, 0)
)

# Rest of the code remains as before with corrections:
mesh.SetAutomaticGravity(False)
sys.Add(mesh)

# Visualization and simulation setup...
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

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('FEA beams')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0.1, 0.1, 0.2))
vis.AddTypicalLights()

sys.SetSolver(mkl.ChSolverPardisoMKL())

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.001)

import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

print("Example: PyChrono using beam finite elements")

sys = chrono.ChSystemSMC()

## Create a mesh, that is a container for groups
## of elements and their referenced nodes.
mesh = fea.ChMesh()

## Create a section, i.e. thickness and material properties
## for beams. This will be shared among some beams.

msection = fea.ChBeamSectionEulerAdvanced()

beam_wy = 0.012
beam_wz = 0.025
msection.SetAsRectangularSection(beam_wy, beam_wz)
msection.SetYoungModulus(0.01e9)
msection.SetShearModulus(0.01e9 * 0.3)
msection.SetRayleighDamping(0.000)
msection.SetCentroid(0, 0.02)
msection.SetShearCenter(0, 0.1)
msection.SetSectionRotation(45 * chrono.CH_C_DEG_TO_RAD)  # Fixed rotation conversion

# Create nodes with corrected ChFrameD and ChVectorD
hnode1 = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
hnode2 = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD(0.1, 0, 0)))
hnode3 = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD(0.2, 0, 0)))

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
mesh.AddElement(belement2)  # Removed semicolon

# Apply forces/torques and fix nodes as before...
hnode2.SetForce(chrono.ChVectorD(4, 2, 0))
hnode3.SetTorque(chrono.ChVectorD(0, -0.04, 0))

mtruss = chrono.ChBody()
mtruss.SetFixed(True)
sys.Add(mtruss)

constr_bc = chrono.ChLinkMateGeneric()
constr_bc.Initialize(hnode3, mtruss, False, hnode3.GetFrame(), hnode3.GetFrame())
sys.Add(constr_bc)
constr_bc.SetConstrainedCoords(True, True, True, True, True, True)

constr_d = chrono.ChLinkMateGeneric()
constr_d.Initialize(hnode1, mtruss, False, hnode1.GetFrame(), hnode1.GetFrame())
sys.Add(constr_d)
constr_d.SetConstrainedCoords(False, True, True, False, False, False)

# Add Euler-Bernoulli beams using the builder
builder = fea.ChBuilderBeamEuler()

# Existing beam
builder.BuildBeam(
    mesh,
    msection,
    5,
    chrono.ChVectorD(0, 0, -0.1),
    chrono.ChVectorD(0.2, 0, -0.1),
    chrono.ChVectorD(0, 1, 0)
)

# New beam as per instruction
builder.BuildBeam(
    mesh,
    msection,
    5,
    chrono.ChVectorD(0.2, 0, -0.1),  # Starting at previous beam's endpoint
    chrono.ChVectorD(0.2, 0.1, -0.1),
    chrono.ChVectorD(0, 1, 0)
)

mesh.SetAutomaticGravity(False)
sys.Add(mesh)

# Visualization and simulation setup
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

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('FEA beams')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0.1, 0.1, 0.2))
vis.AddTypicalLights()

sys.SetSolver(mkl.ChSolverPardisoMKL())

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.001)