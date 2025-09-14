```python
import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

print("Example: PyChrono using beam finite elements")

sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()

# =============================================================================
# 1. Create Euler-Bernoulli beam section using ChBuilderBeamEuler
# =============================================================================

# Create a section for Euler-Bernoulli beams
msection = fea.ChBeamSectionEulerAdvanced()
beam_wy = 0.012
beam_wz = 0.025
msection.SetAsRectangularSection(beam_wy, beam_wz)
msection.SetYoungModulus(0.01e9)
msection.SetShearModulus(0.01e9 * 0.3)
msection.SetRayleighDamping(0.000)
msection.SetCentroid(0, 0.02)
msection.SetShearCenter(0, 0.1)
msection.SetSectionRotation(45 * chrono.CH_DEG_TO_RAD)  # Corrected rotation conversion

# =============================================================================
# 2. Create beam using ChBuilderBeamEuler helper
# =============================================================================

# Create builder and build beam with 5 elements
builder = fea.ChBuilderBeamEuler()
start_point = chrono.ChVector3d(0, 0, -0.1)
end_point = chrono.ChVector3d(0.2, 0, -0.1)
builder.BuildBeam(
    mesh,                # mesh to store elements
    msection,            # beam section properties
    5,                   # number of elements
    start_point,         # start point
    end_point,           # end point
    chrono.ChVector3d(0, 1, 0)  # 'Y' up direction
)

# Fix last node and apply force to first node
builder.GetLastBeamNodes()[-1].SetFixed(True)
builder.GetFirstBeamNodes()[0].SetForce(chrono.ChVector3d(0, -1, 0))

# =============================================================================
# 3. Modify original node constraints using proper FEA constraints
# =============================================================================

# Create fixed truss
mtruss = chrono.ChBody()
mtruss.SetFixed(True)
sys.Add(mtruss)

# Create proper constraints for original nodes using ChLinkNodeXYZ
constr_node1 = fea.ChLinkNodeXYZ()
constr_node1.Initialize(builder.GetLastBeamNodes()[0], mtruss, builder.GetLastBeamNodes()[0].GetPos())
sys.Add(constr_node1)

constr_node3 = fea.ChLinkNodeXYZ()
constr_node3.Initialize(builder.GetLastBeamNodes()[-1], mtruss, builder.GetLastBeamNodes()[-1].GetPos())
sys.Add(constr_node3)

# =============================================================================
# 4. Maintain visualization and solver settings
# =============================================================================

# Add mesh to system
sys.Add(mesh)

# Visualization settings
visualizebeamA = chrono.ChVisualShapeFEA(mesh)
visualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA