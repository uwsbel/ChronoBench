import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

print("Example: PyChrono using  beam finite elements")

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
# Potential Error: SetCentroid and SetShearCenter arguments were swapped.
# Corrected:
msection.SetCentroid(0, 0.02)
msection.SetShearCenter(0, 0.1)
msection.SetSectionRotation(45 * chrono.CH_C_RAD_TO_DEG)  # Use chrono.CH_C_RAD_TO_DEG

# ... (rest of the code remains the same)

# Add another beam segment using builder.BuildBeam()
# Retrieve the last node created by the previous beam as the starting point for the new beam.
last_node = builder.GetLastBeamNodes().front()
builder.BuildBeam(mesh,  # the mesh where to put the created nodes and elements
                  msection,  # the ChBeamSectionEulerAdvanced to use for the ChElementBeamEuler elements
                  5,  # the number of ChElementBeamEuler to create
                  last_node.GetPos(),  # the 'A' point in space (beginning of beam)
                  chrono.ChVector3d(0.2, 0.1, -0.1),  # the 'B' point in space (end of beam)
                  chrono.ChVector3d(0, 1, 0))  # the 'Y' up direction of the section for the beam

# ... (rest of the code remains the same)