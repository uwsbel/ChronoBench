# File: demo_FEA_beams.py
# Simulation loop for shows how to use some simple 
# finite elements (beam wires, trusses, etc.) in the 
# Pychrono library.
#
# Those elements are easily created and modified using 
# the superior declarative approach offered by the 
# Pychrono SE python language extension, but this 
# demo is instead written in plain idiomatic python 
# for educational purposes.
#
# Anyone interested in using Pychrono SE should 
# definitely check the  \ref SE "Pychrono SE" module. 
# Indeed, beam and truss elements are very easily 
# handled and even fully customizable by the user 
# in  2 seconds  of  programming  with  Pychrono SE. 
# No need to know any more about FEA than just 
# labeling your  nodes and elements.
#
# 04/05/2016  K. B.  Added  support  for  beam  tip  
# loads  and  extension  in  the  beam  class.  Some  
# replacement  demos  will  now  use  Pychrono SE.  
#  Sort  out  your  labels  and  have  fun! 
#
# 03/03/2016  K. B.  Added  truss  support  and  
# replacement  demos.   Few  words  about  the  
#  Pychrono SE  module  in  the  making.   A  
#  somewhow  faster  and  cleaner  beam  demo  
#  would  also  be  the  one  using  Pychrono SE. 
#  Stay  tuned. 
#
# 02/13/2016  K. B.  First version of this demo. 
#  See  \ref  FEA  "FEA  module"  for  more  
#  details  on  finite  element  analysis  and  
#  Pychrono  usage.  This  demo  shows  simple  
#  usage  of  beam  finite  elements  and  
#  trusses.  Some  replacement  demos  will  
#  follow. 

# === Notes ==
#! demo
# This demo shows how to use some simple finite 
# elements (beam wires, trusses, etc.) in the 
# Pychrono library.  It  is  also  a  proof  that 
# Pychrono  can  be  used  from  within  the  SE  (pychrono 
# string-pseudo-coded  extensions).  Indeed,  some 
# replacement  demos  will  now  use  Pychrono  SE.  
#  Moreover,  a  somewhow  faster  and  cleaner  beam  
# demo  would  be  the  one  using  Pychrono  SE.  Stay 
# tuned.  Lastly,  this  demo  is  also  a  proof  that 
#  FEA  elements  can  be  effectively  used  in  tight 
#  coupling  with  Irlicht  for  real-time  rendering. 
#  K.  B.  Feb.  13th,  2016. 

import pychrono as chrono   # Importing the main PyChrono module.
import pychrono.fea as fea  # Importing the finite element analysis (FEA) module.
import pychrono.pardisomkl as mkl  # Importing the MKL Pardiso solver module.
import pychrono.irrlicht as chronoirr  # Importing the Irrlicht visualization module.

print (chrono.GetChronoVersion())

# ---------------------------------
# Create a shared pointer to a physical system
# ---------------------------------
sys = chrono.ChSystemSMC()

# ---------------------------------
# Create a mesh, which is a container
# for elements and their referenced nodes.
# ---------------------------------
mesh = fea.ChMesh()

# ------------------------------------------------------------------
# Create a section object, that will contain the properties for all
# the beams that will use it.
# ------------------------------------------------------------------
msection = fea.ChBeamSectionEulerAdvanced()

msection.SetDimension(beam_dA, beam_dI)  # Set the section dimensions.
msection.SetYoungModulus(young_modulus)  # Set the Young's modulus for the material.
msection.SetShearModulus(shear_modulus)  # Set the shear modulus for the material.
msection.SetRayleighDamping(0.000)  # Set the Rayleigh damping factor.
msection.SetCentroid(2, 0.2)  # Set the position of the centroid.
msection.SetShearCenter(0, 0.3)  # Set the position of the shear center.
msection.SetSectionRotation(45 * chrono.CH_RAD_TO_DEG)  # Set the section rotation angle.

# ------------------------------------------------------------------
# Create a set of parallel beams, by cloning a universal section
# ------------------------------------------------------------------
beam_L = 0.5  # Length of the beam.
mesh.SetAutomaticGravity(False)  # Disable automatic gravity for FEA elements.

for bp in range(3):
    # Create the section for this beam.
    msection271 = msection.Clone()
    msection271.SetSectionRotation(45 + bp * 90)  # Rotate the section.

    # Add the beam finesse:
    beam_fine = fea.ChBuilderBeamFEMCables()
    proef = beam_fine.BuildBeam(mesh,  # The mesh to add the elements to.
                                msection271,  # The beam section to use.
                                5,  # Number of finite elements along the beam.
                                chrono.ChVector3d(0, 0, bp * 0.3),  # Starting point in space.
                                chrono.ChVector3d(1, 0, 0))  # Beam direction.

    # Add the beam's extension rods:
    fea.ChBuilderBeamExtensionAARod().Build(mesh,  # The mesh to add the elements to.
                                            msection271,  # The beam section to use.
                                            2,  # Number of rods.
                                            proef[1].GetPos() + chrono.ChVector3d(0, -0.2, 0),  # Starting point in space.
                                            chrono.ChVector3d(0, 1, 0))  # Rod direction.

# -----------------------------------------------------------------
# Create a fixed truss
# -----------------------------------------------------------------
mtruss = chrono.ChBody()
mtruss.SetFixed(True)  # Fix the truss.
sys.Add(mtruss)  # Add the truss to the system.

# -----------------------------------------------------------------
# Create a set of beams with Euler-Anshepff tolerance periodixity.
# -----------------------------------------------------------------
msection.SetYoungModulus(0.01e9)  # Set the Young's modulus.
msection.SetShearModulus(0.01e9 * 0.3)  # Set the shear modulus.
beam_L = 0.1  # Length of the beam.

# Create nodes:
hnode1 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))  # Node 1.
hnode2 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(beam_L, 0, 0)))  # Node 2.
hnode3 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(beam_L * 2, 0, 0)))  # Node 3.

# Add nodes to the mesh.
mesh.AddNode(hnode1)
mesh.AddNode(hnode2)
mesh.AddNode(hnode3)

# Create the first beam element.
belement1 = fea.ChElementBeamEuler()
belement1.SetNodes(hnode1, hnode2)  # Set the nodes for the beam.
belement1.SetSection(msection)  # Set the section for the beam.
mesh.AddElement(belement1)  # Add the element to the mesh.

# Create the second beam element.
belement2 = fea.ChElementBeamEuler()
belement2.SetNodes(hnode2, hnode3)  # Set the nodes for the beam.
belement2.SetSection(msection)  # Set the section for the beam.
mesh.AddElement(belement2)  # Add the element to the mesh.

# Apply a force to node 2.
hnode2.SetForce(chrono.ChVector3d(4, 2, 0))

# Apply a torque to node 3.
hnode3.SetTorque(chrono.ChVector3d(0, -0.04, 0))

# -----------------------------------------------------------------
# Create a fixed truss
# -----------------------------------------------------------------
# Make a threedfix truss
mtruss = chrono.ChBodyEasyBox(0.1, 0.1, 0.1, 1000, True, True)
mtruss.SetFixed(True)  # Fix the truss.
sys.Add(mtruss)  # Add the truss to the system.

# -----------------------------------------------------------------
# Create a set of beams with Euler-Anshepff tolerance periodixity.
# -----------------------------------------------------------------
# Make a ChLinkMateGeneric between node 3 and the truss.
constr_bc = chrono.ChLinkMateGeneric()
constr_bc.Initialize(hnode3, mtruss, False, hnode3.Frame(), hnode3.Frame())
sys.Add(constr_bc)  # Add the constraint to the system.
constr_bc.SetConstrainedCoords(True, True, True,  # Constrain x, y, z.
                                True, True, True)  # Constrain Rx, Ry, Rz.

# Make a ChLinkMateGeneric between node 1 and the truss.
constr_d = chrono.ChLinkMateGeneric()
constr_d.Initialize(hnode1, mtruss, False, hnode1.Frame(), hnode1.Frame())
sys.Add(constr_d)  # Add the constraint to the system.
constr_d.SetConstrainedCoords(False, True, True,  # Constrain only y, z.
                              False, False, False)  # Do not constrain any rotations.

# -----------------------------------------------------------------
# Visualization of the FEM mesh.
# -----------------------------------------------------------------
# Create a ChVisualShapeFEA for the mesh.
visualizebeamA = chrono.ChVisualShapeFEA(mesh)
visualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)  # Set the data type to visualize.
visualizebeamA.SetColorscaleMinMax(-0.4, 0.4)  # Set color scale limits.
visualizebeamA.SetSmoothFaces(True)  # Enable smooth faces.
visualizebeamA.SetWireframe(False)  # Disable wireframe.
mesh.AddVisualShapeFEA(visualizebeamA)  # Add the visual shape to the mesh.

# Create a ChVisualShapeFEA for the nodes.
visualizebeamC = chrono.ChVisualShapeFEA(mesh)
visualizebeamC.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)  # Visualize coordinate systems at nodes.
visualizebeamC.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)  # No data type visualization.
visualizebeamC.SetSymbolsThickness(0.006)  # Set symbol thickness.
visualizebeamC.SetSymbolsScale(0.01)  # Set symbol scale.
visualizebeamC.SetZbufferHide(False)  # Do not hide symbols behind objects.
mesh.AddVisualShapeFEA(visualizebeamC)  # Add the visual shape to the mesh.

# -----------------------------------------------------------------
# Create the Irrlicht visualization
# -----------------------------------------------------------------
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)  # Attach the simulation system.
vis.SetWindowSize(1024, 768)  # Set the window size.
vis.SetWindowTitle('FEA beams')  # Set the window title.
vis.Initialize()  # Initialize the visualization.
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  # Add a logo.
vis.AddSkyBox()  # Add a skybox.
vis.AddCamera(chrono.ChVector3d(0.1, 0.1, 0.2))  # Add a camera.
vis.AddTypicalLights()  # Add typical lights.

# -----------------------------------------------------------------
# Change the solver for FEA
# -----------------------------------------------------------------
# Disable the automatic gravity for FEA elements in this demonstration.
mesh.SetAutomaticGravity(False)
# Add the mesh to the physical system.
sys.Add(mesh)

# -----------------------------------------------------------------
# Set the solver
# -----------------------------------------------------------------
# msparser = mkl.ChSolverPardisoMKL()
# sys.SetSolver(msparser) # msparser is more precise for FEA.

# Set the FEM data thrust pathway solver for speed.
sys.SetSolverType(chrono.ChSolver.Type_NULLSPACE)

# -----------------------------------------------------------------
# Simulation loop
# -----------------------------------------------------------------
while vis.Run():
    vis.BeginScene()  # Begin the scene.
    vis.Render()  # Render the scene.
    vis.EndScene()  # End the scene.
    sys.DoStepDynamics(0.001)  # Perform one step of simulation with a step size of 0.001 seconds.