import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

# ----------------------------------------------------------------------------
# Sample
# This demo shows the use of  beam ANCF 3 element class.
# A single beam is modeled and its dinamics are simulated.
# This system is solved with the NSC (non smooth contact) formulation.
# ----------------------------------------------------------------------------

# Uncomment this line if you want to measure the performance of this script
# chrono.SetCommandLineArguments('-ls')

# Create the main Chrono component system
sys = chrono.ChSystemNSC()

# Create a mesh containing finite elements
mesh = fea.ChMesh()

# Create a section object, which will hold properties for all beams that
# will use this section.
msection = fea.ChBeamSectionEulerAdvanced()

# Set the width and height of the beam section.
msection.SetAsRectangularSection(0.12, 0.24)

# Set the material properties of the beam section.
msection.SetYoungModulus(0.01e9)
msection.SetShearModulus(0.01e9 * 0.3)  # Shear modulus is typically less than Young's modulus
msection.SetRayleighDamping(0.000)      # Damping coefficient for dynamic analysis

# Set the section properties,, which affect the bending behavior of the beam.
msection.SetQuadraticCurve(False)  # Use linear curvature instead of quadratic
msection.SetQ1(0.04)                # Bend stiffness parameter
msection.SetMass(0.0)                # Set mass of the section (usually zero for beam elements)

# Create a fixed point part which will act as the root of the beam
# Set displacement and orientation of the fixed point part
ch_p1 = chrono.ChVector3d(-0.2, 0, 0)
ch_fixed_point = fea.ChNodeFEAxyz(ch_p1)
ch_fixed_point.SetMarkup(False)  # Disable markup (visualization) for this node
mesh.AddNode(ch_fixed_point)      # Add the fixed point node to the mesh

# Create a beam element and set its properties
beam1 = fea.ChElementBeamANCF()
beam1.SetNodes(ch_fixed_point,  # One end of the beam
               fea.ChNodeFEAxyzrot(chrono.ChVector3d(0.8, 0, 0)))  # Other end of the beam
beam1.SetSection(msection)  # Assign the section properties to the beam element
mesh.AddElement(beam1)      # Add the beam element to the mesh

# Create a visual representation of the beam element
visualbeamA = chrono.ChVisualShapeFEA(beam1)
visualbeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)  # Visualize the bending moments
visualbeamA.SetColorscaleMinMax(-0.4, 0.4)  # Set color scale for moments
visualbeamA.SetSmoothFaces(True)            # Enable smooth faces for visualization
visualbeamA.SetWireframe(False)              # Disable wireframe mode
beam1.AddVisualShapeFEA(visualbeamA)        # Add the visual shape to the beam element

# ----------------------------------------------------------------------------
# Create the Irrlicht visualization system
# Note: this will be automatically attached to the simulation system
# ----------------------------------------------------------------------------

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)  # Attach the simulation system to the visualization
vis.SetWindowSize(1024, 768)  # Set the window size for the visualization
vis.SetWindowTitle('FEA beams')  # Set the window title
vis.Initialize()  # Initialize the visualization system
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  # Add a logo to the window
vis.AddSkyBox()  # Add a skybox for better visualization
vis.AddCamera(chrono.ChVector3d(0, 0.6, 0.8))  # Add a camera to the scene
vis.AddTypicalLights()  # Add typical lights for better illumination

# ----------------------------------------------------------------------------
# Initialize the system
# ----------------------------------------------------------------------------

sys.Add(mesh)  # Add the mesh to the simulation system

# ----------------------------------------------------------------------------
# Simulation loop
# ----------------------------------------------------------------------------

while vis.Run():
    vis.BeginScene()  # Begin the scene for rendering
    vis.Render()  # Render the scene
    vis.EndScene()  # End the scene
    sys.DoStepDynamics(0.01)  # Perform one step of simulation with a step size of 0.01 seconds