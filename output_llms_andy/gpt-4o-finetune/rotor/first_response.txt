import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

# Initialize the physical system
sys = chrono.ChSystemNSC()

# Add a visualization asset to the physical system
vis = chrono.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('FEA Test: IGA beam')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3, 10))
vis.AddTypicalLights()

# Set the camera to move with a specific node (commented out here)
# vis.SetSpinningCamera(chrono.ChVector3d(0, 0, 0), 15, 90, 5)

# Create the mesh container for FEM elements and add it to the physical system
mesh = fea.ChMesh()
sys.Add(mesh)

# Define beam parameters
beam_L = 3  # Length of the beam in meters
beam_rad = 0.04  # Radius of the beam in meters

# Create the IGA beam (a cylinder) using ChBuilderBeamIGA
builder = fea.ChBuilderBeamIGA()
section_cyl = fea.ChBeamSectionEulerAdvanced()
section_cyl.SetDensity(1600)  # Set the density of the beam section in kg/m^3
section_cyl.SetYoungModulus(3.5e9)  # Set the Young's modulus in Pascals (N/m^2)
section_cyl.SetShearModulus(3.5e9 / (2.0 * (1.0 + 0.3)))  # Set the shear modulus
section_cyl.SetRadii(beam_rad, beam_rad + 0.01)  # Set the inner and outer radii of the section
section_cyl.SetRayleighDamping(0.000)  # Set Rayleigh damping (here it's zero)

# Build a straight IGA beam of specified length
builder.BuildBeam(mesh, section_cyl,  # The mesh container and section
                  2,  # Number of segments (chords) - here it's 2, creating 3 nodes
                  chrono.ChVector3d(0, 0, 0),  # Start point of the beam
                  chrono.ChVector3d(beam_L, 0, 0),  # End point of the beam
                  chrono.VECT_Y,  # Twist vector (Y-axis)
                  1)  # Order of the beam (cubic spline)

# Set the boundary conditions for the beam
node_div_6 = builder.GetLastBeamNodes()[1]  # Get the node at 1/6th length from the left end
node_div_6.SetFixed(True)  # Fix this node (no movement)

# Create and initialize a flywheel and attach it to the beam
flywheel = chrono.ChBody()
flywheel.SetMass(0.96)  # Set the mass of the flywheel in kg
flywheel.SetInertiaXX(chrono.ChVector3d(0.032, 0.96, 0.96))  # Set the moment of inertia
flywheel.SetFixed(False)  # Make sure the flywheel is not fixed
flywheel.SetFrictIon(0)  # Set friction to zero
flywheel.SetRestitution(0)  # Set restitution to zero
flywheel.SetContactMethod(chrono.ChContactMethod_NSC)  # Set contact method
# Add a visualization asset for the flywheel
flywheel.AddAsset(chrono.ChVisualShapeCylinder(.05, .2))

# Add the flywheel to the physical system
sys.Add(flywheel)

# Create a spherical joint to connect the flywheel to the beam
spjoint = chrono.ChLinkLockSpherical()
# Initialize the joint at the specified position (center of the beam)
spjoint.Initialize(flywheel,
                  node_div_6,
                  chrono.ChVector3d(beam_L / 2, 0, 0))
# Add the joint to the system
sys.AddLink(spjoint)

# Create the second IGA beam
builder2 = fea.ChBuilderBeamIGA()
section_box = fea.ChBeamSectionEulerAdvanced()
section_box.SetDensity(1600)  # Set density
section_box.SetYoungModulus(3.5e9)  # Set Young's modulus
section_box.SetRayleighDamping(0.000)  # Set Rayleigh damping
section_box.SetAsRectangularSection(0.05, 0.15)  # Define rectangular section

# Define a coordinate system for orientation
rot90 = chrono.ChQuaterniond()
rot90.SetFromAngleAxis(chrono.CHPI_2, chrono.VECT_X)
mcs = chrono.ChMatrix33d(rot90)

# Build a tapered beam with the rectangular section at ends
builder2.BuildBeam(mesh, section_box,
                   2,  # Number of segments
                   chrono.ChVector3d(beam_L / 2, 0.1, 0),  # Start point
                   chrono.ChVector3d(beam_L / 2, 0.25, 0),  # End point
                   mcs,  # Orientation
                   1)  # Order of beam

# Create a rotational motor and attach it to the left end of the beam
motor = chrono.ChLinkMotorRotationSpeed()
# Create a flywheel for the motor
flywheel_motor = chrono.ChBody()
flywheel_motor.SetFixed(False)
flywheel_motor.AddVisualShape(chrono.ChVisualShapeSphere(0.1))
motor.SetMotorFlywheel(flywheel_motor)  # Set the flywheel for the motor
sys.Add(flywheel_motor)  # Add flywheel to system
my_gear = chrono.ChLinkLockScrew()  # Create a screw link
my_gear.Initialize(flywheel_motor,
                  sys.GetBodyFromFrame(motor.Frame2),
                  motor.Frame2)
sys.Add(my_gear)  # Add the link to the system

# Attach the motor to the beam node
motor.Initialize(builder.GetLastBeamNodes()[-1],  # Node at the right end of the beam
                chrono.ChNullBody(),  # No second body, ground referenced
                chrono.ChFramed(chrono.ChVector3d(beam_L, 0, 0)))  # Motor reference frame
sys.AddLink(motor)  # Add the motor to the system
# Set the rotational speed function for the motor (constant speed here)
rot_funct = chrono.ChFunctionConst(35)
motor.SetMotorFunction(rot_funct)

# Attach a visual shape container to the mesh for better visualization
visualizebeamA = chrono.ChVisualShapeContainer()
mesh.AddVisualShapeContainer(visualizebeamA)

# Add FEM data visualization for strain (commented options are available)
g_strain = fea.ChVisualFEMdata(mesh)
g_strain.SetFEMdataType(chrono.ChVisualFEMdata.E_PLOT_UNITSTRAIN) # Plot unit strain
g_strain.SetThickness(0.010)  # Set thickness for visualization
g_strain.SetSmoothFaces(True)  # Enable smooth faces
g_strain.SetSmoothFalloff(True)  # Enable smooth falloff
visualizebeamA.AddVisualShape(g_strain)  # Add strain visualization to container

g_stress = fea.ChVisualFEMdata(mesh)
g_stress.SetFEMglyphType(chrono.ChVisualFEMdata.E_GLYPH_NODE_STRESS_RADII) # Set glyph type for node stress
g_stress.SetFEMdataType(chrono.ChVisualFEMdata.E_PLOT_NONE) # No direct plot for stress
g_stress.SetSymbolsThickness(0.01)  # Set symbol thickness
g_stress.SetStressTensorScale(0.05)  # Set stress tensor scale
g_stress.SetSmoothFaces(True)  # Enable smooth faces
visualizebeamA.AddVisualShape(g_stress)  # Add stress visualization to container

# Final visualization settings
mesh.SetAutomaticGravity(False)  # Disable automatic gravity on the mesh
mesh.SetVisualFEMstyle(chrono.ChVisualFEMdata.Style_ELEM_WIREframe)  # Set visual style for FEM
# Add a coordinate system for visualization
coordsys = chrono.ChVisualShapeFrame(0.3)
sys.AddVisualShape(coordsys, chrono.ChFramed(chrono.VECT3_ZERO))

# Set solver and timestepper for the simulation
solver = chrono.ChSolverMINRES()
solver.SetDiagonalPreconditioning(True)
solver.SetMaxIterations(100)
solver.SetTolerance(1e-10)
solver.SetVerbose(False)
sys.SetSolver(solver)

# Set the timestepper type (using HHT here)
ts = chrono.ChTimestepperHHT()
ts.SetAlpha(-0.2)
sys.SetTimestepper(ts)

# Define time step for simulation
timestep = 0.001

# Run the simulation loop
while vis.Run():
    sys.DoStepDynamics(timestep)  # Advance the simulation by one time step
    vis.BeginScene()  # Begin scene for visualization
    vis.Render()  # Render the scene
    vis.EndScene()  # End scene for visualization