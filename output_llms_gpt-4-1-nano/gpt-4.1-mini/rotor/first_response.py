import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

# 1. Initialize PyChrono system and components
chrono.SetChronoDataPath(chrono.GetChronoDataPath())  # Ensure data path set for assets
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Set gravity

# 2. Create the beam as an IGA beam (IGA requires specific setup using fea.ChMesh, fea.ChElementBeamIGA)
# Here, we create a simple straight beam using IGA elements

# Beam parameters
beam_length = 1.0         # length in meters
beam_divisions = 10       # number of elements along the beam length
beam_width = 0.02         # width of cross-section
beam_height = 0.02        # height of cross-section
E = 2.1e11                # Young's modulus for steel (Pa)
rho = 7800                # density (kg/m^3)
nu = 0.3                  # Poisson ratio

# Create the IGA mesh
mesh = fea.ChMesh()

# Create section properties for rectangular cross-section
section = fea.ChBeamSectionEulerAdvanced()
section.SetYoungModulus(E)
section.SetGfactor(E / (2*(1+nu)))
section.SetDensity(rho)
section.SetArea(beam_width * beam_height)
section.SetInertiaXX(  # Area moments of inertia for rectangular cross section (about centroidal axes)
    (beam_height*beam_width**3)/12, 
    (beam_width*beam_height**3)/12, 
    0)
section.SetWarpingConstant(beam_width * beam_height**3 / 12.0)  # approximate warping constant

# Create nodes for IGA beam (using quad spline for isogeometric beam)
# Nodes positioned along x axis from 0 to beam_length

# We will create 1D spline basis with beam_divisions+1 control points
# For simplicity, we place nodes uniformly along the beam axis.

# Initialize list of nodes
nodes = []
for i in range(beam_divisions + 1):
    x = i * beam_length / beam_divisions
    node = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD(x, 0, 0)))
    node.SetNeumann(chrono.ChVectorD(0, 0, 0))  # zero initial force/moment
    mesh.AddNode(node)
    nodes.append(node)

# Define spline basis for the beam: cubic spline with appropriate knot vector
# PyChrono provides ChContinuumBSpline basis used for IGA beams

# Knot vector for open uniform knots with beam_divisions elements and degree 3 (cubic)
degree = 3
num_ctrl_pts = beam_divisions + 1
num_knots = num_ctrl_pts + degree + 1

# Open uniform knots with multiplicity degree+1 at start and end
knots = []
for i in range(degree + 1):
    knots.append(0.0)
for i in range(1, num_knots - 2 * (degree + 1) + 1):
    knots.append(i / (num_knots - 2 * (degree + 1) + 1))
for i in range(degree + 1):
    knots.append(1.0)
knots_arr = knots

# Create spline basis object
basis = fea.ChContinuumBSpline(degree, knots_arr)

# Create the beam section geometry for IGA beam
# Using rectangular section created already (section)

# Now create IGA beam element for each span between nodes:
# The PyChrono IGA beam elements need the mesh, the spline basis,
# the section, and the span index.

# Add the spline basis to the mesh for the only dimension (1D spline)
mesh.SetSplines([basis])

# Create the beam element with given section, spanning each element interval
for i in range(beam_divisions):
    elem = fea.ChElementBeamIGA()
    elem.SetNodes(nodes[i], nodes[i + 1])
    elem.SetSection(section)
    elem.SetSpan(i)
    mesh.AddElement(elem)

# Add the mesh to the system
system.Add(mesh)

# Fix the beam start node (at x=0) to be a fixed boundary (cantilever style)
nodes[0].SetFixed(True)

# 3. Create the flywheel at the beam center

# Flywheel parameters
flywheel_radius = 0.1
flywheel_width = 0.02
flywheel_pos = chrono.ChVectorD(beam_length/2, 0, 0)

flywheel = chrono.ChBody()
flywheel.SetPos(flywheel_pos)
flywheel.SetMass(10)
flywheel.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.05))

# Add a visual representation of the flywheel as a cylinder
flywheel_shape = chrono.ChCylinderShape()
flywheel_shape.GetCylinderGeometry().rad = flywheel_radius
flywheel_shape.GetCylinderGeometry().p1 = chrono.ChVectorD(0, -flywheel_width/2, 0)
flywheel_shape.GetCylinderGeometry().p2 = chrono.ChVectorD(0, flywheel_width/2, 0)
flywheel.AddVisualShape(flywheel_shape)

system.Add(flywheel)

# Connect flywheel to beam center node with a rigid joint
# The beam center node is at beam_length/2 => node index beam_divisions//2 for even divisions
center_node = nodes[beam_divisions // 2]

# Create a ChLinkMateGeneric to lock translation and rotation between flywheel and beam node
joint = chrono.ChLinkMateGeneric()
joint.SetConstrainedCoords(True, True, True, True, True, True)
joint.SetFlipped(True)
joint.Initialize(flywheel, center_node, chrono.ChFrameD(flywheel_pos))
system.AddLink(joint)

# Alternatively, since nodes are FEA nodes without ChBody,
# and we cannot link ChBody to nodes directly with ChLinkMateGeneric,
# we need to create a ChBodyDummy at the node position and connect the beam node DOFs to it.

# Remove previous link and do this approach:

system.RemoveLink(joint)

# Create a marker body at node's position, fixed to node (use a ChNodeXYZRot)
# We use a ChLinkPointFrame to tie flywheel to beam node's frame

# Create a ChFrame here at node position
marker_node = chrono.ChBody()
marker_node.SetBodyFixed(True)
marker_node.SetPos(flywheel_pos)
system.Add(marker_node)

# Link node motion to marker_node: tie node position to marker_node
link_node = chrono.ChLinkPointFrame()
link_node.Initialize(center_node, marker_node)
system.AddLink(link_node)

# Now fix the flywheel to the marker_node with a revolute joint allowing spin about beam axis (x-axis)
motor_axis = chrono.ChVectorD(1, 0, 0)
rev_joint = chrono.ChLinkLockRevolute()
rev_joint.Initialize(flywheel, marker_node, chrono.ChFrameD(flywheel_pos, chrono.Q_from_AngAxis(0, chrono.VECT_Z)))  # rotation axis= x by default, rotate frame accordingly if needed
system.AddLink(rev_joint)

# 4. Add rotational motor driving the beam at the fixed end (node 0)
# The beam's fixed end node is fixed, so to drive it we apply rotation to the fixed body.
# Since node0 is fixed, we interpret that a motor applies rotation at that end -> drive beam rotation.

# We can simulate motor by imposing a rotation to the fixed node (or via a ChLinkMotorRotationAngularSpeed or directly adding torque at the start node).

# Since the node0 is fixed, to represent motor at this end, we can alternatively create a fixed body at node 0 and a motor between it and the beam.

# Create a dummy body fixed in space, representing the motor base
motor_base = chrono.ChBody()
motor_base.SetBodyFixed(True)
motor_base.SetPos(nodes[0].GetPos())
system.Add(motor_base)

# Create a ChLinkMotorRotationAngle or ChLinkMotorRotationSpeed between motor_base and first node

# But nodes are not bodies. So it's simpler to create a shaft on the beam start node by adding rotational constraint directly on node0 or its equivalent.

# Alternatively, create a ChLinkMotorRotationAngle driving the flywheel or marker_node.

# The best option for Jeffcott rotor is to impose angular velocity at flywheel (center). But instructions say motor drives one end of beam.

# To do this, let's create a ChBody representing the beam start (virtual).

beam_start = chrono.ChBody()
beam_start.SetBodyFixed(True)
beam_start.SetPos(nodes[0].GetPos())
system.Add(beam_start)

# Connect node0 to beam_start with a rigid joint (link node0 to beam_start)

link_node0 = chrono.ChLinkPointFrame()
link_node0.Initialize(nodes[0], beam_start)
system.AddLink(link_node0)

# Now connect a motor between beam_start and ground in x-axis rotation

motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(beam_start, system.Get_bodylist()[0], chrono.ChFrameD(beam_start.GetPos(), chrono.Q_from_AngAxis(chrono.CH_C_PI_2, chrono.VECT_Y)))  # rotation axis: along x (beam axis), frame rotated such that y-axis is rotation axis (motor uses z-axis by default)
motor.SetSpeedFunction(chrono.ChFunction_Const(50))  # constant angular speed 50 rad/s
system.AddLink(motor)

# 5. Visualization setup

# Setup Irrlicht application
application = chronoirr.ChIrrApp(system, "Jeffcott Rotor IGA Beam", chronoirr.dimension2du(1024, 768))

application.AddTypicalLogo()
application.AddTypicalSky()
application.AddTypicalLights()
application.AddTypicalCamera(chronoirr.vector3df(1.5, 0.3, 0.3))

# Add mesh visualization for beam
vis_mymesh = fea.ChVisualizationMesh(mesh)
vis_mymesh.SetFEMdataType(fea.ChVisualizationMesh.E_PLOT_NODE_DISPLACEMENT)
vis_mymesh.SetColorscaleMinMax(-0.005, 0.005)
vis_mymesh.SetSmoothFaces(True)
mesh.AddVisualShape(vis_mymesh)

# Add flywheel visualization already added

application.AssetBindAll()
application.AssetUpdateAll()

# Simulation loop parameters
timestep = 1e-4

application.SetTimestep(timestep)

print("Starting simulation... Press Q or ESC to quit.")

while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()