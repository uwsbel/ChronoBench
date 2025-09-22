import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as irr
import math
import os

# ---------------------------------------------------------------------
# Utility function to create a ChBody for linking FEM nodes
# ---------------------------------------------------------------------
def create_link_body(system, node_pos):
    body = chrono.ChBody()
    body.SetPos(node_pos)
    body.SetFixed(True) # Initially fixed, motor or other constraints will move it
    system.Add(body)
    return body

# ---------------------------------------------------------------------
# Main simulation setup
# ---------------------------------------------------------------------
def main():
    # Create a Chrono system
    system = chrono.ChSystemNSC()
    system.SetGrav(chrono.ChVector3d(0, -9.81, 0)) # Optional: Add gravity

    # Create an FEA mesh
    mesh = fea.ChMesh()
    system.Add(mesh)

    # --- IGA Beam Parameters ---
    beam_L = 1.0  # Length of the beam
    beam_radius = 0.02
    beam_young_modulus = 2.1e11  # Steel
    beam_poisson_ratio = 0.3
    beam_density = 7850  # Steel
    
    num_elements_beam = 1 # Single IGA element for the whole beam
    num_control_points = 5 # Example: for a cubic beam, degree p=3, n_cp = p+num_segments = 3+2=5
                           # Or for degree p=2, n_cp = p+num_segments = 2+3 = 5
    beam_degree = 3 # Polynomial degree of the IGA beam

    # --- Flywheel Parameters ---
    flywheel_mass = 5.0
    flywheel_radius = 0.1
    flywheel_thickness = 0.05

    # --- Motor Parameters ---
    motor_speed_rad_s = 2.0 * math.pi # 1 revolution per second

    # --- Material and Section for IGA Beam ---
    # Create the material
    mat_beam = fea.ChMaterialBeamIGA(beam_young_modulus, 
                                     beam_young_modulus / (2.0 * (1.0 + beam_poisson_ratio)), # G
                                     beam_density)
    
    # Create the section
    # For IGA, we use ChBeamSectionIGA, which can wrap a simpler section type
    # For simplicity, using Euler properties, Cosserat would be more general.
    section_euler = fea.ChBeamSectionEuler()
    section_euler.SetYoungModulus(beam_young_modulus)
    section_euler.SetGshearModulus(beam_young_modulus / (2.0 * (1.0 + beam_poisson_ratio)))
    section_euler.SetBeamRaleyghFactor(0.0) # Optional damping
    section_euler.SetDensity(beam_density)
    section_euler.SetAsCircularSection(beam_radius)

    # Wrap it into an IGA-compatible section
    section_iga = fea.ChBeamSectionIGA()
    section_iga.SetBeamMaterial(mat_beam) # Set the IGA-specific material
    section_iga.SetSectionMyProperties(section_euler.GetInertiaJ(), # Use Euler's J as torsional constant
                                       section_euler.GetArea() * (section_euler.GetYoungModulus() / (2.0 * (1.0 + section_euler.GetPoissonRatio()))), # approx G*A as shear stiffness placeholder
                                       section_euler.GetArea() * (section_euler.GetYoungModulus() / (2.0 * (1.0 + section_euler.GetPoissonRatio())))) # approx G*A as shear stiffness placeholder
    section_iga.SetArtificialJyyJzzFactor(0.0001) # Small factor for stability if needed
    section_iga.SetCircular(True)
    section_iga.SetRadius(beam_radius) # Or use SetDrawCircularRadius()

    # --- Create IGA Beam Nodes (Control Points) ---
    beam_nodes = []
    for i in range(num_control_points):
        x_pos = (beam_L / (num_control_points - 1)) * i
        node = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVector3d(x_pos, 0, 0)))
        mesh.AddNode(node)
        beam_nodes.append(node)

    # --- Create IGA Beam Element ---
    # Knot vector: clamped-clamped like (p+1 zeros, then uniform, then p+1 ones)
    # Total knots = num_control_points + degree + 1
    num_knots = num_control_points + beam_degree + 1
    knots = chrono.vector_double()
    
    # Start with p+1 zeros
    for _ in range(beam_degree + 1):
        knots.push_back(0.0)
    
    # Interior knots (if any)
    # n_internal_knots = num_knots - 2 * (beam_degree + 1)
    # if n_internal_knots > 0:
    #     for i in range(n_internal_knots):
    #         knots.push_back( (i+1) / (n_internal_knots + 1.0) )
    # Simpler: uniform spacing for remaining knots up to 1.0
    # The number of intervals for internal knots is num_control_points - beam_degree
    num_intervals = num_control_points - beam_degree
    if num_intervals > 1 : # Avoid division by zero if num_control_points = beam_degree
        for i in range(1, num_intervals):
            knots.push_back(float(i) / (num_intervals -1) ) # This needs to be checked based on Chrono convention
                                                            # Often (num_control_points - degree -1) internal knots

    # For a single segment (num_control_points = degree + 1), knot vector is just [0,0,...,0, 1,1,...,1]
    # More generally for a C^p-1 continuous beam with m segments, n_cp = p + m.
    # Example for a single segment (m=1): n_cp = p+1. knot vector is p+1 zeros, p+1 ones. size = 2*(p+1).
    # For num_control_points = 5, degree = 3: means 5-3 = 2 segments.
    # knot vector = [0,0,0,0, 0.5, 1,1,1,1] (size 5+3+1 = 9)
    # For this script, let's assume standard open knot vector
    
    # Corrected knot vector for open B-spline (p+1 zeros, p+1 ones, uniform in between)
    knots.clear()
    for _ in range(beam_degree + 1): # p+1 zeros
        knots.push_back(0.0)
    
    # n = num_control_points - 1
    # m = n + beam_degree + 1 (total knots)
    # Number of internal distinct knots = num_control_points - 1 - beam_degree
    num_distinct_internal_knots = num_control_points - 1 - beam_degree
    for i in range(num_distinct_internal_knots):
        knots.push_back( (i+1) / (num_distinct_internal_knots + 1.0) )
        
    for _ in range(beam_degree + 1): # p+1 ones
        knots.push_back(1.0)

    # Create the element
    beam_element = fea.ChElementBeamIGA()
    beam_element.SetNodes(chrono.vector_ChNodeFEAxyzrot(beam_nodes), knots) # Pass nodes and knots
    beam_element.SetSection(section_iga)
    beam_element.SetPolynomialDegree(beam_degree)
    mesh.AddElement(beam_element)

    # --- Create a Ground Body ---
    ground_body = chrono.ChBody()
    ground_body.SetFixed(True)
    ground_body.SetName("ground")
    system.Add(ground_body)
    
    # --- Create Flywheel ---
    # Determine center node index
    center_idx = num_control_points // 2 
    flywheel_pos = beam_nodes[center_idx].GetPos()

    flywheel = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, # Axis of cylinder (thickness direction)
                                         flywheel_radius, flywheel_thickness,
                                         beam_density, # Use beam density for consistency, or specify flywheel density
                                         True, True) # enable visualization and collision
    flywheel.SetPos(flywheel_pos)
    flywheel.SetMass(flywheel_mass)
    # Inertia for a cylinder: Ixx=Izz = m/12*(3*r^2+h^2), Iyy = 1/2*m*r^2 (if Y is axis of thickness)
    # Our beam is along X, flywheel disk perpendicular to X. So, rotation is around X.
    # Flywheel rotates WITH the beam around X axis.
    # Inertia for rotation around X (beam axis): Iz = m/2 * r^2 (if Z is axis of cylinder)
    # Let's assume cylinder axis is along X to align with shaft rotation
    I_axial = 0.5 * flywheel_mass * flywheel_radius * flywheel_radius
    I_perp  = 0.25 * flywheel_mass * flywheel_radius * flywheel_radius + (1/12) * flywheel_mass * flywheel_thickness * flywheel_thickness
    flywheel.SetInertiaXX(chrono.ChVector3d(I_axial, I_perp, I_perp)) # Assuming flywheel axis is along X
    flywheel.SetName("flywheel")
    system.Add(flywheel)

    # --- Constraints ---
    # 1. Motor at one end (node 0)
    #    Connect beam_nodes[0] to ground_body with a motor
    #    We need an intermediate body for ChLinkPointFrame if the motor cannot directly act on a node.
    #    ChLinkMotorRotation needs two ChBody parts.
    
    motor_housing_body = create_link_body(system, beam_nodes[0].GetPos())
    motor_housing_body.SetFixed(True) # Motor housing is fixed to ground conceptually

    # Link beam_nodes[0] to motor_housing_body (this body will be driven by the motor)
    # This setup is a bit indirect. A simpler ChLinkPointFrame to a new body, then motor on that body.
    motor_driven_body = chrono.ChBody()
    motor_driven_body.SetPos(beam_nodes[0].GetPos())
    motor_driven_body.SetMass(1e-6) # Very small mass
    motor_driven_body.SetInertiaXX(chrono.ChVector3d(1e-6,1e-6,1e-6))
    system.Add(motor_driven_body)

    link_node_to_motor_body = fea.ChLinkPointFrame()
    link_node_to_motor_body.Initialize(beam_nodes[0], motor_driven_body) # Connects node to body
    mesh.Add(link_node_to_motor_body)

    # Motor: drives motor_driven_body relative to ground_body (which is fixed)
    # Motor rotates around the X-axis
    motor_frame = chrono.ChFrameD(beam_nodes[0].GetPos(), chrono.Q_from_AngY(math.pi/2)) # Rot axis is Z of this frame
    motor = chrono.ChLinkMotorRotation()
    motor.Initialize(motor_driven_body,          # slave
                     ground_body,       # master
                     motor_frame)       # motor frame (axis of rotation is Z loc)
    
    motor_func = chrono.ChFunction_Ramp(0, motor_speed_rad_s) # 0 intercept, slope = ang_speed
    motor.SetMotorFunction(motor_func)
    system.Add(motor)

    # 2. Attach Flywheel to the center node (node_center_idx)
    link_flywheel = fea.ChLinkPointFrame() # Rigidly attach center node to flywheel
    link_flywheel.Initialize(beam_nodes[center_idx], flywheel)
    mesh.Add(link_flywheel)
    
    # 3. Support at the other end (node -1) - e.g., a simple bearing (spherical joint)
    #    Create a small body for the bearing housing (fixed to ground)
    #    Then connect beam_nodes[-1] to this fixed body using a spherical joint.
    
    bearing_housing_body = create_link_body(system, beam_nodes[-1].GetPos())
    bearing_housing_body.SetFixed(True)

    # We need a body associated with beam_nodes[-1] to link with ChLinkMateSpherical
    end_node_body = chrono.ChBody()
    end_node_body.SetPos(beam_nodes[-1].GetPos())
    end_node_body.SetMass(1e-6)
    end_node_body.SetInertiaXX(chrono.ChVector3d(1e-6,1e-6,1e-6))
    system.Add(end_node_body)

    link_node_to_end_body = fea.ChLinkPointFrame()
    link_node_to_end_body.Initialize(beam_nodes[-1], end_node_body)
    mesh.Add(link_node_to_end_body)
    
    # Spherical joint between end_node_body and ground (via bearing_housing_body)
    # A simpler way: Fix the translation of the end node, allowing rotation.
    # beam_nodes[-1].SetFixed(True) # This fixes rotation too, not ideal for a bearing.
    # So, using a ChLinkMateSpherical is better for a simple support.
    
    link_bearing = chrono.ChLinkMateSpherical()
    link_bearing.Initialize(end_node_body,  # body1 (beam end)
                            bearing_housing_body, # body2 (ground)
                            chrono.ChFrameD(beam_nodes[-1].GetPos())) # abs joint pos
    system.Add(link_bearing)
    
    # --- FEM Visualization ---
    # Create a visual shape for the beam element itself (shows curvature, etc.)
    # Vis টাইপ: shows how the beam shape is rendered.
    # PLOT_BEAM_WIDTH shows a representation of the beam's cross-section.
    vis_beam_shape = fea.ChVisualShapeFEA(mesh)
    vis_beam_shape.SetFEMdataType(fea.ChVisualShapeFEA.DataType.SURFACE)
    vis_beam_shape.SetFEMglyphType(fea.ChVisualShapeFEA.GlyphType.NONE) # Don't show node glyphs from this
    vis_beam_shape.SetSymbolsThickness(0.005)
    vis_beam_shape.SetDefaultGLE اساس(fea.ChVisualShapeFEA.GLE اساس.SOLID)
    vis_beam_shape.SetZbufferHide(False)
    mesh.AddVisualShapeFEA(vis_beam_shape)

    # Create a visual shape for beam element axes
    vis_beam_axes = fea.ChVisualShapeFEA(mesh)
    vis_beam_axes.SetFEMdataType(fea.ChVisualShapeFEA.DataType.NONE) # Not plotting scalar/vector data
    vis_beam_axes.SetFEMglyphType(fea.ChVisualShapeFEA.GlyphType.AXES) # Show element axes
    vis_beam_axes.SetSymbolsThickness(0.01)
    vis_beam_axes.SetDefaultGLE اساس(fea.ChVisualShapeFEA.GLE اساس. चंडीगढ़) # LINES is better for axes
    mesh.AddVisualShapeFEA(vis_beam_axes)
    
    # Optional: Visualize Node CSYS
    vis_node_csys = fea.ChVisualShapeFEA(mesh)
    vis_node_csys.SetFEMdataType(fea.ChVisualShapeFEA.DataType.NONE)
    vis_node_csys.SetFEMglyphType(fea.ChVisualShapeFEA.GlyphType.NODE_CSYS)
    vis_node_csys.SetSymbolsThickness(0.015) # Make them a bit larger
    mesh.AddVisualShapeFEA(vis_node_csys)

    # --- Irrlicht Visualization Setup ---
    vis = irr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 768)
    vis.SetWindowTitle('IGA Jeffcott Rotor Simulation')
    vis.Initialize()
    vis.AddLogo(os.path.join(chrono.GetChronoDataPath(), 'logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(beam_L * 0.5, beam_L * 0.5, -beam_L * 1.5)) # Camera position
    vis.AddTypicalLights()
    vis.EnableCollisionShapeDrawing(True) # Show collision shapes if any

    # --- Solver and Timestepper ---
    # system.SetSolverType(chrono.ChSolver.Type.MINRES) # Good for FEA
    # solver = system.GetSolver()
    # solver.AsIterative().SetMaxIterations(200)
    # solver.AsIterative().SetTolerance(1e-10)
    # solver.AsIterative().EnableDiagonalPreconditioner(True)
    
    # Or use a direct solver for potentially better stability with IGA
    system.SetSolverType(chrono.ChSolver.Type.SPARSE_LU)
    
    system.SetTimestepperType(chrono.ChTimestepper.Type.EULER_IMPLICIT_LINEARIZED) # Good for stiff systems
    # system.SetTimestepperType(chrono.ChTimestepper.Type.HHT)
    # if isinstance(system.GetTimestepper(), chrono.ChTimestepperHHT):
    #     system.GetTimestepper().SetAlpha(-0.2) # HHT alpha, range [-1/3, 0]
    #     system.GetTimestepper().SetStepControl(False)
    #     system.GetTimestepper().SetMaxiters(10)
    #     system.GetTimestepper().SetAbsoler(1e-4)

    # --- Simulation Loop ---
    time_step = 0.005
    end_time = 10.0

    # Pre-analysis to ensure constraints are well-formed (optional, but good for complex systems)
    system.Setup()
    system.Update()

    print(f"Number of control points: {num_control_points}, Degree: {beam_degree}")
    print("Knot vector: ", [knots[i] for i in range(knots.size())])
    print(f"Simulating Jeffcott Rotor with IGA Beam. Motor speed: {motor_speed_rad_s:.2f} rad/s")
    print(f"Beam Length: {beam_L} m, Radius: {beam_radius} m")
    print(f"Flywheel Mass: {flywheel_mass} kg, Radius: {flywheel_radius} m")

    while vis.Run():
        vis.BeginScene()
        vis.Render()
        # Optionally draw link frames, etc.
        # irr.draw_ChFrame(vis, motor_frame, 0.2)
        
        # chrono.ChronoGlobals.ArchiveOUT.Stream().seek(0) # Reset for ChArchiveAscii dump
        # system.ArchiveOUT(chrono.ChArchiveAscii(chrono.ChronoGlobals.ArchiveOUT)) # Dump system state

        vis.EndScene()
        system.DoStepDynamics(time_step)

        if system.GetChTime() > end_time:
            vis.GetDevice().closeDevice() # Close window after simulation time
            break
            
    print("Simulation finished.")

if __name__ == "__main__":
    main()