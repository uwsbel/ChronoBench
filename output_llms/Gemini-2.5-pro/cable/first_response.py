import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as irr
import math
import os

def main():
    # 0. Set path to Chrono data directory
    chrono.SetChronoDataPath(os.environ.get('CHRONO_DATA_DIR', '../../../chrono_data/'))

    # 1. Initialize PyChrono environment and core components
    # Create a Chrono system
    system = chrono.ChSystemSMC()
    system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0)) # Gravity in -Y direction

    # Create an Irrlicht visualizer
    application = irr.ChIrrApp(system, "ANCF Cable Beam Simulation", irr.dimension2du(1024, 768))
    application.AddTypicalSky()
    application.AddTypicalLights()
    application.AddTypicalCamera(irr.vector3df(0.5, 0.5, 1.5), irr.vector3df(0.5, -0.2, 0)) # Camera position and target
    application.SetContactsDrawMode(irr.CONTACT_NONE) # No contact visualization

    # Create a ChMesh
    mesh = fea.ChMesh()
    system.Add(mesh)

    # 2. Add the required physical systems and objects
    # Beam properties
    beam_length = 1.0
    num_elements = 10
    num_nodes = num_elements + 1
    element_length = beam_length / num_elements
    beam_radius = 0.02 # Radius of the cable cross-section

    # Material properties (e.g., steel)
    density = 7850  # kg/m^3
    E_modulus = 210e9  # Pa (Young's modulus)
    poisson_ratio = 0.3
    alpha_damping = 0.005 # Rayleigh damping alpha (mass proportional)
    beta_damping = 0.001  # Rayleigh damping beta (stiffness proportional)

    # Create ANCF beam material
    ancf_material = fea.ChMaterialBeamANCF()
    ancf_material.Set_rho(density)
    ancf_material.Set_E(E_modulus)
    ancf_material.Set_v(poisson_ratio)
    ancf_material.Set_alpha_damping(alpha_damping) # Adding some damping for stability
    # ancf_material.Set_beta_damping(beta_damping) # Stiffness damping can be added too

    # Create ANCF beam section (cable type)
    # For ChBeamSectionCable, diameter is the primary input.
    # It automatically computes Area, Izz, Iyy based on a circular section.
    beam_section = fea.ChBeamSectionCable()
    beam_section.SetDiameter(2 * beam_radius)
    beam_section.SetMaterial(ancf_material)
    beam_section.SetDrawThickness(beam_radius) # For visualization thickness

    # Create nodes and elements
    nodes = []
    for i in range(num_nodes):
        # Nodes are created along the X-axis
        node_pos = chrono.ChVectorD(i * element_length, 0, 0)
        # Director vector, initially along the beam axis (X-axis)
        node_dir = chrono.ChVectorD(1, 0, 0)
        
        node = fea.ChNodeFEAxyzD(node_pos, node_dir)
        nodes.append(node)
        mesh.AddNode(node)

    # Create elements
    for i in range(num_elements):
        element = fea.ChElementBeamANCF()
        element.SetNodes(nodes[i], nodes[i+1])
        element.SetSection(beam_section) # Assign section properties (which includes material)
        mesh.AddElement(element)

    # 3. Set necessary default parameters
    # Hinge one end to the ground (fix the first node)
    # This creates a cantilever beam, which is a common interpretation of "hinged"
    # for one end of a flexible beam unless rotational freedom is explicitly stated.
    nodes[0].SetFixed(True)

    # Visualization of the FEA mesh
    # For beam elements
    vis_beam_elements = chrono.ChVisualShapeFEA(mesh)
    vis_beam_elements.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE) # No data plot on elements
    vis_beam_elements.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NONE) # No glyphs from this shape
    vis_beam_elements.SetSymbolsThickness(beam_radius * 0.5) # Affects line thickness for beam axis if DataType_BEAM_AXIS
    vis_beam_elements.SetColor(chrono.ChColor(0.2, 0.5, 0.8)) # Blue-ish color for the beam
    vis_beam_elements.SetWireframe(False) # Render as solid
    vis_beam_elements.SetBeamSections(True) # Render the beam cross-section shape
    mesh.AddVisualShapeFEA(vis_beam_elements)

    # For nodal positions
    vis_beam_nodes = chrono.ChVisualShapeFEA(mesh)
    vis_beam_nodes.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE) # No data plot
    vis_beam_nodes.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS) # Show nodes as dots
    vis_beam_nodes.SetSymbolsThickness(0.01) # Size of the node dots
    vis_beam_nodes.SetSymbolsColor(chrono.ChColor(1, 0, 0)) # Red color for nodes
    mesh.AddVisualShapeFEA(vis_beam_nodes)
    
    # Set up the HHT implicit integrator for FEA (good for stiff problems)
    hht_stepper = chrono.ChTimestepperHHT(mesh) # Pass the mesh to the HHT stepper
    hht_stepper.SetAlpha(-0.2)  # Default HHT alpha factor
    hht_stepper.SetMaxiters(100)
    hht_stepper.SetAbsolerance(1e-5)
    hht_stepper.SetMode(chrono.ChTimestepperHHT.POSITION) # Solve for position
    hht_stepper.SetVerbose(False)
    hht_stepper.SetStepControl(True) # Enable step control
    hht_stepper.SetModifiedNewton(False) # Use full Newton (can be more robust)
    system.SetTimestepper(hht_stepper)

    # Set solver (MKL if available, otherwise fallback)
    try:
        mkl_solver = chrono.ChSolverMKL()
        system.SetSolver(mkl_solver)
        print("Using MKL solver.")
    except AttributeError:
        print("MKL solver not available, using SPARSE_LU.")
        # Fallback to another sparse direct solver if MKL is not available
        # Note: Make sure CHOLMOD or another sparse solver is compiled in.
        # For simplicity, if MKL is not there, SPARSE_LU is often a good alternative.
        # If even that is not available, MINRES is an iterative option.
        try:
            lu_solver = chrono.ChSolverSparseLU()
            system.SetSolver(lu_solver)
        except AttributeError:
            print("SPARSE_LU solver not available, using MINRES.")
            minres_solver = chrono.ChSolverMINRES()
            system.SetSolver(minres_solver)


    # Simulation loop
    application.SetTimestep(0.01) # Timestep for rendering and simulation
    application.SetVideoframeSaveInterval(int(1 / (application.GetTimestep() * 25))) # For 25 FPS video

    # Optional: Output node positions to a file
    output_file_path = "beam_node_positions.txt"
    with open(output_file_path, "w") as outfile:
        outfile.write("Time NodeIndex X Y Z\n")

    simulation_time = 5.0 # Total simulation time in seconds
    time = 0

    while application.GetDevice().run():
        application.BeginScene()
        application.DrawAll()
        
        # Print node positions (e.g., tip node) to console for quick check
        if len(nodes) > 0:
            tip_node = nodes[-1] # Last node
            # print(f"Time: {system.GetChTime():.2f} Tip Pos: {tip_node.GetPos().x:.3f}, {tip_node.GetPos().y:.3f}, {tip_node.GetPos().z:.3f}")
            
            # Write node positions to file
            # with open(output_file_path, "a") as outfile:
            #     for i, node in enumerate(nodes):
            #         pos = node.GetPos()
            #         outfile.write(f"{system.GetChTime():.4f} {i} {pos.x:.6f} {pos.y:.6f} {pos.z:.6f}\n")

        application.DoStep()
        application.EndScene()
        
        time = system.GetChTime()
        if time > simulation_time:
            break
            
    print(f"Simulation finished. Node positions saved to {output_file_path}")
    application.GetDevice().closeDevice()

if __name__ == "__main__":
    main()