"""
PyChrono Beam Finite Elements Simulation
========================================
This script demonstrates the use of beam finite elements in PyChrono.
It creates a beam structure with nodes and visualizes it using Irrlicht.
"""

# Import necessary PyChrono modules
import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math
import numpy as np

def create_beam_simulation():
    """
    Create and run a beam finite element simulation with visualization.
    """
    
    # =====================================================================
    # 1. INITIALIZE PYCHRONO ENVIRONMENT
    # =====================================================================
    
    # Create a physical system (Smooth Contact Mechanics - for rigid bodies)
    system = chrono.ChSystemSMC()
    
    # Set simulation parameters
    system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Gravity
    system.SetSolverType(chrono.ChSolver.Type_SOR)    # Solver type
    system.SetMaxItersSolverSpeed(100)               # Max iterations
    system.SetStepSize(0.001)                         # Time step
    
    # Set contact properties
    system.SetUseMaterialProperties(False)
    
    print("PyChrono environment initialized successfully!")
    print(f"System gravity: {system.Get_G_acc()}")
    
    # =====================================================================
    # 2. CREATE BEAM STRUCTURE WITH NODES AND ELEMENTS
    # =====================================================================
    
    # Define material properties for the beam
    # Using a structural steel material
    E = 210e9      # Young's modulus [Pa]
    nu = 0.3       # Poisson's ratio
    rho = 7850.0   # Density [kg/m³]
    yield_stress = 250e6  # Yield stress [Pa]
    
    # Create beam material
    beam_material = chrono.ChBeamSectionTimoshenkoRD()
    
    # Set circular cross-section properties
    radius = 0.02  # Beam radius [m]
    beam_material.SetCircular(radius, E, nu, rho, yield_stress)
    
    print(f"\nBeam Properties:")
    print(f"  - Radius: {radius} m")
    print(f"  - Young's Modulus: {E/1e9} GPa")
    print(f"  - Density: {rho} kg/m³")
    print(f"  - Poisson's Ratio: {nu}")
    
    # =====================================================================
    # 3. CREATE BEAM NODES AND ELEMENTS
    # =====================================================================
    
    # Define beam structure geometry
    num_elements = 10           # Number of beam elements
    beam_length = 2.0          # Total beam length [m]
    element_length = beam_length / num_elements
    
    # Storage for beam elements and nodes
    nodes = []
    beams = []
    
    # Create nodes along the beam
    print(f"\nCreating beam structure:")
    print(f"  - Number of elements: {num_elements}")
    print(f"  - Total length: {beam_length} m")
    print(f"  - Element length: {element_length} m")
    
    for i in range(num_elements + 1):
        # Calculate node position
        x_pos = i * element_length
        y_pos = 0.0
        z_pos = 0.0
        
        # Create a beam node with 6 DOF (position, rotation)
        node = chrono.ChNodeFEAxyzD())
        node.SetPos(chrono.ChVectorD(x_pos, y_pos, z_pos))
        node.SetDOF(chrono.ChVectorD(0, 0, 0))
        node.SetVel(chrono.ChVectorD(0, 0, 0))
        node.SetAngularVel(chrono.ChVectorD(0, 0, 0))
        
        # Add node to the system
        system.Add(node)
        nodes.append(node)
        
        # Print node creation info for first few and last
        if i <= 2 or i >= num_elements - 1:
            print(f"  Created node {i}: pos=({x_pos:.3f}, {y_pos:.3f}, {z_pos:.3f})")
    
    # Create beam elements connecting consecutive nodes
    print(f"\nCreating beam elements...")
    for i in range(num_elements):
        # Get the two nodes for this beam element
        node_a = nodes[i]
        node_b = nodes[i + 1]
        
        # Create a Timoshenko beam element (accounts for shear deformation)
        beam = chrono.ChElementTimoshenko()
        beam.SetNodes(node_a, node_b)
        beam.SetSection(beam_material)
        
        # Set initial curvature (optional)
        # beam.SetMasonryDamping(0.1)
        
        # Add beam to the system
        system.Add(beam)
        beams.append(beam)
        
        if i <= 2 or i >= num_elements - 2:
            print(f"  Created beam element {i}: nodes {i} to {i+1}")
    
    # =====================================================================
    # 4. ADD BOUNDARY CONDITIONS AND LOADS
    # =====================================================================
    
    print("\nApplying boundary conditions and loads:")
    
    # Fix the first node (cantilever support)
    fixed_node = nodes[0]
    fixed_node.SetFixed(True)
    print(f"  - Fixed node 0 (cantilever support)")
    
    # Apply a point load at the free end
    load_magnitude = 100.0  # [N]
    load_force = chrono.ChVectorD(0, -load_magnitude, 0)
    nodes[-1].SetForce(load_force)
    print(f"  - Applied point load at node {len(nodes)-1}: {load_magnitude} N downward")
    
    # Add a distributed load along the beam (self-weight)
    distributed_load = rho * math.pi * radius**2 * 9.81  # Weight per unit length
    for beam in beams:
        beam.SetLoad(chrono.ChVectorD(0, -distributed_load, 0))
    print(f"  - Applied self-weight: {distributed_load:.4f} N/m")
    
    # =====================================================================
    # 5. ADD VISUALIZATION GEOMETRY
    # =====================================================================
    
    # Create a ground body for reference
    ground = chrono.ChBody()
    ground.SetBodyFixed(True)
    ground.SetPos(chrono.ChVectorD(0, -0.5, 0))
    
    # Create ground visualization shape
    ground_shape = chrono.ChBoxShape()
    ground_shape.SetLengths(chrono.ChVectorD(5, 0.1, 2))
    ground.AddAsset(ground_shape)
    
    # Add color to ground
    ground_color = chrono.ChColorAsset()
    ground_color.SetColor(chrono.ChColor(0.4, 0.4, 0.4))
    ground.AddAsset(ground_color)
    
    system.Add(ground)
    
    # Add visualization assets to beam nodes
    for i, node in enumerate(nodes):
        # Create a small sphere at each node for visualization
        node_sphere = chrono.ChSphereShape()
        node_sphere.GetSphereGeometry().rad = 0.015
        node.AddAsset(node_sphere)
        
        # Add node number label
        node_label = chrono.ChTextShape()
        node_label.SetText(f"N{i}")
        node_label.SetColor(chrono.ChColor(0.2, 0.8, 0.2))
        node.AddAsset(node_label)
    
    # Add visualization shapes to beams
    for i, beam in enumerate(beams):
        # Create a cylinder for each beam element
        beam_cyl = chrono.ChCylinderShape()
        beam_cyl.GetCylinderGeometry().rad = radius
        beam.AddAsset(beam_cyl)
    
    print("\nVisualization assets created")
    
    # =====================================================================
    # 6. SET UP IRRLICHT VISUALIZATION
    # =====================================================================
    
    print("\nInitializing Irrlicht visualization...")
    
    # Create Irrlicht application
    application = chronoirr.ChIrrApp(
        system,                           # The physical system
        "Beam Finite Elements Demo",       # Window title
        chronoirr.dimension2du(1280, 720), # Window size
        chronoirr.VerticalLayout_STACKED   # Layout
    )
    
    # Setup application
    application.AddTypicalLogo()
    application.AddTypicalSky()
    application.AddTypicalCamera(
        chronoirr.vector3df(1.5, 1.0, 2.5),  # Camera position
        chronoirr.vector3df(1.0, 0.0, 0.0)   # Look-at position
    )
    application.AddTypicalLights()
    
    # Set the application to draw all objects
    application.DrawAll
    
    # =====================================================================
    # 7. RUN SIMULATION LOOP
    # =====================================================================
    
    print("\n" + "="*60)
    print("STARTING SIMULATION")
    print("="*60)
    
    # Simulation parameters
    simulation_time = 5.0  # Total simulation time [s]
    frame_rate = 30       # Target frame rate for output
    
    # Custom output interval
    output_interval = 1.0 / frame_rate
    
    # Time tracking
    current_time = 0.0
    last_output_time = 0.0
    
    # Run the simulation loop
    while(application.GetDevice().run()):
        # Check if simulation should stop
        if current_time >= simulation_time:
            print(f"\nSimulation completed at t = {current_time:.2f} s")
            break
        
        # Advance the physics simulation
        application.BeginScene()
        application.DrawAll()
        
        # Advance physics by one step
        system.DoStepDynamics(application.GetStep())
        
        # Update current time
        current_time = system.GetChTime()
        
        # Periodic output
        if current_time - last_output_time >= output_interval:
            last_output_time = current_time
            
            # Get displacement of the free end
            if len(nodes) > 0:
                displacement = nodes[-1].GetPos()
                velocity = nodes[-1].GetVel()
                
                # Print status every second
                if int(current_time) != int(current_time - application.GetStep()):
                    print(f"t = {current_time:.2f} s | "
                          f"End displacement: ({displacement.x:.4f}, "
                          f"{displacement.y:.4f}, {displacement.z:.4f}) m | "
                          f"Velocity: ({velocity.x:.4f}, {velocity.y:.4f}, "
                          f"{velocity.z:.4f}) m/s")
        
        application.EndScene()
    
    # =====================================================================
    # 8. PRINT FINAL RESULTS
    # =====================================================================
    
    print("\n" + "="*60)
    print("SIMULATION RESULTS")
    print("="*60)
    
    if len(nodes) > 0:
        final_node = nodes[-1]
        initial_pos = chrono.ChVectorD(beam_length, 0, 0)
        final_pos = final_node.GetPos()
        displacement = final_pos - initial_pos
        
        print(f"\nFinal position of free end:")
        print(f"  X: {final_pos.x:.6f} m")
        print(f"  Y: {final_pos.y:.6f} m")
        print(f"  Z: {final_pos.z:.6f} m")
        print(f"\nTotal displacement from initial position:")
        print(f"  ΔX: {displacement.x:.6f} m")
        print(f"  ΔY: {displacement.y:.6f} m")
        print(f"  ΔZ: {displacement.z:.6f} m")
        
        # Calculate total displacement magnitude
        total_disp = math.sqrt(displacement.x**2 + displacement.y**2 + displacement.z**2)
        print(f"\nMagnitude of total displacement: {total_disp:.6f} m")
        
        # Compare with analytical cantilever beam formula
        # δ = PL³/(3EI) for cantilever with end load
        I = math.pi * radius**4 / 4  # Second moment of area
        analytical_deflection = (load_magnitude * beam_length**3) / (3 * E * I)
        print(f"\nAnalytical cantilever deflection (end load only): "
              f"{analytical_deflection:.6f} m")
        
        # Compare numerical and analytical
        if analytical_deflection > 0:
            error_percent = abs(total_disp - analytical_deflection) / analytical_deflection * 100
            print(f"Error: {error_percent:.2f}%")
    
    # Calculate maximum stress in beams (simplified)
    print(f"\nBeam stress analysis:")
    for i, beam in enumerate(beams):
        # Get beam internal forces (simplified calculation)
        # In real simulation, these would come from the element state
        section = beam.GetSection()
        if section:
            area = math.pi * radius**2
            axial_force = distributed_load * beam_length / 2  # Approximate
            axial_stress = abs(axial_force) / area
            print(f"  Beam {i}: Approximate axial stress = {axial_stress/1e6:.2f} MPa")
    
    print("\n" + "="*60)
    print("Simulation finished successfully!")
    print("="*60)
    
    return system, nodes, beams

def create_multi_span_beam():
    """
    Create a more complex multi-span beam simulation.
    """
    print("\n" + "="*60)
    print("CREATING MULTI-SPAN BEAM SIMULATION")
    print("="*60)
    
    # Create a new system
    system = chrono.ChSystemSMC()
    system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
    system.SetSolverType(chrono.ChSolver.Type_SOR)
    system.SetStepSize(0.001)
    
    # Beam properties
    E = 200e9      # Young's modulus [Pa]
    nu = 0.3       # Poisson's ratio
    rho = 7800.0   # Density [kg/m³]
    radius = 0.015  # Beam radius [m]
    
    # Create beam material
    beam_material = chrono.ChBeamSectionTimoshenkoRD()
    beam_material.SetCircular(radius, E, nu, rho, 250e6)
    
    # Create a continuous beam with two spans
    span_length = 1.5  # Length of each span
    num_elements_per_span = 8
    total_elements = num_elements_per_span * 2 + 1  # Including middle support
    element_length = span_length / num_elements_per_span
    
    # Create nodes
    nodes = []
    for i in range(total_elements + 1):
        x_pos = i * element_length
        y_pos = 0.0
        
        node = chrono.ChNodeFEAxyzD()
        node.SetPos(chrono.ChVectorD(x_pos, y_pos, 0))
        
        # Fix both ends
        if i == 0 or i == total_elements:
            node.SetFixed(True)
        
        system.Add(node)
        nodes.append(node)
    
    # Create beam elements
    beams = []
    for i in range(total_elements):
        beam = chrono.ChElementTimoshenko()
        beam.SetNodes(nodes[i], nodes[i + 1])
        beam.SetSection(beam_material)
        system.Add(beam)
        beams.append(beam)
    
    # Add intermediate support
    middle_support = chrono.ChNodeFEAxyzD()
    middle_support.SetPos(chrono.ChVectorD(span_length, -0.1, 0))
    middle_support.SetFixed(True)
    system.Add(middle_support)
    
    # Apply distributed load
    distributed_load = rho * math.pi * radius**2 * 9.81
    for beam in beams:
        beam.SetLoad(chrono.ChVectorD(0, -distributed_load * 10, 0))  # Amplified for visibility
    
    # Apply point loads
    load_point = 50.0  # N
    nodes[4].SetForce(chrono.ChVectorD(0, -load_point, 0))  # Near middle of first span
    nodes[11].SetForce(chrono.ChVectorD(0, -load_point, 0))  # Near middle of second span
    
    print(f"Created {len(nodes)} nodes and {len(beams)} beam elements")
    print(f"Two-span beam: {span_length}m + {span_length}m")
    print(f"Applied loads: Distributed self-weight + point loads")
    
    return system

def main():
    """
    Main function to run the beam simulation.
    """
    print("\n" + "="*60)
    print(" PyChrono Beam Finite Elements Simulation")
    print("="*60)
    print("\nThis simulation demonstrates:")
    print("  1. Creation of beam finite elements (Timoshenko beams)")
    print("  2. Application of boundary conditions")
    print("  3. Loading (point loads and distributed loads)")
    print("  4. Real-time visualization with Irrlicht")
    print("  5. Post-processing and results analysis")
    
    try:
        # Run the main cantilever beam simulation
        system, nodes, beams = create_beam_simulation()
        
        # Optionally run additional simulations
        # Uncomment to run multi-span beam simulation
        # multi_system = create_multi_span_beam()
        
        print("\n✓ Simulation completed successfully!")
        
    except Exception as e:
        print(f"\n✗ Error during simulation: {str(e)}")
        import traceback
        traceback.print_exc()
        return 1
    
    return 0

if __name__ == "__main__":
    exit(main())