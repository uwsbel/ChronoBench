#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
PyChrono ANCF Cable Beam Simulation
====================================
This script simulates a flexible beam composed of ANCF (Absolute Nodal 
Coordinate Formulation) cable elements. One end is hinged to the ground
and the beam deforms under gravity.
"""

import pychrono as chrono
import pychrono.irrlicht as irr
import math


def main():
    # ==========================================================================
    # 1. INITIALIZE PYCHRONO ENVIRONMENT
    # ==========================================================================
    
    # Set the path to Chrono data directory (for textures and assets)
    chrono.SetChronoDataPath("../../../data/")
    
    # Create the Chrono system with non-smooth contact (NSC) solver
    my_system = chrono.ChSystemNSC()
    
    # Set gravity (Y-direction downward)
    my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
    
    # Set simulation timestep
    my_system.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT)
    my_system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
    
    # ==========================================================================
    # 2. CREATE GROUND/ANCHOR STRUCTURE
    # ==========================================================================
    
    # Create a ground body (visual reference)
    ground_size = 2.0
    ground_height = 0.2
    ground = chrono.ChBodyEasyBox(
        ground_size, ground_height, ground_size,
        1000,  # density
        True,  # collide
        False  # visible
    )
    ground.SetPos(chrono.ChVectorD(0, -ground_height/2 - 0.5, 0))
    ground.SetBodyFixed(True)
    ground.SetName("Ground")
    my_system.AddBody(ground)
    
    # Create anchor post (fixed point for the beam)
    anchor_post = chrono.ChBodyEasyCylinder(
        0.05,  # radius
        1.0,   # height
        1000,  # density
        True
    )
    anchor_post.SetPos(chrono.ChVectorD(0, -0.5, 0))
    anchor_post.SetBodyFixed(True)
    anchor_post.SetName("Anchor Post")
    my_system.AddBody(anchor_post)
    
    # ==========================================================================
    # 3. CREATE ANCF CABLE BEAM ELEMENTS
    # ==========================================================================
    
    # Beam configuration parameters
    num_elements = 15          # Number of cable elements
    beam_length = 5.0          # Total beam length (m)
    element_length = beam_length / num_elements
    
    # Material properties (Steel)
    E = 2.1e11      # Young's modulus (Pa)
    nu = 0.3        # Poisson's ratio
    rho = 7850.0    # Density (kg/m^3)
    
    # Cross-sectional properties (circular beam)
    beam_radius = 0.02         # Beam radius (m)
    area = math.pi * beam_radius**2
    Izz = math.pi * beam_radius**4 / 4  # Second moment of area
    J = 2 * Izz                         # Torsional constant
    
    # Initialize beam nodes list
    beam_nodes = []
    beam_elements = []
    
    # Create beam nodes using ChNodeCableANCF
    print("Creating ANCF cable beam nodes...")
    for i in range(num_elements + 1):
        # Calculate node position along X-axis
        x_pos = i * element_length
        pos = chrono.ChVectorD(x_pos, 0.5, 0)
        
        # Create ANCF cable node with position and initial velocity
        node = chrono.ChNodeCableANCF(pos)
        
        # Set initial conditions for first node (at origin)
        if i == 0:
            node.SetFixed(True)  # Hinge constraint - fixed in space
            print(f"  Node {i}: Fixed at {pos.x}, {pos.y}, {pos.z}")
        else:
            print(f"  Node {i}: Free at {pos.x}, {pos.y}, {pos.z}")
        
        # Add node to system
        node.SetName(f"BeamNode_{i}")
        my_system.Add(node)
        beam_nodes.append(node)
    
    # Create ANCF cable elements connecting nodes
    print("\nCreating ANCF cable elements...")
    for i in range(num_elements):
        # Get the two nodes for this element
        nodeA = beam_nodes[i]
        nodeB = beam_nodes[i + 1]
        
        # Create ANCF cable element
        element = chrono.ChElementCableANCF()
        
        # Set the nodes for this element
        element.SetNodes(nodeA, nodeB)
        
        # Set element properties
        element.SetData(E, nu, rho, area, Izz, J)
        
        # Set element name
        element.SetName(f"BeamElement_{i}")
        
        # Add element to system
        my_system.Add(element)
        beam_elements.append(element)
        
        print(f"  Element {i}: Nodes {i} to {i+1}")
    
    print(f"\nBeam created with {num_elements} elements and {num_elements + 1} nodes")
    
    # ==========================================================================
    # 4. APPLY LOADS AND CONSTRAINTS
    # ==========================================================================
    
    # Gravity is already applied to the system (set in step 1)
    # The beam will naturally sag under its own weight
    
    # Apply a small initial perturbation to make the beam deflect
    # (otherwise it would stay perfectly horizontal)
    perturbation = 0.001
    for i in range(1, len(beam_nodes)):
        node = beam_nodes[i]
        current_pos = node.GetPos()
        # Add small perturbation in Y direction
        new_pos = chrono.ChVectorD(
            current_pos.x,
            current_pos.y + perturbation * math.sin(i * 0.5),
            current_pos.z + perturbation * math.cos(i * 0.3)
        )
        node.SetPos(new_pos)
    
    # ==========================================================================
    # 5. SET UP IRRLICHT VISUALIZATION
    # ==========================================================================
    
    print("\nInitializing Irrlicht visualization...")
    
    # Create Irrlicht application
    myapplication = irr.ChIrrApp(
        my_system,           # The Chrono system
        "ANCF Cable Beam",   # Window title
        irr.dimension2du(1280, 720),  # Window size
        irr.VerticalLayout_ID
    )
    
    # Setup basic assets
    myapplication.AddTypicalLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    myapplication.AddTypicalSky()
    myapplication.AddTypicalCamera(irr.vector3df(3.0, 2.0, 4.0))
    myapplication.AddTypicalLight(irr.vector3df(0.5, 0.8, 0.6), 0.7)
    
    # Bind assets for visualization
    myapplication.AssetBinding(my_system)
    
    # Add all visualization assets
    myapplication.AddAllAssets()
    
    # Set up custom visualization for beam nodes and elements
    # Create a visual representation of the beam nodes
    for i, node in enumerate(beam_nodes):
        if i == 0:
            # First node - show as fixed anchor (green sphere)
            ball_mat = chrono.ChVisualMaterial()
            ball_mat.SetDiffuseColor(chrono.ChColor(0.0, 1.0, 0.0))  # Green
            ball_mat.SetSpecularColor(chrono.ChColor(0.3, 0.3, 0.3))
        else:
            # Other nodes - show as regular nodes (blue spheres)
            ball_mat = chrono.ChVisualMaterial()
            ball_mat.SetDiffuseColor(chrono.ChColor(0.0, 0.5, 1.0))  # Blue
            ball_mat.SetSpecularColor(chrono.ChColor(0.3, 0.3, 0.3))
        
        # Create a small sphere to visualize the node
        node_sphere = chrono.ChSphereShape(0.03)
        node_sphere.AddMaterial(ball_mat)
        node.GetAssets().push_back(node_sphere)
    
    # ==========================================================================
    # 6. SIMULATION LOOP
    # ==========================================================================
    
    print("\nStarting simulation...")
    print("=" * 60)
    print("Simulation Parameters:")
    print(f"  - Number of elements: {num_elements}")
    print(f"  - Beam length: {beam_length} m")
    print(f"  - Young's modulus: {E:.2e} Pa")
    print(f"  - Density: {rho} kg/m³")
    print(f"  - Gravity: 9.81 m/s² (downward)")
    print("=" * 60)
    print("\nSimulation running... Close the window to stop.")
    
    # Simulation timing
    simulation_time = 0.0
    output_interval = 0.5  # Output every 0.5 seconds
    last_output_time = 0.0
    
    # Run the simulation loop
    myapplication.SetTimestep(0.002)  # 2ms timestep
    
    while myapplication.GetDevice().run():
        # Start rendering frame
        myapplication.BeginScene()
        
        # Clear the screen
        myapplication.DrawAll()
        
        # Render the system
        myapplication.Render()
        
        # Draw additional information
        device = myapplication.GetDevice()
       guienv = device.getGUIEnvironment()
        
        # Create info text
        if guienv:
            # Clear old GUI
            guienv.clear()
            
            # Add title
            guienv.addStaticText(
                "ANCF Cable Beam Simulation",
                irr.rect<s32>(10, 10, 300, 30),
                False
            )
            
            # Add simulation info
            sim_text = f"Time: {simulation_time:.2f} s"
            guienv.addStaticText(
                sim_text,
                irr.rect<s32>(10, 40, 250, 60),
                False
            )
            
            # Add node positions info
            if len(beam_nodes) > 1:
                last_node = beam_nodes[-1]
                pos = last_node.GetPos()
                end_text = f"End position: ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f})"
                guienv.addStaticText(
                    end_text,
                    irr.rect<s32>(10, 70, 400, 100),
                    False
                )
        
        # Advance the simulation
        myapplication.DoStep()
        
        # Update simulation time
        simulation_time = my_system.GetChTime()
        
        # Output periodically
        if simulation_time - last_output_time >= output_interval:
            # Print simulation status
            print(f"\n[{simulation_time:.2f}s] Beam Status:")
            print(f"  End node position: ({beam_nodes[-1].GetPos().x:.4f}, "
                  f"{beam_nodes[-1].GetPos().y:.4f}, "
                  f"{beam_nodes[-1].GetPos().z:.4f})")
            print(f"  End node velocity: ({beam_nodes[-1].GetPosDt().x:.4f}, "
                  f"{beam_nodes[-1].GetPosDt().y:.4f}, "
                  f"{beam_nodes[-1].GetPosDt().z:.4f})")
            
            # Print all node positions
            print("  Node positions:")
            for i, node in enumerate(beam_nodes):
                p = node.GetPos()
                print(f"    Node {i}: ({p.x:.4f}, {p.y:.4f}, {p.z:.4f})")
            
            last_output_time = simulation_time
        
        # End rendering frame
        myapplication.EndScene()
    
    # ==========================================================================
    # 7. FINAL RESULTS
    # ==========================================================================
    
    print("\n" + "=" * 60)
    print("SIMULATION COMPLETED")
    print("=" * 60)
    print("\nFinal Node Positions:")
    for i, node in enumerate(beam_nodes):
        p = node.GetPos()
        v = node.GetPosDt()
        print(f"  Node {i:2d}: Pos=({p.x:.4f}, {p.y:.4f}, {p.z:.4f}) "
              f"Vel=({v.x:.4f}, {v.y:.4f}, {v.z:.4f})")
    
    # Calculate beam deflection at the free end
    initial_end_y = beam_length + 0.5
    final_end_y = beam_nodes[-1].GetPos().y
    max_deflection = initial_end_y - final_end_y
    
    print(f"\nMaximum Beam Deflection: {max_deflection:.4f} m")
    print(f"Initial end height: {initial_end_y:.4f} m")
    print(f"Final end height: {final_end_y:.4f} m")
    
    return 0


if __name__ == "__main__":
    # Run the main simulation
    exit_code = main()
    exit(exit_code)