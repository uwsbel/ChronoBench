import pychrono as chrono
import pychrono.irrlicht as irr
import math


def main():
    
    
    
    
    
    chrono.SetChronoDataPath("../../../data/")
    
    
    my_system = chrono.ChSystemNSC()
    
    
    my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
    
    
    my_system.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT)
    my_system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
    
    
    
    
    
    
    ground_size = 2.0
    ground_height = 0.2
    ground = chrono.ChBodyEasyBox(
        ground_size, ground_height, ground_size,
        1000,  
        True,  
        False  
    )
    ground.SetPos(chrono.ChVectorD(0, -ground_height/2 - 0.5, 0))
    ground.SetBodyFixed(True)
    ground.SetName("Ground")
    my_system.AddBody(ground)
    
    
    anchor_post = chrono.ChBodyEasyCylinder(
        0.05,  
        1.0,   
        1000,  
        True
    )
    anchor_post.SetPos(chrono.ChVectorD(0, -0.5, 0))
    anchor_post.SetBodyFixed(True)
    anchor_post.SetName("Anchor Post")
    my_system.AddBody(anchor_post)
    
    
    
    
    
    
    num_elements = 15          
    beam_length = 5.0          
    element_length = beam_length / num_elements
    
    
    E = 2.1e11      
    nu = 0.3        
    rho = 7850.0    
    
    
    beam_radius = 0.02         
    area = math.pi * beam_radius**2
    Izz = math.pi * beam_radius**4 / 4  
    J = 2 * Izz                         
    
    
    beam_nodes = []
    beam_elements = []
    
    
    print("Creating ANCF cable beam nodes...")
    for i in range(num_elements + 1):
        
        x_pos = i * element_length
        pos = chrono.ChVectorD(x_pos, 0.5, 0)
        
        
        node = chrono.ChNodeCableANCF(pos)
        
        
        if i == 0:
            node.SetFixed(True)  
            print(f"  Node {i}: Fixed at {pos.x}, {pos.y}, {pos.z}")
        else:
            print(f"  Node {i}: Free at {pos.x}, {pos.y}, {pos.z}")
        
        
        node.SetName(f"BeamNode_{i}")
        my_system.Add(node)
        beam_nodes.append(node)
    
    
    print("\nCreating ANCF cable elements...")
    for i in range(num_elements):
        
        nodeA = beam_nodes[i]
        nodeB = beam_nodes[i + 1]
        
        
        element = chrono.ChElementCableANCF()
        
        
        element.SetNodes(nodeA, nodeB)
        
        
        element.SetData(E, nu, rho, area, Izz, J)
        
        
        element.SetName(f"BeamElement_{i}")
        
        
        my_system.Add(element)
        beam_elements.append(element)
        
        print(f"  Element {i}: Nodes {i} to {i+1}")
    
    print(f"\nBeam created with {num_elements} elements and {num_elements + 1} nodes")
    
    
    
    
    
    
    
    
    
    
    perturbation = 0.001
    for i in range(1, len(beam_nodes)):
        node = beam_nodes[i]
        current_pos = node.GetPos()
        
        new_pos = chrono.ChVectorD(
            current_pos.x,
            current_pos.y + perturbation * math.sin(i * 0.5),
            current_pos.z + perturbation * math.cos(i * 0.3)
        )
        node.SetPos(new_pos)
    
    
    
    
    
    print("\nInitializing Irrlicht visualization...")
    
    
    myapplication = irr.ChIrrApp(
        my_system,           
        "ANCF Cable Beam",   
        irr.dimension2du(1280, 720),  
        irr.VerticalLayout_ID
    )
    
    
    myapplication.AddTypicalLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    myapplication.AddTypicalSky()
    myapplication.AddTypicalCamera(irr.vector3df(3.0, 2.0, 4.0))
    myapplication.AddTypicalLight(irr.vector3df(0.5, 0.8, 0.6), 0.7)
    
    
    myapplication.AssetBinding(my_system)
    
    
    myapplication.AddAllAssets()
    
    
    
    for i, node in enumerate(beam_nodes):
        if i == 0:
            
            ball_mat = chrono.ChVisualMaterial()
            ball_mat.SetDiffuseColor(chrono.ChColor(0.0, 1.0, 0.0))  
            ball_mat.SetSpecularColor(chrono.ChColor(0.3, 0.3, 0.3))
        else:
            
            ball_mat = chrono.ChVisualMaterial()
            ball_mat.SetDiffuseColor(chrono.ChColor(0.0, 0.5, 1.0))  
            ball_mat.SetSpecularColor(chrono.ChColor(0.3, 0.3, 0.3))
        
        
        node_sphere = chrono.ChSphereShape(0.03)
        node_sphere.AddMaterial(ball_mat)
        node.GetAssets().push_back(node_sphere)
    
    
    
    
    
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
    
    
    simulation_time = 0.0
    output_interval = 0.5  
    last_output_time = 0.0
    
    
    myapplication.SetTimestep(0.002)  
    
    while myapplication.GetDevice().run():
        
        myapplication.BeginScene()
        
        
        myapplication.DrawAll()
        
        
        myapplication.Render()
        
        
        device = myapplication.GetDevice()
       guienv = device.getGUIEnvironment()
        
        
        if guienv:
            
            guienv.clear()
            
            
            guienv.addStaticText(
                "ANCF Cable Beam Simulation",
                irr.rect<s32>(10, 10, 300, 30),
                False
            )
            
            
            sim_text = f"Time: {simulation_time:.2f} s"
            guienv.addStaticText(
                sim_text,
                irr.rect<s32>(10, 40, 250, 60),
                False
            )
            
            
            if len(beam_nodes) > 1:
                last_node = beam_nodes[-1]
                pos = last_node.GetPos()
                end_text = f"End position: ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f})"
                guienv.addStaticText(
                    end_text,
                    irr.rect<s32>(10, 70, 400, 100),
                    False
                )
        
        
        myapplication.DoStep()
        
        
        simulation_time = my_system.GetChTime()
        
        
        if simulation_time - last_output_time >= output_interval:
            
            print(f"\n[{simulation_time:.2f}s] Beam Status:")
            print(f"  End node position: ({beam_nodes[-1].GetPos().x:.4f}, "
                  f"{beam_nodes[-1].GetPos().y:.4f}, "
                  f"{beam_nodes[-1].GetPos().z:.4f})")
            print(f"  End node velocity: ({beam_nodes[-1].GetPosDt().x:.4f}, "
                  f"{beam_nodes[-1].GetPosDt().y:.4f}, "
                  f"{beam_nodes[-1].GetPosDt().z:.4f})")
            
            
            print("  Node positions:")
            for i, node in enumerate(beam_nodes):
                p = node.GetPos()
                print(f"    Node {i}: ({p.x:.4f}, {p.y:.4f}, {p.z:.4f})")
            
            last_output_time = simulation_time
        
        
        myapplication.EndScene()
    
    
    
    
    
    print("\n" + "=" * 60)
    print("SIMULATION COMPLETED")
    print("=" * 60)
    print("\nFinal Node Positions:")
    for i, node in enumerate(beam_nodes):
        p = node.GetPos()
        v = node.GetPosDt()
        print(f"  Node {i:2d}: Pos=({p.x:.4f}, {p.y:.4f}, {p.z:.4f}) "
              f"Vel=({v.x:.4f}, {v.y:.4f}, {v.z:.4f})")
    
    
    initial_end_y = beam_length + 0.5
    final_end_y = beam_nodes[-1].GetPos().y
    max_deflection = initial_end_y - final_end_y
    
    print(f"\nMaximum Beam Deflection: {max_deflection:.4f} m")
    print(f"Initial end height: {initial_end_y:.4f} m")
    print(f"Final end height: {final_end_y:.4f} m")
    
    return 0


if __name__ == "__main__":
    
    exit_code = main()
    exit(exit_code)