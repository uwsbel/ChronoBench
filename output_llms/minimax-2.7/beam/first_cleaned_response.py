import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math
import numpy as np

def create_beam_simulation():
    
    
    
    
    
    
    
    system = chrono.ChSystemSMC()
    
    
    system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  
    system.SetSolverType(chrono.ChSolver.Type_SOR)    
    system.SetMaxItersSolverSpeed(100)               
    system.SetStepSize(0.001)                         
    
    
    system.SetUseMaterialProperties(False)
    
    print("PyChrono environment initialized successfully!")
    print(f"System gravity: {system.Get_G_acc()}")
    
    
    
    
    
    
    
    E = 210e9      
    nu = 0.3       
    rho = 7850.0   
    yield_stress = 250e6  
    
    
    beam_material = chrono.ChBeamSectionTimoshenkoRD()
    
    
    radius = 0.02  
    beam_material.SetCircular(radius, E, nu, rho, yield_stress)
    
    print(f"\nBeam Properties:")
    print(f"  - Radius: {radius} m")
    print(f"  - Young's Modulus: {E/1e9} GPa")
    print(f"  - Density: {rho} kg/m³")
    print(f"  - Poisson's Ratio: {nu}")
    
    
    
    
    
    
    num_elements = 10           
    beam_length = 2.0          
    element_length = beam_length / num_elements
    
    
    nodes = []
    beams = []
    
    
    print(f"\nCreating beam structure:")
    print(f"  - Number of elements: {num_elements}")
    print(f"  - Total length: {beam_length} m")
    print(f"  - Element length: {element_length} m")
    
    for i in range(num_elements + 1):
        
        x_pos = i * element_length
        y_pos = 0.0
        z_pos = 0.0
        
        
        node = chrono.ChNodeFEAxyzD())
        node.SetPos(chrono.ChVectorD(x_pos, y_pos, z_pos))
        node.SetDOF(chrono.ChVectorD(0, 0, 0))
        node.SetVel(chrono.ChVectorD(0, 0, 0))
        node.SetAngularVel(chrono.ChVectorD(0, 0, 0))
        
        
        system.Add(node)
        nodes.append(node)
        
        
        if i <= 2 or i >= num_elements - 1:
            print(f"  Created node {i}: pos=({x_pos:.3f}, {y_pos:.3f}, {z_pos:.3f})")
    
    
    print(f"\nCreating beam elements...")
    for i in range(num_elements):
        
        node_a = nodes[i]
        node_b = nodes[i + 1]
        
        
        beam = chrono.ChElementTimoshenko()
        beam.SetNodes(node_a, node_b)
        beam.SetSection(beam_material)
        
        
        
        
        
        system.Add(beam)
        beams.append(beam)
        
        if i <= 2 or i >= num_elements - 2:
            print(f"  Created beam element {i}: nodes {i} to {i+1}")
    
    
    
    
    
    print("\nApplying boundary conditions and loads:")
    
    
    fixed_node = nodes[0]
    fixed_node.SetFixed(True)
    print(f"  - Fixed node 0 (cantilever support)")
    
    
    load_magnitude = 100.0  
    load_force = chrono.ChVectorD(0, -load_magnitude, 0)
    nodes[-1].SetForce(load_force)
    print(f"  - Applied point load at node {len(nodes)-1}: {load_magnitude} N downward")
    
    
    distributed_load = rho * math.pi * radius**2 * 9.81  
    for beam in beams:
        beam.SetLoad(chrono.ChVectorD(0, -distributed_load, 0))
    print(f"  - Applied self-weight: {distributed_load:.4f} N/m")
    
    
    
    
    
    
    ground = chrono.ChBody()
    ground.SetBodyFixed(True)
    ground.SetPos(chrono.ChVectorD(0, -0.5, 0))
    
    
    ground_shape = chrono.ChBoxShape()
    ground_shape.SetLengths(chrono.ChVectorD(5, 0.1, 2))
    ground.AddAsset(ground_shape)
    
    
    ground_color = chrono.ChColorAsset()
    ground_color.SetColor(chrono.ChColor(0.4, 0.4, 0.4))
    ground.AddAsset(ground_color)
    
    system.Add(ground)
    
    
    for i, node in enumerate(nodes):
        
        node_sphere = chrono.ChSphereShape()
        node_sphere.GetSphereGeometry().rad = 0.015
        node.AddAsset(node_sphere)
        
        
        node_label = chrono.ChTextShape()
        node_label.SetText(f"N{i}")
        node_label.SetColor(chrono.ChColor(0.2, 0.8, 0.2))
        node.AddAsset(node_label)
    
    
    for i, beam in enumerate(beams):
        
        beam_cyl = chrono.ChCylinderShape()
        beam_cyl.GetCylinderGeometry().rad = radius
        beam.AddAsset(beam_cyl)
    
    print("\nVisualization assets created")
    
    
    
    
    
    print("\nInitializing Irrlicht visualization...")
    
    
    application = chronoirr.ChIrrApp(
        system,                           
        "Beam Finite Elements Demo",       
        chronoirr.dimension2du(1280, 720), 
        chronoirr.VerticalLayout_STACKED   
    )
    
    
    application.AddTypicalLogo()
    application.AddTypicalSky()
    application.AddTypicalCamera(
        chronoirr.vector3df(1.5, 1.0, 2.5),  
        chronoirr.vector3df(1.0, 0.0, 0.0)   
    )
    application.AddTypicalLights()
    
    
    application.DrawAll
    
    
    
    
    
    print("\n" + "="*60)
    print("STARTING SIMULATION")
    print("="*60)
    
    
    simulation_time = 5.0  
    frame_rate = 30       
    
    
    output_interval = 1.0 / frame_rate
    
    
    current_time = 0.0
    last_output_time = 0.0
    
    
    while(application.GetDevice().run()):
        
        if current_time >= simulation_time:
            print(f"\nSimulation completed at t = {current_time:.2f} s")
            break
        
        
        application.BeginScene()
        application.DrawAll()
        
        
        system.DoStepDynamics(application.GetStep())
        
        
        current_time = system.GetChTime()
        
        
        if current_time - last_output_time >= output_interval:
            last_output_time = current_time
            
            
            if len(nodes) > 0:
                displacement = nodes[-1].GetPos()
                velocity = nodes[-1].GetVel()
                
                
                if int(current_time) != int(current_time - application.GetStep()):
                    print(f"t = {current_time:.2f} s | "
                          f"End displacement: ({displacement.x:.4f}, "
                          f"{displacement.y:.4f}, {displacement.z:.4f}) m | "
                          f"Velocity: ({velocity.x:.4f}, {velocity.y:.4f}, "
                          f"{velocity.z:.4f}) m/s")
        
        application.EndScene()
    
    
    
    
    
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
        
        
        total_disp = math.sqrt(displacement.x**2 + displacement.y**2 + displacement.z**2)
        print(f"\nMagnitude of total displacement: {total_disp:.6f} m")
        
        
        
        I = math.pi * radius**4 / 4  
        analytical_deflection = (load_magnitude * beam_length**3) / (3 * E * I)
        print(f"\nAnalytical cantilever deflection (end load only): "
              f"{analytical_deflection:.6f} m")
        
        
        if analytical_deflection > 0:
            error_percent = abs(total_disp - analytical_deflection) / analytical_deflection * 100
            print(f"Error: {error_percent:.2f}%")
    
    
    print(f"\nBeam stress analysis:")
    for i, beam in enumerate(beams):
        
        
        section = beam.GetSection()
        if section:
            area = math.pi * radius**2
            axial_force = distributed_load * beam_length / 2  
            axial_stress = abs(axial_force) / area
            print(f"  Beam {i}: Approximate axial stress = {axial_stress/1e6:.2f} MPa")
    
    print("\n" + "="*60)
    print("Simulation finished successfully!")
    print("="*60)
    
    return system, nodes, beams

def create_multi_span_beam():
    
    print("\n" + "="*60)
    print("CREATING MULTI-SPAN BEAM SIMULATION")
    print("="*60)
    
    
    system = chrono.ChSystemSMC()
    system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
    system.SetSolverType(chrono.ChSolver.Type_SOR)
    system.SetStepSize(0.001)
    
    
    E = 200e9      
    nu = 0.3       
    rho = 7800.0   
    radius = 0.015  
    
    
    beam_material = chrono.ChBeamSectionTimoshenkoRD()
    beam_material.SetCircular(radius, E, nu, rho, 250e6)
    
    
    span_length = 1.5  
    num_elements_per_span = 8
    total_elements = num_elements_per_span * 2 + 1  
    element_length = span_length / num_elements_per_span
    
    
    nodes = []
    for i in range(total_elements + 1):
        x_pos = i * element_length
        y_pos = 0.0
        
        node = chrono.ChNodeFEAxyzD()
        node.SetPos(chrono.ChVectorD(x_pos, y_pos, 0))
        
        
        if i == 0 or i == total_elements:
            node.SetFixed(True)
        
        system.Add(node)
        nodes.append(node)
    
    
    beams = []
    for i in range(total_elements):
        beam = chrono.ChElementTimoshenko()
        beam.SetNodes(nodes[i], nodes[i + 1])
        beam.SetSection(beam_material)
        system.Add(beam)
        beams.append(beam)
    
    
    middle_support = chrono.ChNodeFEAxyzD()
    middle_support.SetPos(chrono.ChVectorD(span_length, -0.1, 0))
    middle_support.SetFixed(True)
    system.Add(middle_support)
    
    
    distributed_load = rho * math.pi * radius**2 * 9.81
    for beam in beams:
        beam.SetLoad(chrono.ChVectorD(0, -distributed_load * 10, 0))  
    
    
    load_point = 50.0  
    nodes[4].SetForce(chrono.ChVectorD(0, -load_point, 0))  
    nodes[11].SetForce(chrono.ChVectorD(0, -load_point, 0))  
    
    print(f"Created {len(nodes)} nodes and {len(beams)} beam elements")
    print(f"Two-span beam: {span_length}m + {span_length}m")
    print(f"Applied loads: Distributed self-weight + point loads")
    
    return system

def main():
    
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
        
        system, nodes, beams = create_beam_simulation()
        
        
        
        
        
        print("\n✓ Simulation completed successfully!")
        
    except Exception as e:
        print(f"\n✗ Error during simulation: {str(e)}")
        import traceback
        traceback.print_exc()
        return 1
    
    return 0

if __name__ == "__main__":
    exit(main())