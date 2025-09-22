import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as irr
import math
import os




def create_link_body(system, node_pos):
    body = chrono.ChBody()
    body.SetPos(node_pos)
    body.SetFixed(True) 
    system.Add(body)
    return body




def main():
    
    system = chrono.ChSystemNSC()
    system.SetGrav(chrono.ChVector3d(0, -9.81, 0)) 

    
    mesh = fea.ChMesh()
    system.Add(mesh)

    
    beam_L = 1.0  
    beam_radius = 0.02
    beam_young_modulus = 2.1e11  
    beam_poisson_ratio = 0.3
    beam_density = 7850  
    
    num_elements_beam = 1 
    num_control_points = 5 
                           
    beam_degree = 3 

    
    flywheel_mass = 5.0
    flywheel_radius = 0.1
    flywheel_thickness = 0.05

    
    motor_speed_rad_s = 2.0 * math.pi 

    
    
    mat_beam = fea.ChMaterialBeamIGA(beam_young_modulus, 
                                     beam_young_modulus / (2.0 * (1.0 + beam_poisson_ratio)), 
                                     beam_density)
    
    
    
    
    section_euler = fea.ChBeamSectionEuler()
    section_euler.SetYoungModulus(beam_young_modulus)
    section_euler.SetGshearModulus(beam_young_modulus / (2.0 * (1.0 + beam_poisson_ratio)))
    section_euler.SetBeamRaleyghFactor(0.0) 
    section_euler.SetDensity(beam_density)
    section_euler.SetAsCircularSection(beam_radius)

    
    section_iga = fea.ChBeamSectionIGA()
    section_iga.SetBeamMaterial(mat_beam) 
    section_iga.SetSectionMyProperties(section_euler.GetInertiaJ(), 
                                       section_euler.GetArea() * (section_euler.GetYoungModulus() / (2.0 * (1.0 + section_euler.GetPoissonRatio()))), 
                                       section_euler.GetArea() * (section_euler.GetYoungModulus() / (2.0 * (1.0 + section_euler.GetPoissonRatio())))) 
    section_iga.SetArtificialJyyJzzFactor(0.0001) 
    section_iga.SetCircular(True)
    section_iga.SetRadius(beam_radius) 

    
    beam_nodes = []
    for i in range(num_control_points):
        x_pos = (beam_L / (num_control_points - 1)) * i
        node = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVector3d(x_pos, 0, 0)))
        mesh.AddNode(node)
        beam_nodes.append(node)

    
    
    
    num_knots = num_control_points + beam_degree + 1
    knots = chrono.vector_double()
    
    
    for _ in range(beam_degree + 1):
        knots.push_back(0.0)
    
    
    
    
    
    
    
    
    num_intervals = num_control_points - beam_degree
    if num_intervals > 1 : 
        for i in range(1, num_intervals):
            knots.push_back(float(i) / (num_intervals -1) ) 
                                                            

    
    
    
    
    
    
    
    
    knots.clear()
    for _ in range(beam_degree + 1): 
        knots.push_back(0.0)
    
    
    
    
    num_distinct_internal_knots = num_control_points - 1 - beam_degree
    for i in range(num_distinct_internal_knots):
        knots.push_back( (i+1) / (num_distinct_internal_knots + 1.0) )
        
    for _ in range(beam_degree + 1): 
        knots.push_back(1.0)

    
    beam_element = fea.ChElementBeamIGA()
    beam_element.SetNodes(chrono.vector_ChNodeFEAxyzrot(beam_nodes), knots) 
    beam_element.SetSection(section_iga)
    beam_element.SetPolynomialDegree(beam_degree)
    mesh.AddElement(beam_element)

    
    ground_body = chrono.ChBody()
    ground_body.SetFixed(True)
    ground_body.SetName("ground")
    system.Add(ground_body)
    
    
    
    center_idx = num_control_points // 2 
    flywheel_pos = beam_nodes[center_idx].GetPos()

    flywheel = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 
                                         flywheel_radius, flywheel_thickness,
                                         beam_density, 
                                         True, True) 
    flywheel.SetPos(flywheel_pos)
    flywheel.SetMass(flywheel_mass)
    
    
    
    
    
    I_axial = 0.5 * flywheel_mass * flywheel_radius * flywheel_radius
    I_perp  = 0.25 * flywheel_mass * flywheel_radius * flywheel_radius + (1/12) * flywheel_mass * flywheel_thickness * flywheel_thickness
    flywheel.SetInertiaXX(chrono.ChVector3d(I_axial, I_perp, I_perp)) 
    flywheel.SetName("flywheel")
    system.Add(flywheel)

    
    
    
    
    
    
    motor_housing_body = create_link_body(system, beam_nodes[0].GetPos())
    motor_housing_body.SetFixed(True) 

    
    
    motor_driven_body = chrono.ChBody()
    motor_driven_body.SetPos(beam_nodes[0].GetPos())
    motor_driven_body.SetMass(1e-6) 
    motor_driven_body.SetInertiaXX(chrono.ChVector3d(1e-6,1e-6,1e-6))
    system.Add(motor_driven_body)

    link_node_to_motor_body = fea.ChLinkPointFrame()
    link_node_to_motor_body.Initialize(beam_nodes[0], motor_driven_body) 
    mesh.Add(link_node_to_motor_body)

    
    
    motor_frame = chrono.ChFrameD(beam_nodes[0].GetPos(), chrono.Q_from_AngY(math.pi/2)) 
    motor = chrono.ChLinkMotorRotation()
    motor.Initialize(motor_driven_body,          
                     ground_body,       
                     motor_frame)       
    
    motor_func = chrono.ChFunction_Ramp(0, motor_speed_rad_s) 
    motor.SetMotorFunction(motor_func)
    system.Add(motor)

    
    link_flywheel = fea.ChLinkPointFrame() 
    link_flywheel.Initialize(beam_nodes[center_idx], flywheel)
    mesh.Add(link_flywheel)
    
    
    
    
    
    bearing_housing_body = create_link_body(system, beam_nodes[-1].GetPos())
    bearing_housing_body.SetFixed(True)

    
    end_node_body = chrono.ChBody()
    end_node_body.SetPos(beam_nodes[-1].GetPos())
    end_node_body.SetMass(1e-6)
    end_node_body.SetInertiaXX(chrono.ChVector3d(1e-6,1e-6,1e-6))
    system.Add(end_node_body)

    link_node_to_end_body = fea.ChLinkPointFrame()
    link_node_to_end_body.Initialize(beam_nodes[-1], end_node_body)
    mesh.Add(link_node_to_end_body)
    
    
    
    
    
    
    link_bearing = chrono.ChLinkMateSpherical()
    link_bearing.Initialize(end_node_body,  
                            bearing_housing_body, 
                            chrono.ChFrameD(beam_nodes[-1].GetPos())) 
    system.Add(link_bearing)
    
    
    
    
    
    vis_beam_shape = fea.ChVisualShapeFEA(mesh)
    vis_beam_shape.SetFEMdataType(fea.ChVisualShapeFEA.DataType.SURFACE)
    vis_beam_shape.SetFEMglyphType(fea.ChVisualShapeFEA.GlyphType.NONE) 
    vis_beam_shape.SetSymbolsThickness(0.005)
    vis_beam_shape.SetDefaultGLE اساس(fea.ChVisualShapeFEA.GLE اساس.SOLID)
    vis_beam_shape.SetZbufferHide(False)
    mesh.AddVisualShapeFEA(vis_beam_shape)

    
    vis_beam_axes = fea.ChVisualShapeFEA(mesh)
    vis_beam_axes.SetFEMdataType(fea.ChVisualShapeFEA.DataType.NONE) 
    vis_beam_axes.SetFEMglyphType(fea.ChVisualShapeFEA.GlyphType.AXES) 
    vis_beam_axes.SetSymbolsThickness(0.01)
    vis_beam_axes.SetDefaultGLE اساس(fea.ChVisualShapeFEA.GLE اساس. चंडीगढ़) 
    mesh.AddVisualShapeFEA(vis_beam_axes)
    
    
    vis_node_csys = fea.ChVisualShapeFEA(mesh)
    vis_node_csys.SetFEMdataType(fea.ChVisualShapeFEA.DataType.NONE)
    vis_node_csys.SetFEMglyphType(fea.ChVisualShapeFEA.GlyphType.NODE_CSYS)
    vis_node_csys.SetSymbolsThickness(0.015) 
    mesh.AddVisualShapeFEA(vis_node_csys)

    
    vis = irr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 768)
    vis.SetWindowTitle('IGA Jeffcott Rotor Simulation')
    vis.Initialize()
    vis.AddLogo(os.path.join(chrono.GetChronoDataPath(), 'logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(beam_L * 0.5, beam_L * 0.5, -beam_L * 1.5)) 
    vis.AddTypicalLights()
    vis.EnableCollisionShapeDrawing(True) 

    
    
    
    
    
    
    
    
    system.SetSolverType(chrono.ChSolver.Type.SPARSE_LU)
    
    system.SetTimestepperType(chrono.ChTimestepper.Type.EULER_IMPLICIT_LINEARIZED) 
    
    
    
    
    
    

    
    time_step = 0.005
    end_time = 10.0

    
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
        
        
        
        
        

        vis.EndScene()
        system.DoStepDynamics(time_step)

        if system.GetChTime() > end_time:
            vis.GetDevice().closeDevice() 
            break
            
    print("Simulation finished.")

if __name__ == "__main__":
    main()