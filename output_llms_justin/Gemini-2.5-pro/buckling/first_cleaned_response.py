import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as irr
import pychrono.mkl as mkl 
import math
import os






class MyCompressionFunction(chrono.ChFunction):
    def __init__(self, max_displacement, ramp_duration):
        super().__init__()
        self.max_displacement = max_displacement
        self.ramp_duration = ramp_duration
        self.speed = 0
        if self.ramp_duration > 1e-9: 
            self.speed = self.max_displacement / self.ramp_duration

    def Get_y(self, x): 
        if x <= 0:
            return 0
        elif x < self.ramp_duration:
            return self.speed * x
        else:
            return self.max_displacement

    def Get_y_dx(self, x): 
        if x <= 0 or x >= self.ramp_duration:
            return 0
        else:
            return self.speed

    def Get_y_dxdx(self, x): 
        return 0




def main():
    print("Beam Buckling Simulation with PyChrono FEA")

    
    
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0)) 

    
    beam_length = 1.0  
    beam_width = 0.02 
    beam_thickness = 0.01 
    num_elements = 20
    num_nodes = num_elements + 1

    
    density = 7850  
    E_modulus = 210e9  
    poisson_ratio = 0.3
    
    area = beam_width * beam_thickness
    Iyy = (beam_width * beam_thickness**3) / 12.0 
    Izz = (beam_thickness * beam_width**3) / 12.0 

    
    mesh = fea.ChMesh()
    system.Add(mesh)

    
    nodes = []
    for i in range(num_nodes):
        x_pos = (i / num_elements) * beam_length
        node = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD(x_pos, 0, 0)))
        mesh.AddNode(node)
        nodes.append(node)

    
    
    if num_nodes > 2:
        mid_node_idx = num_nodes // 2
        imperfection_mag = beam_thickness * 0.05 
        nodes[mid_node_idx].SetPos(nodes[mid_node_idx].GetPos() + chrono.ChVectorD(0, imperfection_mag, 0))


    
    material = fea.ChMaterialBeamEuler()
    material.Set_density(density)
    material.Set_E(E_modulus)
    material.Set_G(E_modulus / (2 * (1 + poisson_ratio))) 
    material.Set_Iy(Iyy) 
    material.Set_Iz(Izz) 
    material.Set_A(area)

    for i in range(num_elements):
        element = fea.ChElementBeamEuler()
        element.SetNodes(nodes[i], nodes[i+1])
        element.SetSectionMaterial(material)
        mesh.AddElement(element)

    
    
    nodes[0].SetFixed(True)

    
    
    
    ground_body = system.GetGroundBody() 

    
    
    
    
    

    
    
    motor_ground_frame = chrono.ChFrameD(nodes[-1].GetPos())

    
    max_compression_displacement = -beam_length * 0.15 
    compression_duration = 2.0 
    compression_func = MyCompressionFunction(max_compression_displacement, compression_duration)

    
    
    
    axial_motor = chrono.ChLinkMotorLinearPosition()
    
    
    
    motor_csys = chrono.ChCoordsysD(nodes[-1].GetPos(), chrono.Q_from_AngY(chrono.CH_C_PI_2)) 

    
    
    
    
    
    axial_motor.Initialize(
        nodes[-1],          
        ground_body,        
        chrono.ChFrameD(nodes[-1].GetPos(), chrono.Q_from_AngY(chrono.CH_C_PI_2)) 
                                                                        
    )
    axial_motor.SetMotionFunction(compression_func)
    system.Add(axial_motor)

    
    vis_mesh = fea.ChVisualShapeFEA(mesh)
    vis_mesh.SetFEMdataType(fea.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ) 
    vis_mesh.SetColorscaleMinMax(-0.1 * E_modulus * Izz / beam_length, 0.1 * E_modulus * Izz / beam_length) 
    vis_mesh.SetSmoothFaces(True)
    vis_mesh.SetWireframe(False) 
    vis_mesh.SetBeamResolution(10)
    vis_mesh.SetBeamSectionType(fea.ChVisualShapeFEA.SectionType_RECTANGULAR) 
    vis_mesh.SetScaleBeam(beam_width, beam_thickness) 
    mesh.AddVisualShapeFEA(vis_mesh)

    vis_nodes = fea.ChVisualShapeFEA(mesh)
    vis_nodes.SetFEMdataType(fea.ChVisualShapeFEA.DataType_NONE)
    vis_nodes.SetFEMglyphType(fea.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
    vis_nodes.SetSymbolsScale(0.01)
    vis_nodes.SetDefaultSymbolsColor(chrono.ChColor(0.1,0.1,0.8))
    mesh.AddVisualShapeFEA(vis_nodes)


    
    irrlicht_app = irr.ChIrrApp(system, "FEA Beam Buckling", irr.dimension2du(1024, 768))
    irrlicht_app.AddTypicalLights()
    irrlicht_app.AddTypicalSky()
    irrlicht_app.AddTypicalLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    irrlicht_app.AddTypicalCamera(irr.vector3df(beam_length * 0.5, beam_length * 0.3, -beam_length * 0.7), 
                                  irr.vector3df(beam_length * 0.5, 0, 0))      
    irrlicht_app.AssetBindAll()
    irrlicht_app.AssetUpdateAll()


    
    
    try:
        mkl_solver = mkl.ChSolverPardisoMKL()
        mkl_solver.LockSparsityPattern(True) 
        system.SetSolver(mkl_solver)
        print("Using MKL Pardiso Solver.")
    except Exception as e:
        print(f"MKL solver not available or failed to initialize: {e}")
        print("Using MINRES solver as fallback.")
        solver = chrono.ChSolverMINRES()
        system.SetSolver(solver)
        solver.SetMaxIterations(200)
        solver.SetTolerance(1e-10)
        solver.EnableWarmStart(True)
        solver.SetVerbose(False)


    
    system.SetTimestepperType(chrono.ChTimestepper.Type_HHT)
    timestepper = system.GetTimestepper().CastToChTimestepperHHT()
    timestepper.SetAlpha(-0.2)  
    timestepper.SetMaxiters(10)
    timestepper.SetAbsTolerances(1e-5, 1e-4) 
    timestepper.SetMode(chrono.ChTimestepperHHT.ACCELERATION)
    timestepper.SetStepControl(False) 
    timestepper.SetVerbose(False)

    
    timestep = 0.002 
    simulation_end_time = compression_duration + 1.0 

    irrlicht_app.SetTimestep(timestep)
    irrlicht_app.SetTryRealtime(False) 

    max_y_displacement = 0

    while irrlicht_app.GetDevice().run():
        current_time = system.GetChTime()
        if current_time > simulation_end_time:
            break

        irrlicht_app.BeginScene(True, True, chrono.ChColor(0.2, 0.25, 0.3))
        irrlicht_app.DrawAll()

        
        if system.GetNumcontacts() > 0 : 
            print(f"Time: {current_time:.4f} s, Contacts: {system.GetNumcontacts()}")

        
        mid_node_y_disp = nodes[mid_node_idx].GetPos().y
        if abs(mid_node_y_disp) > max_y_displacement:
            max_y_displacement = abs(mid_node_y_disp)

        
        font = irrlicht_app.GetIGUIEnvironment().getBuiltInFont()
        text_pos = irr.position2di(10,10)
        text_color = irr.SColor(255, 255,255,0) 
        font.draw(f"Time: {current_time:.3f} s", text_pos, text_color)
        text_pos.Y += 20
        font.draw(f"Max Y-Disp (mid): {max_y_displacement:.4f} m", text_pos, text_color)
        text_pos.Y += 20
        applied_disp = compression_func.Get_y(current_time)
        font.draw(f"Applied X-Disp: {applied_disp:.4f} m", text_pos, text_color)

        
        irrlicht_app.DoStep() 

        irrlicht_app.EndScene()

    print(f"Simulation finished. Max mid-node Y displacement: {max_y_displacement:.4f} m")
    irrlicht_app.GetDevice().closeDevice()


if __name__ == "__main__":
    
    chrono.SetChronoDataPath(os.environ.get('CHRONO_DATA_DIR', '../../../chrono_data/'))
    main()