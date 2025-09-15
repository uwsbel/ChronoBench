import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as irrlicht

def main():
    
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, 0, 0))  
    
    
    mesh = fea.ChMesh()
    material = fea.ChMaterialBeamEuler(
        2.1e11,  
        0.3,     
        7800,    
        1e8,     
        0.05     
    )
    
    
    length = 5.0
    num_elements = 20
    node_spacing = length / num_elements
    beam_w = 0.1  
    
    
    nodes = []
    for i in range(num_elements + 1):
        pos = chrono.ChVectorD(node_spacing * i, 0, 0)
        node = fea.ChNodeFEAxyzrot(chrono.ChFrameD(pos))
        nodes.append(node)
        mesh.AddNode(node)
        
        if i > 0:
            element = fea.ChElementBeamEuler()
            element.SetNodes(nodes[i-1], nodes[i])
            element.SetSection(fea.ChBeamSectionEulerSimple(material, beam_w**2/12, beam_w**2))
            mesh.AddElement(element)
    
    
    constraint = fea.ChLinkPointFrame()
    constraint.Initialize(nodes[0], nodes[0].Frame().GetPos())
    mesh.AddLink(constraint)
    
    
    motor_function = chrono.ChFunction_Ramp(0, 0.02)  
    
    motor_load = fea.ChLoadMotorLinearPosition()
    motor_load.SetMotorFunction(motor_function)
    motor_load.SetLink(nodes[-1])
    mesh.AddLoad(motor_load)
    
    
    visbeam = fea.ChVisualizationFEAmesh(mesh)
    visbeam.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_ELEM_BEAM_MZ)
    mesh.AddAsset(visbeam)
    
    system.Add(mesh)
    
    
    solver = chrono.ChSolverMINRES()
    system.SetSolver(solver)
    solver.SetMaxIterations(100)
    solver.SetTolerance(1e-12)
    solver.EnableDiagonalPreconditioner(True)
    
    system.SetTimestepperType(chrono.ChTimestepper.Type_HHT)
    integrator = system.GetTimestepper().As_ChTimestepperHHT()
    integrator.SetAlpha(-0.2)
    integrator.SetStepControl(True)
    
    
    vis = irrlicht.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle('Beam Buckling FEA')
    vis.Initialize()
    vis.AddLogo()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVectorD(3, 0.5, 3))
    vis.AddTypicalLights()
    
    
    time_step = 0.001
    while vis.Run():
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(time_step)
    
if __name__ == '__main__':
    main()