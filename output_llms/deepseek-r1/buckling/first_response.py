#!/usr/bin/env python3
import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as irrlicht

def main():
    # 1. Initialize PyChrono environment
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, 0, 0))  # Optional: remove gravity
    
    # 2. Create FEA beam mesh
    mesh = fea.ChMesh()
    material = fea.ChMaterialBeamEuler(
        2.1e11,  # Young's modulus (steel)
        0.3,     # Poisson ratio
        7800,    # Density
        1e8,     # Damping coefficient
        0.05     # Rayleigh damping
    )
    
    # Beam geometry parameters
    length = 5.0
    num_elements = 20
    node_spacing = length / num_elements
    beam_w = 0.1  # Cross-section width
    
    # Create nodes and elements
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
    
    # Fix first node (cantilever)
    constraint = fea.ChLinkPointFrame()
    constraint.Initialize(nodes[0], nodes[0].Frame().GetPos())
    mesh.AddLink(constraint)
    
    # 3. Create motor function for axial displacement
    motor_function = chrono.ChFunction_Ramp(0, 0.02)  # Displacement rate
    
    motor_load = fea.ChLoadMotorLinearPosition()
    motor_load.SetMotorFunction(motor_function)
    motor_load.SetLink(nodes[-1])
    mesh.AddLoad(motor_load)
    
    # 4. Add visualization and assets
    visbeam = fea.ChVisualizationFEAmesh(mesh)
    visbeam.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_ELEM_BEAM_MZ)
    mesh.AddAsset(visbeam)
    
    system.Add(mesh)
    
    # 5. Set up solver and timestepper
    solver = chrono.ChSolverMINRES()
    system.SetSolver(solver)
    solver.SetMaxIterations(100)
    solver.SetTolerance(1e-12)
    solver.EnableDiagonalPreconditioner(True)
    
    system.SetTimestepperType(chrono.ChTimestepper.Type_HHT)
    integrator = system.GetTimestepper().As_ChTimestepperHHT()
    integrator.SetAlpha(-0.2)
    integrator.SetStepControl(True)
    
    # 6. Irrlicht visualization setup
    vis = irrlicht.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle('Beam Buckling FEA')
    vis.Initialize()
    vis.AddLogo()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVectorD(3, 0.5, 3))
    vis.AddTypicalLights()
    
    # 7. Simulation loop
    time_step = 0.001
    while vis.Run():
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(time_step)
    
if __name__ == '__main__':
    main()