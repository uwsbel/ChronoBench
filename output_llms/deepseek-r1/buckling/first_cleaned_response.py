import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea

def main():
    
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, 0, 0))  

    
    ground = chrono.ChBodyEasyBox(0.1, 0.1, 0.1, 1000)
    ground.SetBodyFixed(True)
    ground.SetPos(chrono.ChVectorD(0, 0, 0))
    system.Add(ground)

    
    mesh = fea.ChMesh()
    system.Add(mesh)

    
    density = 7800  
    E = 2.0e11      
    nu = 0.3         
    width = 0.02     
    height = 0.01    
    
    beam_section = fea.ChBeamSectionEulerAdvanced()
    beam_section.SetDensity(density)
    beam_section.SetYoungModulus(E)
    beam_section.SetShearModulus(E / (2 * (1 + nu)))
    beam_section.SetAsRectangularSection(width, height)
    beam_section.SetRayleighDampingBeta(0.001)  

    
    length = 1.0    
    num_elements = 10
    dx = length / num_elements
    nodes = []

    
    for i in range(num_elements + 1):
        node = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD(i * dx, 0, 0)))
        node.SetMass(0)
        mesh.AddNode(node)
        nodes.append(node)

    
    for i in range(num_elements):
        element = fea.ChElementBeamEuler()
        element.SetNodes(nodes[i], nodes[i + 1])
        element.SetSection(beam_section)
        mesh.AddElement(element)

    
    constraint_fixed = fea.ChLinkPointFrame()
    constraint_fixed.Initialize(nodes[0], ground)
    system.Add(constraint_fixed)

    
    slider = chrono.ChBodyEasyBox(0.1, 0.1, 0.1, 1000)
    slider.SetPos(nodes[-1].GetPos())
    system.Add(slider)

    
    prismatic = chrono.ChLinkLockPrismatic()
    prismatic.Initialize(ground, slider, 
                         chrono.ChCoordsysD(nodes[-1].GetPos(), chrono.Q_from_AngY(chrono.CH_C_PI_2)))
    system.Add(prismatic)

    
    constraint_slider = fea.ChLinkPointFrame()
    constraint_slider.Initialize(nodes[-1], slider)
    system.Add(constraint_slider)

    
    motor_func = chrono.ChFunction_Ramp(0, -0.01)  
    motor = chrono.ChLinkMotorLinearPosition()
    motor.Initialize(ground, slider, chrono.ChFrameD(nodes[-1].GetPos()))
    motor.SetMotionFunction(motor_func)
    system.Add(motor)

    
    system.SetSolverType(chrono.ChSolver.Type_MINRES)
    system.SetSolverMaxIterations(100)
    system.SetSolverTolerance(1e-10)
    system.SetTimestepperType(chrono.ChTimestepper.Type_HHT)
    system.GetSolver().AsIterative().SetVerbose(False)

    
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 768)
    vis.SetWindowTitle("Beam Buckling Simulation")
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVectorD(1.5, 0.5, 1.5), chrono.ChVectorD(0, 0, 0))
    vis.AddTypicalLights()

    
    beam_vis = fea.ChVisualShapeFEA(mesh)
    beam_vis.SetFEMdataType(fea.ChVisualShapeFEA.DataType_ELEM_BEAM_MX)
    beam_vis.SetColorscaleMinMax(-1000, 1000)  
    beam_vis.SetSmoothFaces(True)
    beam_vis.SetWireframe(False)
    mesh.AddVisualShapeFEA(beam_vis)

    
    time_step = 0.001
    time_end = 10.0  

    while vis.Run() and system.GetChTime() < time_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(time_step)

if __name__ == "__main__":
    main()