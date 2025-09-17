import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea

def main():
    
    system = chrono.ChSystemSMC()
    system.Set_G_acc(chrono.ChVectorD(0, 0, 0))  
    
    
    ground = chrono.ChBody()
    ground.SetBodyFixed(True)
    system.Add(ground)
    
    slider = chrono.ChBody()
    slider.SetPos(chrono.ChVectorD(2.0, 0, 0))
    slider.SetMass(0.1)
    slider.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))
    system.Add(slider)
    
    
    prismatic = chrono.ChLinkLockPrismatic()
    prismatic.Initialize(ground, slider, 
                         chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), 
                         chrono.Q_from_AngY(chrono.CH_PI_2)))
    system.Add(prismatic)
    
    
    class RampDisplacement(chrono.ChFunction):
        def __init__(self):
            super().__init__()
            self.speed = 0.01  
        def Get_y(self, t):
            return -self.speed * t  
    
    motor = chrono.ChLinkMotorLinearPosition()
    motor.Initialize(ground, slider, 
                    chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
    motor.SetMotionFunction(RampDisplacement())
    system.Add(motor)
    
    
    mesh = fea.ChMesh()
    system.Add(mesh)
    
    
    beam_section = fea.ChBeamSectionEulerAdvanced()
    rho = 7800    
    E = 2.1e11    
    G = 0.8e11    
    radius = 0.05 
    area = chrono.CH_PI * radius**2
    I = chrono.CH_PI * radius**4 / 4
    J = 2 * I     
    
    beam_section.SetDensity(rho)
    beam_section.SetYoungModulus(E)
    beam_section.SetGshearModulus(G)
    beam_section.SetArea(area)
    beam_section.SetItorsion(J)
    beam_section.SetIyy(I)
    beam_section.SetIzz(I)
    
    
    num_elements = 20
    length = 2.0  
    nodes = []
    
    
    for i in range(num_elements + 1):
        pos = chrono.ChVectorD(i * length / num_elements, 0, 0)
        if i == num_elements // 2:  
            pos.y = 0.001  
        node = fea.ChNodeFEAxyzrot(chrono.ChFrameD(pos))
        mesh.AddNode(node)
        nodes.append(node)
    
    
    for i in range(num_elements):
        element = fea.ChElementBeamEuler()
        element.SetNodes(nodes[i], nodes[i + 1])
        element.SetSection(beam_section)
        mesh.AddElement(element)
    
    
    
    constraint_ground = fea.ChLinkNodeFrame()
    constraint_ground.Initialize(nodes[0], ground)
    system.Add(constraint_ground)
    
    
    constraint_slider = fea.ChLinkNodeFrame()
    constraint_slider.Initialize(nodes[-1], slider)
    system.Add(constraint_slider)
    
    
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 768)
    vis.SetWindowTitle("Beam Buckling Simulation")
    vis.Initialize()
    vis.AddCamera(chrono.ChVectorD(1.0, -1.5, 1.0), chrono.ChVectorD(1.0, 0, 0))
    vis.AddTypicalLights()
    
    
    mvisual = fea.ChVisualShapeFEA(mesh)
    mvisual.SetFEMdataType(fea.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
    mvisual.SetColorscaleMinMax(-20, 20)
    mvisual.SetSmoothFaces(True)
    mvisual.SetWireframe(False)
    mesh.AddVisualShapeFEA(mvisual)
    
    
    solver = chrono.ChSolverMINRES()
    solver.SetMaxIterations(500)
    solver.SetTolerance(1e-12)
    solver.EnableDiagonalPreconditioner(True)
    system.SetSolver(solver)
    
    
    system.SetTimestepperType(chrono.ChTimestepper.Type_HHT)
    hht_stepper = system.GetTimestepper().AsTimestepperHHT()
    hht_stepper.SetAlpha(-0.2)
    hht_stepper.SetStepControl(True)
    hht_stepper.SetVerbose(False)
    
    
    step_size = 0.005
    while vis.Run():
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step_size)
    
if __name__ == "__main__":
    main()