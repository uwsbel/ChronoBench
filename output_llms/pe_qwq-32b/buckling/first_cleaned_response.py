import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr


system = chrono.ChSystemSMC()
system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
system.SetTimestepperType(chrono.ChTimestepper.Type_HHT)
stepper = system.GetTimestepper()
stepper.SetAlpha(-0.2)  
stepper.SetMaxiters(50)
stepper.SetAbsTolerances(1e-5)


mesh = fea.ChMesh()
system.Add(mesh)


beam_length = 2.0
num_elements = 10
element_length = beam_length / num_elements

cross_section_width = 0.1
cross_section_height = 0.05

young_modulus = 2.1e11  
shear_modulus = young_modulus / (2 * (1 + 0.3))  


nodes = []
for i in range(num_elements + 1):
    x = i * element_length
    node = fea.ChNodeFEAxyzrot()
    node.SetPos(chrono.ChVectorD(x, 0, 0))
    mesh.AddNode(node)
    nodes.append(node)


nodes[0].Fix(True)


for i in range(num_elements):
    elem = fea.ChElementBeamEuler()
    elem.SetNodes(nodes[i], nodes[i+1])
    
    section = fea.ChBeamSectionEulerAdvanced()
    section.SetAsRectangularSection(cross_section_width, cross_section_height)
    section.SetYoungModulus(young_modulus)
    section.SetShearModulus(shear_modulus)
    section.SetRayleighDamping(0.001)  
    
    elem.SetSection(section)
    mesh.AddElement(elem)


class CompressionMotor(chrono.ChFunction):
    def Get_y(self, my_chrono, time):
        return -0.01 * time  


dummy_body = chrono.ChBody()
dummy_body.SetPos(nodes[-1].GetPos())
system.Add(dummy_body)

link_dummy_node = chrono.ChLinkLock()
link_dummy_node.Initialize(nodes[-1], dummy_body, chrono.ChFrameD())
system.Add(link_dummy_node)


fixed_body = chrono.ChBody()
fixed_body.SetBodyFixed(True)
fixed_body.SetPos(nodes[-1].GetPos())  
system.Add(fixed_body)


motor = chrono.ChLinkMotorLinear()
motor.Initialize(dummy_body, fixed_body, chrono.ChFrameD())
motor.SetCoordinateFunction(CompressionMotor())
system.Add(motor)


visualizor = chronoirr.ChVisualSystemIrrlicht()
visualizor.AttachSystem(system)
visualizor.SetWindowSize(1024, 768)
visualizor.SetWindowTitle('Beam Buckling Simulation')
visualizor.SetCamera(chronoirr.ChVectorD(beam_length/2, 1, beam_length))
visualizor.SetAntialiasing(True)
visualizor.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
visualizor.AddSkyBox()
visualizor.AddTypicalLights()


for elem in mesh.GetElements():
    vis_shape = elem.CreateVisualShape()
    mesh.AddVisualShape(vis_shape)

visualizor.Initialize()


time_step = 0.0005  
while visualizor.Run():
    system.DoStepDynamics(time_step)
    visualizor.BeginScene()
    visualizor.Render()
    visualizor.EndScene()