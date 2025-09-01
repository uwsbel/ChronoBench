import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as irrlicht


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, 0))  


mesh = fea.ChMesh()
system.Add(mesh)


length = 2.0    
n_elements = 20 
beam_width = 0.1
rho = 7800      
E = 2e11        
I = (beam_width**4)/12  


beam_section = fea.ChBeamSectionEuler()
beam_section.SetDensity(rho)
beam_section.SetYoungModulus(E)
beam_section.SetAsRectangularSection(beam_width, beam_width)


nodes = []
for i in range(n_elements + 1):
    position = chrono.ChVectorD(i * (length / n_elements), 0, 0)
    node = fea.ChNodeFEAxyzrot(chrono.ChFrameD(position))
    node.SetMass(0)
    mesh.AddNode(node)
    nodes.append(node)

for i in range(n_elements):
    element = fea.ChElementBeamEuler()
    element.SetNodes(nodes[i], nodes[i + 1])
    element.SetSection(beam_section)
    mesh.AddElement(element)



truss = chrono.ChBody()
truss.SetBodyFixed(True)
system.Add(truss)

fix = fea.ChLinkPointFrame()
fix.Initialize(nodes[0], truss)
system.Add(fix)


slider = chrono.ChBodyEasyBox(0.1, 0.1, 0.1, 1000)
slider.SetPos(chrono.ChVectorD(length, 0, 0))
system.Add(slider)

motor_link = chrono.ChLinkMotorLinearPosition()
motor_link.Initialize(slider, truss, 
                     chrono.ChFrameD(chrono.ChVectorD(length, 0, 0), 
                                    chrono.Q_ROTATE_Y_TO_X))
system.Add(motor_link)


motor_func = chrono.ChFunction_Ramp(0, 0.01)  
motor_link.SetMotorFunction(motor_func)


node_slider_link = fea.ChLinkPointFrame()
node_slider_link.Initialize(nodes[-1], slider)
system.Add(node_slider_link)


vis = irrlicht.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Beam Buckling Simulation')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(1.5, 0.5, 2), chrono.ChVectorD(1.5, 0, 0))
vis.AddTypicalLights()


beam_visual = fea.ChVisualShapeFEA(mesh)
beam_visual.SetFEMdataType(fea.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
beam_visual.SetColorscaleMinMax(-1000, 1000)
beam_visual.SetSmoothFaces(True)
mesh.AddVisualShapeFEA(beam_visual)


system.SetSolverType(chrono.ChSolver.Type_MINRES)
system.SetSolverMaxIterations(200)
system.SetSolverTolerance(1e-12)
system.SetTimestepperType(chrono.ChTimestepper.Type_HHT)
hht_stepper = system.GetTimestepper().AsTimestepperHHT()
hht_stepper.SetAlpha(-0.2)
hht_stepper.SetStepControl(True)
hht_stepper.SetVerbose(False)


dt = 0.001
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(dt)