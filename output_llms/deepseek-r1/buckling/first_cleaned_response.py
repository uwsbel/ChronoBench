import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr


system = chrono.ChSystemSMC()
system.Set_G_acc(chrono.ChVector3d(0, 0, 0))  


system.SetSolverType(chrono.ChSolver.Type::MINRES)
system.SetSolverMaxIterations(100)
system.SetSolverTolerance(1e-10)
system.SetSolverWarmStarting(True)


mesh = fea.ChMesh()
system.Add(mesh)


length = 10.0  
width = 0.1    
density = 7800 
E = 2e11       
nu = 0.3       
n_elements = 20  


beam_material = fea.ChBeamSectionEulerAdvanced()
beam_material.SetDensity(density)
beam_material.SetYoungModulus(E)
beam_material.SetShearModulus(E/(2*(1+nu)))
beam_material.SetAsRectangularSection(width, width)


nodes = []
for i in range(n_elements + 1):
    node = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(0, i * length/n_elements, 0)))
    node.SetMass(0)
    mesh.AddNode(node)
    nodes.append(node)


for i in range(n_elements):
    element = fea.ChElementBeamEuler()
    element.SetNodes(nodes[i], nodes[i+1])
    element.SetSection(beam_material)
    mesh.AddElement(element)


constraint_fixed = fea.ChLinkNodeFrame()
constraint_fixed.Initialize(nodes[0], 
                           chrono.ChFrameD(nodes[0].GetPos()))
system.Add(constraint_fixed)


motor_body = chrono.ChBodyEasyBox(0.2, 0.2, 0.2, 1000)
motor_body.SetPos(chrono.ChVector3d(0, length, 0))
system.Add(motor_body)


constraint_top = fea.ChLinkNodeFrame()
constraint_top.Initialize(nodes[-1], motor_body)
system.Add(constraint_top)


motor = chrono.ChLinkMotorLinearPosition()
motor.Initialize(motor_body, 
                chrono.ChFrameD(chrono.ChVector3d(0, length, 0)),
                chrono.ChFrameD(chrono.ChVector3d(0, length, 0)))
system.Add(motor)


class LinearDisplacement(chrono.ChFunction):
    def __init__(self):
        super().__init__()
        self.speed = -0.02  
    def GetVal(self, x):
        return self.speed * x

motor.SetMotionFunction(LinearDisplacement())


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Beam Buckling Simulation")
vis.Initialize()
vis.AddCamera(chrono.ChVector3d(0.5, length/2, length), chrono.ChVector3d(0, length/2, 0))
vis.AddTypicalLights()


beam_visual = fea.ChVisualShapeFEA(mesh)
beam_visual.SetFEMdataType(fea.VisualDataType::ELEM_BEAM_MZ)
beam_visual.SetColorscaleMinMax(-500, 500)
beam_visual.SetSmoothFaces(True)
mesh.AddVisualShapeFEA(beam_visual)


time_step = 0.001
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(time_step)