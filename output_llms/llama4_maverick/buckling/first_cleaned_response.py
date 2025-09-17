import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr


chrono.SetChronoDataPath("/path/to/chrono/data/")  


sys = chrono.ChSystemFsi()
sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


mesh = fea.ChMesh()
mesh.SetAutomaticGravity(False)  


E = 2.1e11  
nu = 0.3    
rho = 7850  
beam_mat = fea.ChMaterialBeamEuler(E, nu, rho)


num_elements = 10
beam_length = 1.0
beam_height = 0.1
beam_width = 0.1
dx = beam_length / num_elements

for i in range(num_elements):
    node1 = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD(i * dx, 0, 0)))
    node2 = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD((i + 1) * dx, 0, 0)))
    mesh.AddNode(node1)
    mesh.AddNode(node2)
    
    element = fea.ChElementBeamEuler()
    element.SetNodes(node1, node2, beam_mat)
    element.SetAsNeutral()
    mesh.AddElement(element)


sys.Add(mesh)



revolute_joint = chrono.ChLinkRevolute()
revolute_joint.Initialize(mesh.GetNode(0), chrono.ChBody(), chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
sys.AddLink(revolute_joint)


motor_body = chrono.ChBody()
motor_body.SetBodyFixed(True)
motor = chrono.ChLinkMotorRotationAngle()
motor.Initialize(mesh.GetNode(num_elements), motor_body, chrono.ChFrameD(chrono.ChVectorD(beam_length, 0, 0)))
sys.AddLink(motor)


class MotorFunction(chrono.ChFunction):
    def __init__(self):
        super().__init__()
    def Get_y(self, x):
        return 0.1 * x  

motor_function = MotorFunction()
motor.SetAngleFunction(motor_function)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Beam Buckling Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 1, -3))
vis.AddTypicalLights()


sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED)
solver = chrono.ChSolverMINRES()
sys.SetSolver(solver)
solver.SetMaxIterations(100)
solver.EnableWarmStart(True)


while vis.Run():
    vis.BeginScene()
    vis.Render()
    sys.DoStepDynamics(1e-3)
    vis.EndScene()