importchrono
importchrono.fea
importchrono irrlicht
importnumpy as np


system = chrono.ChSystem()
system.SetSolverType(chrono.ChSolverType.MINRES)
system.SetTimestepperType(chrono.ChTimestepperType.HHT)
system.GetSettings().SetNumThreads(4)
system.GetSettings().SetGravity(chrono.ChVectorD(0, -9.81, 0))


vis = chrono.ChVisualSystemIrrlicht()
vis.SetWindowTitle('Beam Buckling Simulation')
vis.SetWindowSize(1024, 768)
vis.SetCameraPosition(chrono.ChVectorD(0, 2, 3))
vis.SetCameraRotation(chrono.ChVectorD(0, 0, 0))
vis.SetLightPosition(chrono.ChVectorD(0, 2, 3))
vis.SetLightDir(chrono.ChVectorD(0, -1, -1))
vis.AddTypicalGrid()
vis.AddTypicalAxes()
system.SetVisualSystem(vis)


beam_length = 1.0
num_elements = 10
element_length = beam_length / num_elements
E = 2e11  
I = 0.0001  
A = 0.01  
rho = 7800  

nodes = []
for i in range(num_elements + 1):
    node = chrono.fea.ChNodeFEAxyz()
    node.SetPos(chrono.ChVectorD(0, i * element_length, 0))
    system.Add(node)
    nodes.append(node)

for i in range(num_elements):
    element = chrono.fea.ChElementBeamFinite()
    element.SetNodes(nodes[i], nodes[i+1])
    element.SetYoungModulus(E)
    element.SetMomentInertia(I)
    element.SetArea(A)
    element.SetDensity(rho)
    system.Add(element)


class CustomMotor(chrono.ChLinkMotor):
    def GetMotorForce(self, time):
        F0 = 1000  
        omega = 1  
        return F0 * np.sin(omega * time)

motor = CustomMotor()
motor.Initialize(nodes[-1], system.GetGround(), False)
system.Add(motor)


constraint = chrono.ChLinkLock()
constraint.Initialize(nodes[0], system.GetGround(), False)
system.Add(constraint)


vis.Initialize()
vis.AddAll()


for step in range(1000):
    system.DoStepDynamics(0.001)
    vis.Render()
    if vis.GetDevice().IsKeyDown(chrono.KEY_ESCAPE):
        break

vis.ApplicationEnd()