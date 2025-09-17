import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea


length = 1.0  
thickness = 0.01  
width = 0.1  
E = 2.1e11  
nu = 0.3  
rho = 7800  
force = 1000  


my_system = chrono.ChSystemNSC()


beam_mesh = fea.ChMesh()


for i in range(11):
    node = fea.ChNodeFEAxyzD()
    node.SetPos(chrono.ChVectorD(i * length / 10, 0, 0))
    beam_mesh.AddNode(node)


for i in range(10):
    element = fea.ChElementBeamEuler()
    element.SetNodes(beam_mesh.GetNode(i), beam_mesh.GetNode(i + 1))
    element.SetDimensions(width, thickness)
    element.SetMaterialProperties(E, nu, rho)
    beam_mesh.AddElement(element)


my_system.Add(beam_mesh)


class CustomMotor(chrono.ChFunction):
    def __init__(self):
        super().__init__()
        self.amplitude = 0.1
        self.frequency = 1.0

    def Get_y(self, x):
        return self.amplitude * chrono.ChMath.sin(2 * chrono.CH_C_PI * self.frequency * x)


motor = chrono.ChLinkMotorLinearForce()
motor.Initialize(beam_mesh.GetNode(0), beam_mesh.GetNode(10), chrono.ChFrameD())
motor.SetForceFunction(CustomMotor())
my_system.Add(motor)


for i in range(10):
    constraint = chrono.ChLinkTSDA()
    constraint.Initialize(beam_mesh.GetNode(i), beam_mesh.GetNode(i + 1), chrono.ChFrameD())
    my_system.Add(constraint)


application = chronoirr.ChIrrApp(my_system, 'Beam Buckling Simulation', chronoirr.dimension2du(800, 600))


my_system.SetSolverType(chrono.ChSolver.Type_APGD)
my_system.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED)


application.AddTypicalSky()
application.AddTypicalCamera(chronoirr.vector3df(0, 0, -1))
application.AddTypicalLights()
application.AssetBindAll()
application.AssetUpdateAll()
application.Simulate(10)


application.SetTimestep(0.01)
while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()