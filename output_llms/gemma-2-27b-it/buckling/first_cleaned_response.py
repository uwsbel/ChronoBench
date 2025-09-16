import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr


my_system = chrono.ChSystemNSC()
my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


beam_material = fea.ChMaterialShellANCF(rho=7850, E=200e9, nu=0.3)
beam_section = fea.ChSectionShellParametric(
    type=fea.ChSectionShellParametric.PARABOLIC, thickness=0.01
)

beam_length = 1.0
beam_nodes = 10
beam = fea.ChBeamIGA(beam_nodes, beam_length, beam_material, beam_section)


beam.SetPos(chrono.ChVectorD(0, 0, 0))
beam.SetFixed(True)


class MyMotorFunction(chrono.ChFunction):
    def __init__(self, amplitude, frequency):
        chrono.ChFunction.__init__(self)
        self.amplitude = amplitude
        self.frequency = frequency

    def Get_y(self, x):
        return self.amplitude * chrono.ChMath.Sin(self.frequency * x)

motor_function = MyMotorFunction(amplitude=0.01, frequency=2 * chrono.CH_C_PI)


motor = chrono.ChLinkMotorRotationSpeed()
motor.SetTargetFunction(motor_function)
motor.Attach(beam.GetNode(beam_nodes - 1))


my_system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
my_system.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED)
my_system.SetStep(0.001)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(my_system)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle("Beam Buckling Simulation")
vis.Initialize()


while vis.Run():
    my_system.DoStepDynamics(0.001)
    vis.Render()