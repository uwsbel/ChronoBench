import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea


length = 1.0  
thickness = 0.01  
density = 2700  
E = 70e9  
nu = 0.33  
force = 100  
motor_speed = 0.1  


my_system = chrono.ChSystemNSC()


beam = fea.ChElementBeamEuler()
beam.SetDensity(density)
beam.SetYoungModulus(E)
beam.SetPoissonRatio(nu)
beam.SetLength(length)
beam.SetSectionArea(thickness * thickness)
beam.SetIyy(thickness * thickness * thickness * thickness / 12)
beam.SetIzz(thickness * thickness * thickness * thickness / 12)


node1 = fea.ChNodeFEAxyz()
node2 = fea.ChNodeFEAxyz()
node1.SetPosition(chrono.ChVectorD(0, 0, 0))
node2.SetPosition(chrono.ChVectorD(length, 0, 0))


my_system.Add(node1)
my_system.Add(node2)
my_system.Add(beam)


class CustomMotor(chrono.ChFunction):
    def __init__(self):
        super().__init__()
        self.speed = motor_speed

    def Get_y(self, x):
        return self.speed * x


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(node1, node2, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT))
motor.SetMotorFunction(CustomMotor())


my_system.Add(motor)


constraint = chrono.ChLinkPointPoint()
constraint.Initialize(node1, node2, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT))


my_system.Add(constraint)


force_obj = chrono.ChForce()
force_obj.SetF(chrono.ChVectorD(0, -force, 0))
node2.AddForce(force_obj)


myapplication = chronoirr.ChIrrApp(my_system, 'PyChrono Beam Buckling Simulation', chronoirr.dimension2du(800, 600))


myapplication.AddTypicalLights()
myapplication.AddCamera(chrono.ChVectorD(0, 0, 1))
myapplication.AssetBindAll()
myapplication.AssetUpdateAll()


solver = chrono.ChSolverBB()
my_system.SetSolver(solver)
timestepper = chrono.ChTimestepperEulerImplicit()
my_system.SetTimestepper(timestepper)


myapplication.SetTimestep(0.01)
myapplication.SetTryRealtime(True)
while myapplication.GetDevice().run():
    myapplication.BeginScene()
    myapplication.DrawAll()
    myapplication.DoStep()
    myapplication.EndScene()