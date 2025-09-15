importchrono
import math
import time


chrono.ChRandom::SetSeed(time.time())
sys = chrono.ChSystemNSC()
vis = chrono.ChVisualSystemIrrlicht()
vis.SetWindowTitle('Beam Buckling Simulation')
vis.SetWindowSize(1024,768)
vis.SetNumberOfCameras(1)
vis.Camera_placement(chrono.ChVectorD(0,5,5), chrono.ChVectorD(0,0,0))
vis.EnableLight(chrono.ChLightType.CH_LIGHT_DIRECTIONAL)
vis.SetLightDirection(chrono.ChVectorD(1,1,1))
sys.SetVisualSystem(vis)


young_modulus = 2.1e11  
poisson_ratio = 0.3
area = 0.01  
inertia = 0.0001  
num_elements = 10
length = 2.0
spacing = length / num_elements


beam = chrono.ChBody()
beam.SetMass(0)
beam.SetBodyFixed(True)
sys.Add(beam)


nodes = []
for i in range(num_elements + 1):
    node = chrono.ChNodeFEAxyz()
    node.SetPos(chrono.ChVectorD(0, 0, i*spacing))
    node.SetMass(0)
    sys.Add(node)
    nodes.append(node)

    if i > 0:
        element = chrono.ChElementBeam()
        element.SetNodes(nodes[i-1], node)
        element.SetYoungModulus(young_modulus)
        element.SetPoissonRatio(poisson_ratio)
        element.SetArea(area)
        element.SetInertia(inertia)
        sys.Add(element)


for element in sys.GetElements():
    if isinstance(element, chrono.ChElementBeam):
        element.SetVisualization(chrono.ChVisualizationFEA(chrono.ChColor(0.8,0.8,0.8)))


constraint = chrono.ChLinkLockNode()
constraint.Initialize(nodes[0], beam.GetFrame_REF(), chrono.ChVectorD(0,0,0))
sys.Add(constraint)


def motor_function(time):
    max_force = 100000  
    max_time = 5  
    if time < max_time:
        return max_force * (time / max_time)
    else:
        return max_force

motor = chrono.ChLinkMotorRotation()
motor.Initialize(nodes[-1], beam.GetFrame_REF(), chrono.ChVectorD(0,0,1))
motor.SetPositionFunction(motor_function)
sys.Add(motor)


mass = chrono.ChBody()
mass.SetMass(10)
mass.SetPos(chrono.ChVectorD(0, 0, length))
mass.SetVisualization(chrono.ChVisualizationSphere(0.1, chrono.ChColor(1,0,0)))
sys.Add(mass)


constraint_mass = chrono.ChLinkLockNode()
constraint_mass.Initialize(nodes[-1], mass.GetFrame_REF(), chrono.ChVectorD(0,0,0))
sys.Add(constraint_mass)


sys.SetSolverType(chrono.ChSolverType.CH_SOLVER_HHT)
sys.GetSolver().SetTolerance(1e-6)
sys.GetSolver().SetMaxIterations(100)
sys.SetTimestepperType(chrono.ChTimestepperType.CH_Timestepper.Implicit_Newmark)
sys.GetTimestepper().SetGamma(0.58)
sys.GetTimestepper().SetBeta(0.3025)


sys.Add(beam)
sys.Add(nodes)
sys.Add(constraint)
sys.Add(motor)
sys.Add(mass)
sys.Add(constraint_mass)


vis.Begin()
while vis.Run():
    sys.DoStepDynamics(0.001)
    time.sleep(0.001)
vis.End()