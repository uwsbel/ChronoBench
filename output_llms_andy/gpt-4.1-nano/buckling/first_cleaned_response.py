import pychrono.core as chrono
import pychrono.irrlicht as irr


system = chrono.ChSystemNSC()


system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
system.SetTimestepperType(chrono.ChTimestepper.Type_Euler)
system.SetSolverType(chrono.ChSolver.Type_PSSOR)
system.SetMaxPenetrationRecoverySpeed(0.01)


ground = chrono.ChBodyEasyBox(10, 0.5, 1, 1000, True, True)
ground.SetBodyFixed(True)
system.Add(ground)


beam_length = 5.0
beam_width = 0.2
beam_height = 0.2
beam_density = 7850  





num_elements = 10
node_positions = []
for i in range(num_elements + 1):
    x = i * (beam_length / num_elements)
    node_positions.append(chrono.ChVectorD(x, 0, 0))


nodes = []


for pos in node_positions:
    node = chrono.ChNodeFEAxyz(chrono.ChVectorD(pos.x, pos.y, pos.z))
    nodes.append(node)
    system.Add(node)


elements = []
for i in range(num_elements):
    nodeA = nodes[i]
    nodeB = nodes[i + 1]
    element = chrono.ChElementTetra_4()  
    
    
    
    
    
    
    elements.append(element)


material = chrono.ChMaterialBeam()
material.SetYoungModulus(2.0e11)  
material.SetGf(0.0)
material.SetDensity(beam_density)





mesh = chrono.ChMesh()
for node in nodes:
    mesh.AddNode(node)



system.Add(mesh)


fixed_node = nodes[0]
fixed_node.SetFixed(True)


class SinusoidalMotor(chrono.ChFunction):
    def __init__(self, amplitude, frequency):
        super().__init__()
        self.amplitude = amplitude
        self.frequency = frequency

    def Get_y(self, x):
        import math
        return self.amplitude * math.sin(2 * math.pi * self.frequency * x)


end_node = nodes[-1]
motor_amplitude = 0.01  
motor_frequency = 1.0   


motor_function = SinusoidalMotor(motor_amplitude, motor_frequency)



ground_body = chrono.ChBodyEasyBox(0.1, 0.1, 0.1, 1000, True, True)
ground_body.SetBodyFixed(True)
system.Add(ground_body)


link = chrono.ChLinkLockPrismatic()
link.Initialize(end_node, ground_body, chrono.ChFrameD(chrono.ChVectorD(beam_length, 0, 0), chrono.Q_from_AngY(0)))
system.Add(link)


def update_motor(time):
    displacement = motor_function.Get_y(time)
    link.SetDriveFunction(chrono.ChFunction_Const(displacement))
    

vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('PyChrono Beam Buckling Simulation')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(beam_length/2, 1, 3))
vis.SetCameraAngle(45)


time = 0
end_time = 2.0  
while vis.Run():
    
    update_motor(time)
    
    system.DoStepDynamics(1e-3)
    time += 1e-3
    if time > end_time:
        break