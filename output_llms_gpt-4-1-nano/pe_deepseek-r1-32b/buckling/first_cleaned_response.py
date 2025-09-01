importchrono as chrono
import math
import numpy as np


vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Beam Buckling Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chronovector3d(0, 3, 6))
vis.AddTypicalLights()


sys = chronosystemsmc()
sys.SetSolverType(ChSolver.Type_BARZILAI_BORWEIN)
sys.SetTimestepperType(ChTimestepper.Type_RK4)
sys.SetTimestep(0.001)


beam_length = 2.0
beam_height = 0.1
beam_width = 0.1
num_elements = 10


mesh = chfeamesh()


nodes = []
for i in range(num_elements + 1):
    pos = chronovector3d((i * beam_length / num_elements), 0, 0)
    node = chnodefea_xyzrot(chronoframed(pos))
    nodes.append(node)
    mesh.AddNode(node)


for i in range(num_elements):
    element = chfeabemeuler()
    element.SetNodes(nodes[i], nodes[i+1])
    element.SetSection(chfeabesectioneuler())
    element.GetSection().SetYoungModulus(2.1e11)  
    element.GetSection().SetShearModulus(7.9e10)
    element.GetSection().SetArea(beam_height * beam_width)
    element.GetSection().SetMoment(beam_height * beam_width**3 / 12)
    mesh.AddElement(element)


beam_shape = chvisualshapebox(chronovector3d(beam_length, beam_height, beam_width))
nodes[0].AddVisualShape(beam_shape)


fixed_link = chlinklocklock()
fixed_link.Initialize(nodes[0], None, chronoframed())
sys.Add(fixed_link)


def motor_function(time):
    return 0.1 * math.sin(2 * math.pi * time)

motor = chlinkmotorrotation()
motor.Initialize(nodes[-1], None, chronoframed())
motor.SetSpeedFunction(chrono.ChFunction_Sine(0, 0.1, 0))


base_shape = chvisualshapebox(chronovector3d(0.2, beam_height, beam_width))
base_shape.SetColor(chronocolor(0.5, 0.5, 0.5))
nodes[0].AddVisualShape(base_shape)


ground = chbody()
ground.SetPos(chronovector3d(0, -1, 0))
ground_shape = chvisualshapebox(chronovector3d(10, 1, 1))
ground_shape.SetColor(chronocolor(0.2, 0.2, 0.2))
ground.AddVisualShape(ground_shape)
sys.Add(ground)


sys.Add(mesh)


time = 0
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    sys.DoStepDynamics(0.001)
    time += 0.001
    
    
    motor.GetSpeedFunction().SetAmplitude(0.1 * math.sin(2 * math.pi * time))