importchrono
import math


system = chrono.ChSystemNSC()
solver = chrono.ChLcpSolver()
system.SetSolver(solver)
timestepper = chrono.ChTimestepperHHT(system)
system.SetTimestepper(timestepper)


young_modulus = 2e11
poisson_ratio = 0.3
cross_section_area = 0.01
beam_length = 1.0
num_elements = 10
element_length = beam_length / num_elements


elements = []
for i in range(num_elements):
    element = chrono.ChBody()
    element.SetMass(1.0)
    element.SetPos(chrono.ChVectorD(i * element_length, 0, 0))
    elements.append(element)
    system.Add(element)
    
    
    asset = chrono.ChVisualShape()
    asset.SetGeometry(chrono.ChBox(0.5 * element_length, 0.1, 0.1))
    element.AddAsset(asset)


for i in range(num_elements - 1):
    constraint = chrono.ChLinkSpherical()
    constraint.Initialize(elements[i], elements[i+1], chrono.ChCoordsD(chrono.ChVectorD(element_length, 0, 0)))
    system.Add(constraint)


fixed = chrono.ChLinkFix()
fixed.Initialize(elements[0], chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
system.Add(fixed)


def motor_function(time):
    return 1000 * math.sin(time * 2 * math.pi / 5)


force = chrono.ChForce()
force.SetF(motor_function(0) * chrono.ChVectorD(-1, 0, 0))
elements[-1].AddForce(force)


vis = chrono.ChIrrApp(system, "Beam Buckling Simulation", chrono.CH.dimension2D(800, 600))
vis.AddLight(chrono.ChVectorD(0, 1, 3), chrono.ChColor(1, 1, 1))
vis.SetCamera(chrono.ChVectorD(1.5, 1.5, 1.5), chrono.ChVectorD(0, 0, 0))
vis.SetRenderMode(chrono.ChIrrApp.RENDER_MODE_OPENGL)
vis.EnableShadows()
vis.SetRenderFrequency(60)


step_size = 0.001
num_steps = 5000


vis.Begin()
for step in range(num_steps):
    current_time = step * step_size
    force.SetF(motor_function(current_time) * chrono.ChVectorD(-1, 0, 0))
    system.DoStepDynamics(step_size)
    vis.Render()
vis.End()