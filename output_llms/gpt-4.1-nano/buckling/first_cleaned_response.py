import pychrono.core as chrono
import pychrono.irrlicht as irr



system = chrono.ChSystemNSC()


system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))




ground = chrono.ChBodyEasyBox(1, 0.1, 1, 1000, True, True)
ground.SetBodyFixed(True)
system.Add(ground)


beam_length = 10.0  
beam_width = 0.2
beam_height = 0.2
density = 7800  
num_fe_elements = 20  





nodes = []
segment_length = beam_length / num_fe_elements
for i in range(num_fe_elements + 1):
    position = chrono.ChVectorD(i * segment_length, 0, 0)
    node = chrono.ChNodeFEAxyz(chrono.ChVectorD(i * segment_length, 0, 0))
    nodes.append(node)


mesh = chrono.ChMesh()


for node in nodes:
    mesh.AddNode(node)


for i in range(num_fe_elements):
    
    element = chrono.ChElementBeamEulerFourthOrder()
    element.SetNodes(nodes[i], nodes[i + 1])
    
    material = chrono.ChMaterialBeamEulerFEM()
    material.SetYoungModulus(2.0e11)  
    material.SetGv(0.3)               
    element.SetMaterial(material)
    
    section = chrono.ChBeamSectionEulerConcrete()
    section.SetDimension(beam_width, beam_height)
    element.SetSection(section)

    mesh.AddElement(element)


mesh.SetAutomaticGravity(False)
mesh.Build()
system.Add(mesh)



fixed_end = chrono.ChBody()
fixed_end.SetBodyFixed(True)
fixed_end.SetPos(nodes[0].GetPos())
system.Add(fixed_end)


free_end = chrono.ChBody()
free_end.SetPos(nodes[-1].GetPos())
system.Add(free_end)






rot_motor = chrono.ChLinkRotMotor()
rot_motor.SetAxis(chrono.ChVectorD(0, 0, 1))
rot_motor.SetRelA(chrono.Q_from_AngX(0))
rot_motor.SetRelB(chrono.Q_from_AngX(0))
system.Add(rot_motor)


def motor_function(time):
    amplitude = 0.1  
    frequency = 0.5  
    return amplitude * chrono.ChFunction_Sine(chrono.ChVectorD(0, 0, 1), 0, frequency * 2 * chrono.CH_C_PI, 0)






constraint = chrono.ChLinkMateGeneric()
constraint.Initialize(nodes[0], fixed_end, False, nodes[0], fixed_end)
system.Add(constraint)


application = irr.ChIrrApp(system, "Beam Buckling Simulation", irr.dimension2d(1024, 768))
application.AddTypicalSky()
application.AddTypicalCamera(irr.QPTR(chrono.ChVectorD(5, 2, 0)), irr.QPTR(chrono.ChVectorD(0, 0, 0)))
application.AssetBindAll()
application.AssetUpdateAll()



system.SetSolverType(chrono.ChSolver.SOLVER_MUMPS)
system.SetMaxItersSolverSpeed(100)

runner = chrono.ChRunTimeStepper()
timestep = 1e-4  


while application.GetDevice().run():
    current_time = system.GetChTime()
    
    angle = motor_function(current_time)
    rot_motor.SetAngle(angle)
    
    system.DoStepDynamics(timestep)
    
    application.AssetUpdateAll()
    application.Render()