import pychrono as chrono

from pychron.core.helpers import system_error_handler




system_error_handler.set_verbosity(chrono.V.ERROR)

my_system = chrono.ChSystemNSC()






beam_element = chrono.ChBeamElementLinear()

beam_element.SetMaterialIndex(0)

beam_element.SetYoungsModulus(210e9)  

beam_element.SetPoissonsRatio(0.3)

beam_element.SetLength(1.0)  

beam_element.SetMassPerLength(100.0)  

beam_element.SetNodePosition(chrono.ChVectorD(0, 0, 0))

beam_element.SetNodeVelocity(chrono.ChVectorD(0, 0, 0))

beam_element.SetNodeAcceleration(chrono.ChVectorD(0, 0, 0))




my_system.Add(beam_element)




def custom_motor_function(beam_element, force_magnitude):

    force_vector = chrono.ChVectorD(0, 0, force_magnitude)

    beam_element.ApplyForce(force_vector)






custom_motor_function(beam_element, 1000.0)  






base_node = beam_element.GetNode(0)

base_constraint = chrono.ChLinkLockJoint()

base_constraint.Initialize(base_node, my_system.GetGroundBody().GetNode(0))

my_system.Add(base_constraint)




vis = chrono.ChVisualSystemIrrlicht()

my_system.SetVisualization(vis)




my_system.SetSolverType(chrono.ChSolverType.SOR)

my_system.SetTimestep(0.01)




my_system.DoStepDynamics(1.0)




vis.Render()

vis.Close()