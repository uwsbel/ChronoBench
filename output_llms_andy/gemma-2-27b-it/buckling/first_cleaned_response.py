import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as irr
import math







mysystem = chrono.ChSystemNSC()
mysystem.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))






vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(mysystem)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('Beam Buckling Simulation')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(10, 5, 5))
vis.AddLight(chrono.ChVectorD(5, 5, 5), chrono.ChColor(1, 1, 1))







beam_length = 2.0
beam_width = 0.1
beam_height = 0.1

beam_material = fea.ChMaterialShellANCF(100e9, 0.3, 7850)  

beam_section = fea.ChBeamSectionRectangular(beam_width, beam_height)

beam = fea.ChBeamFEA(beam_section, beam_material)
beam.SetNumElements(20)


beam.SetBodyFixed(True)
beam.SetPos(chrono.ChVectorD(0, 0, 0))
beam.SetRot(chrono.Q_from_AngAxis(math.pi / 2, chrono.ChVectorD(0, 0, 1)))


mysystem.Add(beam)








fixed_constraint = chrono.ChLinkLockRevolute()
fixed_constraint.Initialize(beam.GetNode(0), mysystem.GetBody(0))
mysystem.Add(fixed_constraint)







def motor_function(time):
    return 0.1 * math.sin(2 * math.pi * 0.5 * time)

motor_constraint = chrono.ChLinkMotorRotationSpeed()
motor_constraint.Initialize(beam.GetNode(beam.GetNumNodes() - 1), mysystem.GetBody(0))
motor_constraint.SetSpeedFunction(motor_function)
mysystem.Add(motor_constraint)







mysystem.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
mysystem.SetSolverMaxIterations(100)
mysystem.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED)
mysystem.SetTimestep(0.001)







while vis.Run():
    mysystem.DoStepDynamics(0.01)
    vis.Render()