import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea


chrono.SetChronoDataPath("./")  


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


beam_length = 1.0
beam_height = 0.1
beam_width = 0.1
num_elements = 20


body = chrono.ChBodyEasy()
body.SetBodyFixed(False)
system.Add(body)


mesh = fea.ChBeamMesh(beam_length, beam_height, beam_width, num_elements)
body.AddAsset(mesh)


material = chrono.ChMaterialSurfaceNSC()
material.SetFriction(0.2)
material.SetRestitution(0.1)
body.SetMaterialSurface(material)


motor_force = 10.0  
motor_point = chrono.ChVectorD(0.0, 0.0, 0.0)  
motor = chrono.ChPointPointMotor()
motor.Set_Point1(motor_point)
motor.Set_Point2(chrono.ChVectorD(beam_length, 0.0, 0.0))
motor.Set_Force(motor_force)
motor.Set_Mode(chrono.ChPointPointMotor.FORCE)
system.Add(motor)
motor.Attach(body)



fix_point = chrono.ChVectorD(0.0, 0.0, 0.0)
fix_constraint = chrono.ChLinkMateRevolute()
fix_constraint.Initialize(body, None, fix_point, chrono.ChVectorD(0, 0, 0))
system.Add(fix_constraint)


system.SetSolverType(chrono.ChSolver.Type_SOR)
system.SetSolverMaxIterations(100)
system.SetSolverTolerance(1e-5)


timestepper = chrono.ChTimestepper.Type_EULER_IMPLICIT
system.SetTimestepperType(timestepper)
step_size = 0.001
system.SetStep(step_size)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Beam Buckling Simulation')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(beam_length * 1.5, beam_length * 0.5, beam_length * 0.5))
vis.AddTypicalLights()



time_end = 5.0
while system.GetChTime() < time_end:
    system.DoStepDynamics(step_size)
    vis.DrawMultithreaded(True)
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

print("Simulation finished.")