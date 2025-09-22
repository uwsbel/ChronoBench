import pychrono as chrono
import pychrono.irrlicht as chronoirr


chrono.SetChronoDataPath('path_to_chrono_data')  


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


visualization = chronoirr.ChVisualSystemIrrlicht()
visualization.AttachSystem(system)
visualization.SetWindowSize(1280, 720)
visualization.SetWindowTitle('Beam Buckling Simulation')
visualization.Initialize()
visualization.AddLogo()
visualization.AddSkyBox()
visualization.AddTypicalLights()


ground = chrono.ChBodyEasyBox(10, 0.1, 10, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -0.5, 0))
ground.SetBodyFixed(True)
system.Add(ground)


beam = chrono.ChBody()
beam.SetPos(chrono.ChVectorD(0, 0.5, 0))
beam.SetMass(1)
beam.SetInertiaXX(chrono.ChVectorD(1, 1, 1))
beam.SetCollide(True)
system.Add(beam)


beam_geometry = chrono.ChBeamSectionPlane()
beam_geometry.SetThickness(0.01)
beam_geometry.SetYoungModulus(210e9)
beam_geometry.SetGantModulus(80.769e9)
beam_geometry.SetDensity(7800)


beam_elements = chrono.ChLinkFEabeam()
beam_elements.SetBeamSection(beam_geometry)
beam_elements.SetN(10)  
beam_elements.SetBeamR(0.05)  
beam_elements.SetBeamL(1)  
beam_elements.SetBeamMaterial(beam_geometry)
beam_elements.SetNodes(beam.GetNodes())
system.Add(beam_elements)


constraint = chrono.ChLinkLockRevolute()
constraint.Initialize(ground, beam, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
system.Add(constraint)


motor = chrono.ChFunction_Const(chrono.ChFunction_Const.CONST_TYPE_FORCE)
motor.Set_yconst(100)  
motor_function = chrono.ChLinkMotorRotationAngle()
motor_function.Initialize(ground, beam)
motor_function.SetMotionFunction(motor)
system.Add(motor_function)


solver = chrono.ChSolverMINRES()
system.SetSolver(solver)
system.SetTimestepperType(chrono.ChTimestepper.HDF5)


run_time = 10  
time_step = 0.01  
current_time = 0

while current_time < run_time:
    system.DoStepDynamics(time_step)
    visualization.Render()
    current_time += time_step


visualization.Close()