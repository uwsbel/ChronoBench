import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fem as fem


chrono.ChSystemNSC sys


solver = chrono.ChSolverMINRES()
sys.SetSolver(solver)
sys.SetTimestepperType(chrono.ChTimestepper.HBDF)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('Beam Buckling Simulation')
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(1, 1, 1), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()


beam = fem.ChElementBeamFEM()
beam.SetLength(1.0)  
beam.SetCrossSectionArea(0.01)  
beam.SetYoungModulus(210e9)  
beam.SetDensity(7800)  


sys.Add(beam)


fixed_constraint = chrono.ChLinkLockRevolute()
fixed_constraint.Initialize(sys.GetGroundBody(), beam.GetMarker('M_1'))
sys.AddLink(fixed_constraint)


motor = chrono.ChFunction_Const()
motor.Set_yconst(1000)  
motor_applier = chrono.ChLinkMotorRotationAngle()
motor_applier.Initialize(beam.GetMarker('M_2'), motor)
motor_applier.SetMotorFunction(motor)
sys.AddLink(motor_applier)


beam.SetPos(chrono.ChVectorD(0, 0, 0))
beam.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))  


sys.SetStep(0.01)


while vis.Run():
    vis.BeginScene()
    vis.Render()
    sys.DoStepDynamics(0.01)
    vis.EndScene()

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fem as fem


sys = chrono.ChSystemNSC()


solver = chrono.ChSolverMINRES()
sys.SetSolver(solver)
sys.SetTimestepperType(chrono.ChTimestepper.HBDF)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('Beam Buckling Simulation')
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(1, 1, 1), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()


beam = fem.ChElementBeamFEM()
beam.SetLength(1.0)  
beam.SetCrossSectionArea(0.01)  
beam.SetYoungModulus(210e9)  
beam.SetDensity(7800)  


sys.Add(beam)


fixed_constraint = chrono.ChLinkLockRevolute()
fixed_constraint.Initialize(sys.GetGroundBody(), beam.GetMarker('M_1'))
sys.AddLink(fixed_constraint)


motor = chrono.ChFunction_Const()
motor.Set_yconst(1000)  
motor_applier = chrono.ChLinkMotorRotationAngle()
motor_applier.Initialize(beam.GetMarker('M_2'), motor)
motor_applier.SetMotorFunction(motor)
sys.AddLink(motor_applier)


beam.SetPos(chrono.ChVectorD(0, 0, 0))
beam.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))  


sys.SetStep(0.01)


while vis.Run():
    vis.BeginScene()
    vis.Render()
    sys.DoStepDynamics(0.01)
    vis.EndScene()