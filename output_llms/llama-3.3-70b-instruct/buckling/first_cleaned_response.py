import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea


sys = chrono.ChSystemNSC()


sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


ground = chrono.ChBodyEasyBox(sys, 10, 1, 10, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -5, 0))
ground.SetBodyFixed(True)


beam = fea.ChBeamSectionEulerSimsons()
beam.SetYoungModulus(200e9)  
beam.SetArea(0.01)  
beam.SetIy(1e-5)  
beam.SetIz(1e-5)  


mesh = fea.ChMesh()
mesh.SetBeamSection(beam)
mesh.SetNodesNumber(10)  
mesh.SetLength(10)  


beam_body = fea.ChBodyFEA(mesh)
beam_body.SetDensity(7850)  
beam_body.SetPos(chrono.ChVectorD(0, 5, 0))  


def custom_motor(t):
    return 10 * chrono.ChFunction_Const()  

motor = chrono.ChLinkMotorLinearPointPoint()
motor.Initialize(beam_body, ground, chrono.ChVectorD(0, 5, 0), chrono.ChVectorD(0, -5, 0))
motor.SetForceFunction(custom_motor)


constraint = chrono.ChLinkLockPointPoint()
constraint.Initialize(beam_body, ground, chrono.ChVectorD(0, 5, 0), chrono.ChVectorD(0, -5, 0))


sys.Add(ground)
sys.Add(beam_body)
sys.Add(motor)
sys.Add(constraint)


solver = chrono.ChSolverNSC()
solver.SetMaxIters(100)
solver.SetTol(1e-6)
sys.SetSolver(solver)

timestepper = chrono.ChTimestepperEuler()
sys.SetTimestepper(timestepper)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AddCamera(chrono.ChVectorD(0, 0, 10))
vis.AddTypicalCameraMovements()


while vis.Run():
    sys.DoStepDynamics(0.01)
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()