fromchrono = __import__('chrono')
fromchrono.util import *
fromchrono.solver import *
fromchrono.visualization import *
fromchrono.core import *


chrono = __import__('chrono')
engine = chrono.Ch纪元()
engine.SetChTime(0)


default_time_step = 0.001
default_solver_iterations = 100


beam = chrono.ChBody()
beam.SetName("beam")
beam.SetType(chrono.ChBodyType.BEAM)
beam.SetMaterialSurface(chrono.ChMaterialSurfaceNSC())
beam.SetCollide(True)
beam.SetPos(chrono.ChVectorD(0, 0, 0))
beam.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))
beam.SetMass(100)
beam.SetInertiaXX(chrono.ChVectorD(1, 1, 1))
beam.SetBodyFixed(True)
beam.SetBodyPos(chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT))
beam.AddAsset(chrono.ChTexture("assets/textures/beam.png"))


beam_element1 = chrono.ChBeamElement()
beam_element1.SetMaterial(chrono.ChMaterialStd(1000, 0.3, 0.3))
beam_element1.SetNodes(chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(1, 0, 0))
beam_element1.SetLength(1)
beam_element1.SetCrossSection(chrono.ChVectorD(0.05, 0.05, 0.05))
beam_element1.SetElasticity(210000000000)
beam_element1.SetPoissonRatio(0.3)
beam_element1.SetMomentOfInertia(chrono.ChVectorD(0.000025, 0.000025, 0.000025))
beam_element1.SetInitialLength(1)
beam_element1.SetInitialRotation(chrono.QUNIT)
beam_element1.SetInitialTranslation(chrono.ChVectorD(0, 0, 0))
beam.AddElement(beam_element1)


motor = chrono.ChMotor()
motor.SetMotorType(chrono.ChMotorType.MOTOR_TYPE_FRICTIONLESS)
motor.SetTorque(100)
motor.SetPos(chrono.ChVectorD(0, 0, 0))
beam.AddComponent(motor)


constraint = chrono.ChConstraint()
constraint.SetType(chrono.ChConstraintType.CONSTRAINT_TYPE_Hooke)
constraint.SetSpringConstant(1000000)
constraint.SetDampingConstant(100000)
constraint.SetFirstNode(beam_element1.GetNodes()[0])
constraint.SetSecondNode(beam_element1.GetNodes()[1])
constraint.SetRestLength(beam_element1.GetLength())
beam.AddConstraint(constraint)


solver = chrono.ChSolver()
solver.SetMaxIterations(default_solver_iterations)
solver.SetDampingFactor(0.01)
engine.AddSolver(solver)

timestepper = chrono.ChTimestepper()
timestepper.SetSolver(solver)
timestepper.SetTimeStep(default_time_step)
engine.AddTimestepper(timestepper)


vis_engine = chrono.ChVisualEngine()
vis_engine.Initialize()
vis_engine.AddLight(chrono.ChVectorD(10, 10, 10), chrono.ChColor(1, 1, 1))
vis_engine.AddCamera(chrono.ChVectorD(0, 0, 10), chrono.ChVectorD(0, 0, 0))
vis_engine.AddLight(chrono.ChVectorD(-10, -10, 10), chrono.ChColor(1, 1, 1))
vis_engine.AddCamera(chrono.ChVectorD(0, 0, -10), chrono.ChVectorD(0, 0, 0))


while engine.GetChTime() < 10:
    engine.DoStepDynamics()
    vis_engine.RenderStep()


engine.End()