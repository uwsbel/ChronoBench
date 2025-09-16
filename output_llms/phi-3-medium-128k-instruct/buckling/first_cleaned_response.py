from pychrono import *

from pychrono.fea import *

from pychrono.irrlicht import *




chrono = Chrono()

chrono_assets = Chrono_Assets()

chrono.SetAssets(chrono_assets)




irr = Irrlicht()

irr.SetUse3D(True)

irr.SetUse3DProjection(True)




beam_length = 2.0

beam_width = 0.1

beam_height = 0.1

beam_material = IrMaterial_Dominant_Diffuse(Color(0.5, 0.5, 0.5))

beam_mesh = ChBody_Mesh()

beam_mesh.SetScale(Vector(beam_width, beam_height, beam_length))

beam_mesh.SetMaterial(beam_material)

beam_body = ChBody(1.0, beam_mesh)

beam_body.SetPos(Vector(0.0, 0.0, beam_length / 2))




system = ChSystemNSC()

system.SetSolver(ChSolver_Newton())

system.SetSolverIterationLimit(100)

system.SetMaxLinearSolveIterations(100)

system.SetMaxNonlinearSolveIterations(100)

system.SetLinearSolverMethod(ChSolver_Type_Newton_Linear_Full)

system.SetNonlinearSolverMethod(ChSolver_Type_Newton_Nonlinear_Full)

system.AddBody(beam_body)




motor = ChBody_Motor(1.0, Vector(0.0, 0.0, 1.0))

motor.SetVel(Vector(0.0, 0.0, 10.0))

system.AddJoint(motor)




beam_elements = FEA_Beam_Mesh()

beam_elements.SetScale(Vector(beam_width, beam_height, beam_length))

beam_elements.SetMaterial(ChMaterial_Iron)

beam_elements.SetElasticModulus(210e9)

beam_elements.SetPoissonsRatio(0.3)

beam_elements.SetDensity(7850.0)

beam_elements.SetMass(0.0)




beam_body.AddMesh(beam_elements)




irr.AddBody(beam_body)

irr.SetUse3DProjection(True)




system.SetDataStepper(ChTimeStepper_Z_Euler(1.0/100.0, 1.0/100.0))

while chrono.IsTimeStepAvailable():

    system.DoStepDynamics()

    irr.UpdateScene()




chrono.Free()