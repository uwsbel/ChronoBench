from pychrono import ChSystem, ChBody, ChPointPointConstraint, ChMaterialShell, ChVectorD, ChQuaternionD
from pychrono.irrlicht import ChIrrApp
from pychrono.solver import ChSolverPardisoMKL


system = ChSystem()
system.Set_G_acc(ChVectorD(0, 0, -9.81))  





material = ChMaterialShell()
material.Set_E(1e6)  
material.Set_nu(0.3)  





app = ChIrrApp(system, 'Tablecloth Folding', 'data/textures/', 'data/models/')
app.AddCamera(ChVectorD(1, 1, 1), ChVectorD(0, 0, 0), ChVectorD(0, 1, 0))


solver = ChSolverPardisoMKL(system)
system.SetSolver(solver)


app.SetTimestep(0.01)
while app.Run():
    system.DoStepDynamics(app.GetTimestep())