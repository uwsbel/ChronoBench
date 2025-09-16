import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.solver as solver


chrono.ChSystemNSC my_system()
my_system.SetSolverType(solver.ChSolver.PARDISOMKL)  


E = 2e9  
nu = 0.3  
thickness = 0.001  


material = chrono.ChMaterialShellKirchhoff()
material.SetYoungModulus(E)
material.SetPoissonRatio(nu)
material.SetThickness(thickness)


width = 1.0  
height = 1.0  
num_x = 10  
num_y = 10  


mesh = chrono.ChShellMesh()
mesh.CreateGrid(width, height, num_x, num_y, material)


my_system.Add(mesh)


visualization = chronoirr.ChIrrApp(my_system, "Tablecloth Folding Simulation", chronoirr.dimension2du(800, 600))
visualization.AddTypicalSky()
visualization.AddTypicalLogo(chrono.GetChronoDataFile("logo_chrono_small.png"))
visualization.AddTypicalFloor()


visualization.AddVisualSystem(chrono.ChVisualSystemIrrlicht())
visualization.AssetBind(mesh)


visualization.SetTimestep(0.01)
visualization.SetTryRealTime(True)


while visualization.Run():
    my_system.DoStepDynamics(0.01)


visualization.Close()