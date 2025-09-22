import pychrono as chrono
import pychrono.irrlicht as chronoirr


chrono.Ch纪 = chrono.Ch纪


system = chrono.ChSystemNSC()


system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


vis_engine = chronoirr.ChVisualSystemIrrlicht()
vis_engine.AttachSystem(system)
vis_engine.SetWindowSize(800, 600)
vis_engine.SetWindowTitle('Tablecloth Folding Simulation')


vis_engine.Initialize()
vis_engine.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'), 0.1)
vis_engine.AddSkyBox()
vis_engine.AddCamera(chrono.ChVectorD(0, 0, 10))
vis_engine.SetFramerateRange(30, 60)


material = chrono.ChMaterialSurfaceNSC()
material.SetFriction(0.5)
material.SetDampingF(0.7)
material.SetCompliance(0)
material.SetComplianceT(0)



grid_size = 10
node_spacing = 0.5
grid = []


for i in range(grid_size):
    for j in range(grid_size):
        node = chrono.ChNode()
        node.SetPos(chrono.ChVectorD(i * node_spacing, j * node_spacing, 0))
        system.Add(node)
        grid.append(node)


for i in range(grid_size - 1):
    for j in range(grid_size - 1):
        
        elem = chrono.ChShellElementShellShell()
        elem.SetShell1(grid[i * grid_size + j])
        elem.SetShell2(grid[(i + 1) * grid_size + j])
        elem.SetShell3(grid[(i + 1) * grid_size + j + 1])
        elem.SetShell4(grid[i * grid_size + j + 1])
        elem.SetMaterialSurface(material)
        system.AddElement(elem)


solver = chrono.ChSolver()
solver.SetType(chrono.ChSolverType.PARDISO_MKL)
system.SetSolver(solver)


step_size = 0.001
vis_engine.SetTimestep(step_size)
vis_engine.Start()

while vis_engine.GetRun():
    system.DoStepDynamics(step_size)
    vis_engine.DrawAll()
    vis_engine.GetDevice().display()


vis_engine退出()