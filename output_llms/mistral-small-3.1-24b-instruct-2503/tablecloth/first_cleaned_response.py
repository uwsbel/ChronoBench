import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fem as fem
import pychrono.solver as solver


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


visualization = chronoirr.ChIrrApp()
visualization.AddSystem(system)
visualization.SetWindowSize(1280, 720)
visualization.SetWindowTitle('Tablecloth Folding Simulation')
visualization.SetTryFullscreen(True)
visualization.SetUseShadows(True)
visualization.SetRenderMode(chronoirr.ChIrrApp.RENDER_MODE_WIREFRAME)
visualization.SetShowInfos(True)
visualization.SetShowFPS(True)
visualization.SetShowTiming(True)
visualization.SetShowHelp(True)


solver = solver.ChSolverPardisoMKL()
system.SetSolver(solver)



material = fem.ChMaterialShellKirchhoff()
material.SetE(2e9)  
material.SetNu(0.3)  
material.SetK(1e6)   


mesh = fem.ChElementShellQuad4()
mesh.SetMaterial(material)
mesh.SetNodes([system.AddBody() for _ in range(4)])


positions = [
    chrono.ChVectorD(0, 0, 0),
    chrono.ChVectorD(1, 0, 0),
    chrono.ChVectorD(1, 1, 0),
    chrono.ChVectorD(0, 1, 0)
]

for i, pos in enumerate(positions):
    mesh.GetNode(i).SetPos(pos)


system.Add(mesh)



for node in mesh.GetNodes():
    node.AddForce(chrono.ChVectorD(0, -node.GetMass() * 9.81, 0))


for node in mesh.GetNodes():
    node.SetPos_dt(chrono.ChVectorD(0, 0, 0))


while visualization.Run():
    system.DoStepDynamics(0.01)  


visualization.Close()