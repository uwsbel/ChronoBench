import Chrono

import chrono

import pychrono

import pychrono.fea as fea

import pychrono.irrlicht as irr




chrono.Init()




system = chrono.ChSystemNSC()




material = fea.ChMaterialShellIsoKirchhoff(E=1000, rho=1.2, mu=0.3)




mesh = fea.ChMshShellGrid()



mesh.AddRect(0.5, 0.5)




mesh.Generate(system)




mesh.SetMaterial(material)




system.Add(mesh, chrono.ChBodyGroup())




vis = irr.ChIrrApp(&system, "Tablecloth Folding Simulation", chrono.COORDTYPE_DYNAMIC, "", chrono.SColor(0, 255, 0))




system.SetSolver(chrono.ChSolverPardisoMKL())




while (vis.GetDevice()->UserQuit() == False):

    system.DoStepDynamics(0.01)

    vis.Render()




chrono.destroy([system, mesh, vis])