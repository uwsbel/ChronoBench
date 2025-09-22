import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.fea as fea


system = chrono.ChSystemNSC()


mesh = fea.ChMesh()
mesh.AddPoint(chrono.ChVectorD(0, 0, 0))
mesh.AddPoint(chrono.ChVectorD(1, 0, 0))
mesh.AddPoint(chrono.ChVectorD(1, 1, 0))
mesh.AddPoint(chrono.ChVectorD(0, 1, 0))


node_id_1 = mesh.AddNode(chrono.FEA_NODE_POINT, 0)
node_id_2 = mesh.AddNode(chrono.FEA_NODE_POINT, 1)
node_id_3 = mesh.AddNode(chrono.FEA_NODE_POINT, 2)
node_id_4 = mesh.AddNode(chrono.FEA_NODE_POINT, 3)

mesh.AddElementShellANCF_4(node_id_1, node_id_2, node_id_3, node_id_4)


material = fea.ChMaterialShellANCF()
material.Set_E(200e9)
material.Set_nu(0.3)
material.Set_thickness(0.001)


model = fea.ChModelFEA()
model.AddMesh(mesh, material)
model.SetSolverType(chrono.ChSolver.Type_PARDISO_MKL)


system.Add(model)


app = irr.ChIrrApp(system, 'Tablecloth Folding Simulation', chrono.ChVectorD(0.5, 0.5, 1), 1, 1000, 600, False)


app.AddTypicalLogo()
app.AddTypicalCamera(chrono.ChVectorD(0.5, 0.5, 1))
app.AddTypicalLights()
app.AssetBindAll()
app.AssetUpdateAll()


app.SetTimestep(0.01)
app.SetTryRealtime(True)
app.Run()