import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.mkl as mkl


chrono.SetChronoDataPath('PYCHRONO_DATA_DIR')
my_system = chrono.ChSystemNSC()



material = chrono.ChMaterialSurfaceNSC()
material.SetYoungModulus(1e7)
material.SetPoissonRatio(0.3)


mesh = chrono.ChMesh()



tablecloth = chrono.ChShellANCF(mesh, material)
my_system.Add(tablecloth)



tablecloth.SetPos(chrono.ChVectorD(0, 0, 0))


ground = chrono.ChBodyEasyBox(2, 2, 0.1, 1000, True, True)
my_system.Add(ground)


my_system.SetSolverType(chrono.ChSolver.Type_MKL_PARDISO)


app = irr.ChIrrApp(my_system, 'Tablecloth folding', irr.dimension2du(1024, 768))
app.AddTypicalLogo()
app.AddTypicalCamera(irr.vector3df(0, 1, -1))
app.AddTypicalLights()
app.AddSkyBox()
app.AddLightWithShadow(irr.vector3df(1.5, 5.5, -2.5), irr.vector3df(0, 0, 0), 3, 2.2, 7.2, 40, 512, irr.SColorf(0.8, 0.8, 1))


app.SetTimestep(0.01)
while app.GetDevice().run():
    app.BeginScene()
    app.DrawAll()
    app.DoStep()
    app.EndScene()