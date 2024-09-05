import pychrono as chrono
import pychrono.irrlicht as irr


chrono.SetChronoDataPath('PYCHRONO_DATA_DIR')


my_system = chrono.ChSystemNSC()


terrain = chrono.ChBodyEasyBox(10, 1, 10, 1000, True, True)
my_system.Add(terrain)


terrain_material = chrono.ChMaterialSurfaceNSC()
terrain_material.SetFriction(0.9)
terrain_material.SetRestitution(0.01)
terrain.SetMaterialSurface(terrain_material)


m113 = chrono.ChBodyEasyBox(2, 1, 4, 1000, True, True)
m113.SetPos(chrono.ChVectorD(0, 1, 0))
my_system.Add(m113)


driver = chrono.ChDriver(m113)


app = irr.ChIrrApp(my_system, 'M113 Simulation', irr.dimension2du(1024, 768))
app.AddTypicalLogo()
app.AddTypicalSky()
app.AddTypicalLights()
app.AddTypicalCamera(irr.vector3df(0, 3, -6))


while app.GetDevice().run():
    app.BeginScene()
    app.DrawAll()
    app.DoStep()
    app.EndScene()

    
    my_system.DoStepDynamics(0.01)