import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr


print("Copyright (c) 2017 projectchrono.org")


my_hmmwv = veh.HMMWV()


myapplication = chronoirr.ChIrrApp(my_hmmwv.GetSystem(), 'HMMWV with interactive driver', chronoirr.dimension2du(800, 600))


my_hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)
my_hmmwv.SetChassisCollisionType(chrono.ChChassisCollisionType_NONE)
my_hmmwv.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.5), chrono.Q_from_AngX(chrono.CH_C_PI / 2)))
my_hmmwv.SetTireType(veh.TireModelType_TMEASY)
my_hmmwv.SetTireStepSize(0.01)
my_hmmwv.SetInitFwdVel(0.0)


my_hmmwv.Initialize()


terrain = veh.RigidTerrain(my_hmmwv.GetSystem())
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT), 200, 200)
terrain.Initialize()


my_driver = veh.ChIrrGuiDriver()
myapplication.AddTypicalSky()
myapplication.AddTypicalLights()
myapplication.AddCamera(chrono.ChVectorD(0.5, 0.5, 1.5))
myapplication.AddTypicalCameraPanels()


while myapplication.GetDevice().run():
    myapplication.BeginScene()
    myapplication.DrawAll()
    myapplication.DoStep()
    myapplication.EndScene()