import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import os


chrono.SetChronoDataPath(os.path.dirname(os.path.realpath(__file__)) + '/data/')
my_system = chrono.ChSystemNSC()


hmmwv = veh.HMMWV(my_system)
hmmwv.SetChassisFixed(False)
hmmwv.SetChassisCollisionType(chrono.ChCollisionModel.Type_MESH)
hmmwv.SetChassisVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetTireType(veh.TireModelType_TMEASY)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireStepSize(0.05)
hmmwv.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1), chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0))))


terrain_shape = chrono.ChCollisionShape()
terrain_shape.AddFile('Highway_col.obj')
terrain_body = chrono.ChBodyEasyMesh(terrain_shape, 1000, False, True)
terrain_body.SetPos(chrono.ChVectorD(0, 0, 0))
terrain_body.SetBodyFixed(True)
my_system.Add(terrain_body)


application = irr.ChIrrApp(my_system, 'HMMWV on Custom Terrain', irr.dimension2du(1024, 768))
application.AddTypicalLogo()
application.AddTypicalCamera(irr.vector3df(0, 3, -5))
application.AddTypicalLights()
application.AddLightWithShadow(irr.vector3df(2, 6, -2), irr.vector3df(0, 0, 0), 12, 2, 10, 40, 512)
application.AddSkyBox()
application.AddShadowAll()
application.AssetBindAll()
application.AssetUpdateAll()


driver = veh.ChDriver(hmmwv.GetVehicle(), veh.ChDriver.Type_ANALYTIC)
driver.Initialize()
driver.SetSteeringControllerType(veh.ChDriver.Steering_ANALYTIC)
driver.SetThrottleControllerType(veh.ChDriver.Throttle_ANALYTIC)
driver.SetBrakingControllerType(veh.ChDriver.Braking_ANALYTIC)


while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    driver.Synchronize(my_system.GetChTime())
    application.EndScene()
    my_system.DoStepDynamics(1e-3)