import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr


chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


system = chrono.ChSystemNSC()


feda = veh.FEDA(system)
feda.SetContactMethod(chrono.ChMaterialSurface.NSC)
feda.SetTireModel(veh.TireModelType.TMEASY)


feda.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.5)))
feda.SetInitFwdVel(0)


terrain = veh.RigidTerrain(system)
terrain.SetTexture(chrono.GetChronoDataPath() + 'textures/concrete.jpg')
terrain.SetContactFrictionCoefficient(0.9)


application = irr.ChIrrApp(system, 'FEDA Vehicle Simulation', chrono.ChCoreIrrAppAsset(chrono.GetChronoDataPath() + ''))
application.AddTypicalSky()
application.AddTypicalLights()
application.AddTypicalLogo()
application.AddTypicalCamera(chrono.ChVectorD(0, -6, 2), chrono.ChVectorD(0, 0, 1))
application.SetTimestep(1.0 / 50.0)


application.GetActiveCamera().SetFollowMode(irr.ChIrrCamera.FollowMode.FOLLOW_POSITION)
application.GetActiveCamera().SetFollowPos(chrono.ChVectorD(0, 0, 2))
application.GetActiveCamera().SetFollowSpeed(chrono.ChVectorD(0, 5, 0))


feda.GetVehicle().SetMeshVisualizationType(veh.VisualizationType_MESH)


driver = veh.ChIrrGuiDriver(application)


driver.Initialize(feda)


feda.SetDriver(driver)


while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()


application.AssetBindAll()
application.AssetUpdateAll()
application.AddShadowAll()
application.GetSystem().DoFrameDynamics()
application.End()