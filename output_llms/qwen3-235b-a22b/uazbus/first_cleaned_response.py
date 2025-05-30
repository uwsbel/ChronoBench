import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr


my_system = chrono.ChSystemNSC()
my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


uaz = veh.UAZBUS_SAE(my_system)
uaz.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0), chrono.QUNIT))
uaz.SetPowertrainType(veh.PowertrainModelType_SIMPLE_MAP)
uaz.SetTireType(veh.TireModelType_TMEASY)
uaz.Initialize()


uaz.SetChassisVisualizationType(veh.VisualizationType_MESH)
uaz.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
uaz.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
uaz.SetWheelVisualizationType(veh.VisualizationType_MESH)


terrain_material = chrono.ChMaterialSurfaceNSC()
terrain_material.SetFriction(0.9)
terrain_material.SetRestitution(0.01)
terrain = veh.RigidTerrain(my_system, terrain_material)
patch = terrain.AddPatch(chrono.ChCoordsysD(), chrono.ChVectorD(200, 1, 200))
patch.SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
terrain.Initialize()


driver = veh.ChIrrGuiDriverIRR(my_system, uaz.GetVehicle(), 10, 10, 0.3, 0.001)
driver.Initialize()


application = chronoirr.ChIrrApp(my_system, 'UAZBUS Simulation', chronoirr.dimension2du(1024, 768))
application.AddTypicalLogo()
application.AddTypicalCamera(chronoirr.vector3df(0, 3, -6), chronoirr.vector3df(0, 0, 0))
application.AddTypicalLights()
application.AssetBindAll()
application.AssetUpdateAll()


application.SetStepManage(True)
application.SetTimestep(1e-3)  
application.SetTryRealtime(True)  


while application.GetDevice().run():
    time = my_system.GetChTime()
    
    
    driver_inputs = driver.GetInputs()
    uaz.Synchronize(time, driver_inputs)
    terrain.Synchronize(time)
    
    
    application.BeginScene()
    application.DrawAll()
    application.EndScene()
    
    
    application.DoStep()