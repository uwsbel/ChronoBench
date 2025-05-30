import chrono
import chrono.vehicle as veh
import chrono.sensor as sens
import chrono.irrlicht as irr




system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))  




terrain = veh.RigidTerrain(system)
patch = terrain.AddPatch(chrono.ChVectorD(0, 0, 0),  
                         chrono.ChVectorD(0, 0, 1),  
                         400, 400)                   
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4_riverside.png"),
                 40, 40)
patch.SetFriction(0.9)
terrain.Initialize()


vehicle = veh.Man_10t()  
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisCollisionType(veh.ChassisCollisionType_NONE)  
vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.6), chrono.ChQuaternionD(1, 0, 0, 0)))
vehicle.SetTireType(veh.VehicleTire.TMEASY)
vehicle.SetTireStepSize(2e-3)
vehicle.Initialize()


vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)


for i in range(vehicle.GetNumberAxles()):
    for side in [veh.LEFT, veh.RIGHT]:
        tire = veh.TMEASYTire(vehicle.GetSystem(), veh.VehicleSide(side))
        tire.SetStepsize(2e-3)
        
        wheel = vehicle.GetWheel(i, side)
        tire.Initialize(wheel, veh.VisualizationType_MESH)
        vehicle.SetTire(i, side, tire)


driver = veh.ChDriver(vehicle, irr.ChIrrGuiDriver())
driver.SetLookAheadDistance(10)
driver.SetThrottleDelta(0.05)
driver.SetBrakeDelta(0.1)
driver.SetSteeringDelta(0.1)
driver.Initialize()


application = veh.ChIrrApp(vehicle.GetSystem(), "MAN 10t Truck on Rigid Terrain", veh.GetDataFile('terrain/meshes/terrain.obj'))

application.AddTypicalLights()
application.SetChaseCamera(chrono.ChVectorD(0.0, 0.0, 1.5), 6.0, 0.5)  


light = application.AddLightDirectional()
light.Direction = chrono.ChVectorF(-0.5, 0.3, -1)
light.DiffuseColor = chrono.ChColorF(0.9, 0.9, 0.7)
light.SpecularColor = chrono.ChColorF(0.9, 0.9, 0.7)
light.AmbientColor = chrono.ChColorF(0.3, 0.3, 0.3)

application.AddSkyBox()
application.AddLogo()  




application.AssetBindAll()
application.AssetUpdateAll()


timestep = 1e-3
application.SetTimestep(timestep)


application.SetTimestep(timestep)
application.SetTryRealtime(True)

while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()

    
    driver_inputs = driver.GetInputs()
    vehicle.SetDriverInputs(driver_inputs)

    
    time = vehicle.GetSystem().GetChTime()
    driver.Synchronize(time)
    vehicle.Synchronize(time)
    terrain.Synchronize(time)
    application.Synchronize(time)

    
    driver.Advance(timestep)
    vehicle.Advance(timestep)
    terrain.Advance(timestep)
    system.DoStepDynamics(timestep)

    application.EndScene()