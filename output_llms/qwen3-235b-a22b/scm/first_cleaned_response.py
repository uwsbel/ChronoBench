import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import pychrono.driver as driver


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.001)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.001)


vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisFixed(False)

vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 1.0, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
vehicle.SetTireType(veh.TireModelType_RIGID)
vehicle.SetTireStepSize(0.02)
vehicle.SetMeshVisualization(True)  
vehicle.Initialize()


terrain = veh.SCMTerrain(system)
terrain.SetSoilParameters(2e6,   
                          1.1,   
                          0,     
                          0.5,   
                          1e-3,  
                          2000)  

terrain.EnableMovingPatch(vehicle.GetChassisBody(), chrono.ChVectorD(10, 4, 4))  
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE)  
terrain.Initialize()


application = irr.ChIrrApp(system, 'HMMWV on SCM Terrain', irr.dimension2du(1280, 720))
application.AddTypicalLights()
application.AddCamera(chrono.ChVectorD(0, 3, -6), chrono.ChVectorD(0, 0, 0))
application.AssetBindAll()
application.AssetUpdateAll()


drv = veh.ChIrrGuiDriver(vehicle.GetVehicle())
drv.SetSteeringDelta(0.05)
drv.SetThrottleDelta(0.05)
drv.SetBrakingDelta(0.05)
application.SetDriver(drv)


system.SetStep(0.02)  

while application.GetDevice().run():
    
    drv.Synchronize(application.GetTime())
    
    
    vehicle.Synchronize(application.GetTime(), drv.GetInputs(), terrain)
    terrain.Synchronize(application.GetTime())
    
    
    system.DoStepDynamics(0.02)
    
    
    application.BeginScene()
    application.DrawAll()
    application.EndScene()