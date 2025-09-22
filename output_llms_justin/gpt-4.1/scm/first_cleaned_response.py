import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math




chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')





initLoc = chrono.ChVectorD(0, 0, 1.0)  
initYaw = chrono.CH_C_PI / 6            


vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(chrono.ChContactMethod_SMC)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysD(initLoc, chrono.Q_from_AngZ(initYaw)))
vehicle.SetPowertrainType(veh.PowertrainModelType_SHAFTS)
vehicle.SetDriveType(veh.DrivelineTypeWV_AWD)
vehicle.SetTireType(veh.TireModelType_RIGID)
vehicle.SetTireStepSize(1e-3)
vehicle.Initialize()


vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_MESH)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)




terrain = veh.SCMDeformableTerrain(vehicle.GetSystem())


terrain.SetPlane(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT))
terrain.SetMeshResolution(0.04)  


terrain.SetSoilParameters(2e6,   
                          0,     
                          1.1,   
                          0,     
                          30,    
                          0.01,  
                          4e7,   
                          3e4)   


terrain.AddMovingPatch(vehicle.GetChassis(), chrono.ChVectorD(0, 0, 0), 5, 3)  


terrain.SetPlotType(veh.SCMDeformableTerrain.PLOT_SINKAGE, 0, 0.1)  




app = veh.ChWheeledVehicleIrrApp(vehicle, 'HMMWV on SCM Terrain', irr.dimension2du(1024,768))
app.SetSkyBox()
app.AddTypicalLights()
app.AddTypicalLogo()
app.AddTypicalCamera(chrono.ChVectorD(0, 2, 1.75), chrono.ChVectorD(0, 0, 0))
app.SetChaseCamera(chrono.ChVectorD(0.0, 0.0, 1.75), 6.0, 0.5)
app.SetTimestep(0.01)
app.AssetBindAll()
app.AssetUpdateAll()




driver = veh.ChIrrGuiDriver(app)
driver.SetSteeringDelta(1.0 / 50)   
driver.SetThrottleDelta(1.0 / 50)   
driver.SetBrakingDelta(1.0 / 50)    
driver.Initialize()




step_size = 0.01
render_steps = int(math.ceil(1.0 / (step_size * 50)))  

while app.GetDevice().run():
    app.BeginScene()
    app.DrawAll()
    app.EndScene()

    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(vehicle.GetSystem().GetChTime())
    terrain.Synchronize(vehicle.GetSystem().GetChTime())
    vehicle.Synchronize(vehicle.GetSystem().GetChTime(), driver_inputs, terrain)
    app.Synchronize('HMMWV on SCM Terrain', driver_inputs)

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    app.Advance(step_size)

    
    if vehicle.GetSystem().GetChTime() % (1.0 / 50) < step_size:
        pass