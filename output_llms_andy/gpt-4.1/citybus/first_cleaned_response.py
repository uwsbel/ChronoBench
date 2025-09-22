import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math


chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(veh.GetDataPath())


system = chrono.ChSystemSMC()


initLoc = chrono.ChVectorD(0, 0, 1.0)  
initRot = chrono.Q_from_AngY(math.radians(15))  
tire_model = veh.TireModelType.TMEASY

citybus = veh.CityBus()
citybus.SetContactMethod(chrono.ChContactMethod_SMC)
citybus.SetChassisFixed(False)
citybus.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot))
citybus.SetTireType(tire_model)
citybus.SetTireStepSize(1e-3)
citybus.Initialize()


citybus.SetChassisVisualizationType(veh.VisualizationType.MESH)  
citybus.SetSuspensionVisualizationType(veh.VisualizationType.PRIMITIVES)
citybus.SetSteeringVisualizationType(veh.VisualizationType.PRIMITIVES)
citybus.SetWheelVisualizationType(veh.VisualizationType.MESH)    
citybus.SetTireVisualizationType(veh.VisualizationType.PRIMITIVES)


terrain = veh.RigidTerrain(system)
patch = terrain.AddPatch(
    chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT),
    length=200.0, width=100.0
)
patch.SetContactFrictionCoefficient(0.9)
patch.SetRestitutionCoefficient(0.01)
patch.SetMaterialSurface(chrono.ChMaterialSurfaceSMC())
patch.SetTexture(chrono.GetChronoDataFile("terrain/textures/tile4.jpg"), 200, 100)
terrain.Initialize()


app = veh.ChVehicleIrrApp(
    citybus, "PyChrono CityBus Demo", irr.dimension2du(1024, 768)
)
app.SetSkyBox()
app.AddTypicalLights()
app.AddTypicalLogo()
app.AddLightWithShadow(chrono.ChVectorF(50, 50, 100), chrono.ChVectorF(0, 0, 0), 250, 1, 60, 60, 1)
app.SetChaseCamera(
    chrono.ChVectorD(0.0, 0.0, 1.75),  
    6.0,  
    0.5   
)
app.SetTimestep(0.02)  


driver = veh.ChIrrGuiDriver(app)
driver.SetSteeringDelta(0.04)  
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.1)
driver.Initialize()


step_size = 0.02  
render_steps = 1

while app.GetDevice().run():
    time = system.GetChTime()

    
    driver.Synchronize(time)
    
    citybus.Synchronize(time, driver.GetInputs(), terrain)
    terrain.Synchronize(time)

    
    driver.Advance(step_size)
    citybus.Advance(step_size)
    terrain.Advance(step_size)

    
    if app.IsRunning():
        app.BeginScene()
        app.DrawAll()
        app.EndScene()
    else:
        break