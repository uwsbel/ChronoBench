import math
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.terrain as terrain
import pychrono.irrlicht as irr


system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))


vehicle = veh.HMMWV_Full(system)
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0.5, 0), chrono.ChQuaterniond(1, 0, 0, 0)))
vehicle.SetPowertrainType(veh.PowertrainModelType_SHAFTS)
vehicle.SetTireType(veh.TireModelType_TMEASY)
vehicle.SetTireStepSize(1e-3)
vehicle.Initialize()


scm_terrain = terrain.SCMDeformableTerrain(system)
scm_terrain.SetSoilParameters(
    2e6,    
    0.3,    
    30,     
    1000,   
    10,     
    0.01,   
    0.01    
)


def HeightFunction(x, y):
    return 0.05 * math.sin(x * 0.5)


scm_terrain.Initialize(HeightFunction, 100, 100, 20, 20)


application = irr.ChIrrApp(system, 'HMMWV Simulation', irr.dimension2du(1024, 768))
application.AddTypicalLights()
application.SetShowInfos(True)
application.SetStepManage(True)
application.SetTimestep(1e-3)
application.SetTryRealtime(True)


application.AddCamera(chrono.ChVector3d(0, 3, -6), chrono.ChVector3d(0, 0, 0))


driver = irr.ChIrrGuiDriver(vehicle, application.GetDevice(), 0.05, 0.05)
driver.Initialize()


while application.GetDevice().run():
    time = system.GetChTime()
    step = application.GetTimestep()

    
    driver.Synchronize(time)
    scm_terrain.Synchronize(time)
    vehicle.Synchronize(time, driver.GetInputs(), scm_terrain)
    application.Synchronize('HMMWV Simulation', driver.GetInputs())

    
    driver.Advance(step)
    scm_terrain.Advance(step)
    vehicle.Advance(step)
    application.Advance(step)

    
    system.DoStepDynamics(step)

    
    application.BeginScene()
    application.DrawAll()
    application.EndScene()