import pychrono as chrono
from pychrono.vehicle import HMMWV, ChDriver
from pychrono.irrlicht import IrrlichtApplication, ChTerrain


system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVectorD(0, -9.81, 0))


terrain = ChTerrain()
terrain.LoadMesh('Highway_col.obj', 'Highway_vis.obj')
terrain.SetMaterial(chrono.ChMaterialSurfaceNSC())
system.Add(terrain)


vehicle = HMMWV()
vehicle.SetContactMethod(chrono.ChContactMethodNSC)
vehicle.SetTireModelType(TireModelType.TMEASY)
vehicle.SetTireStepSize(1e-3)
vehicle.Initialize(chrono.ChFrameD(chrono.ChVectorD(0, 0.5, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
vehicle.SetVisualizationType(VisualizationType.MESH)
system.Add(vehicle)


driver = ChDriver(vehicle)
driver.SetThrottle(ChDriver.ThrottleType.MAP)
driver.SetSteering(ChDriver.SteeringType.MAP)
driver.SetBraking(ChDriver.BrakingType.MAP)


application = IrrlichtApplication()
application.SetWindowCaption('HMMWV Simulation')
application.SetWindowSize(1024, 768)
application.SetCameraPosition(chrono.ChVectorD(0, 2, -10))
application.SetCameraTarget(chrono.ChVectorD(0, 0.5, 0))
application.AddTerrain(terrain)
application.AddVehicle(vehicle)


while application.Run():
    application.BeginScene()
    application.Render()
    application.EndScene()
    
    
    driver.Synchronize(chrono.ChTime(0))
    system.DoStepDynamics(1.0 / 50)
    driver.Advance(1.0 / 50)