import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as chrono_vehicle
import numpy as np


chrono.SetChronoDataPath('./data/')
system = chrono.ChSystemNSC()


application = chronoirr.ChVisualSystemIrrlicht()
application.SetWindowSize(1024, 768)
application.SetWindowTitle('Gator Vehicle Simulation')
application.Initialize(system)


gator = chrono_vehicle.Gator()
gator.SetContactMethod(chrono.ChContactMethod_NSC)
gator.SetTireModel(chrono_vehicle.TMEASY)
gator.SetChassisVisualization(chrono.ChVisualizationType.MESH)
gator.SetSuspensionVisualization(chrono.ChVisualizationType.MESH)
gator.SetSteeringVisualization(chrono.ChVisualizationType.MESH)
gator.SetWheelVisualization(chrono.ChVisualizationType.MESH)
gator.SetChassisCollisionShape(chrono.ChCollisionShapeType.BOX)
gator.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.5), chrono.ChQuaternionD(1, 0, 0, 0)))
system.Add(gator)


terrain = chrono.ChBodyEasyBox(system, 10, 10, 0.5, 1000, True, True)
terrain.SetBodyFixed(True)
terrain.SetVisualizationType(chrono.ChVisualizationType.MESH)
terrain.SetVisualizationMesh(chrono.GetChronoDataPath() + 'models/box.obj')
terrain.SetTexture(chrono.GetChronoDataPath() + 'models/concrete.jpg')
system.Add(terrain)


gator.SetChassisPosition(chrono.ChVectorD(0, 0, 0.5))
gator.SetChassisOrientation(chrono.ChQuaternionD(1, 0, 0, 0))
gator.SetSteering(0)
gator.SetThrottle(0)
gator.SetBraking(0)


driver = chrono_vehicle.ChIrrlichtDriver()
driver.SetVehicle(gator)
driver.Initialize(application)


time_step = 0.02
application.SetTimestep(time_step)
application.SetTryRealtime(True)


while application.Run():
    application.BeginScene()
    application.DrawAll()
    application.EndScene()
    driver.Synchronize()
    system.DoStepDynamics(time_step)

application.GetSystem().Dump()