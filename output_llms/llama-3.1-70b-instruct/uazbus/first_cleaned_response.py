import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as chrono_vehicle


chrono.SetChronoDataPath('./data/')


mysystem = chrono.ChSystemNSC()
mysystem.SetSolverType(chrono.ChSolver.Type_PSSOR)


UAZBUS = chrono_vehicle.ChVehicleUAZBUS()
UAZBUS.SetChassisVisualizationType(chrono_vehicle.VisualizationType_PRIMITIVES)
UAZBUS.SetChassisCollisionType(chrono_vehicle.CollisionType_NONE)
UAZBUS.SetSuspensionVisualizationType(chrono_vehicle.VisualizationType_PRIMITIVES)
UAZBUS.SetWheelVisualizationType(chrono_vehicle.VisualizationType_PRIMITIVES)
UAZBUS.SetWheelCollisionType(chrono_vehicle.CollisionType_NONE)
UAZBUS.SetChassisFixed(False)


UAZBUS.SetChassisPosition(chrono.ChVectorD(0, 0.4, 0))
UAZBUS.SetChassisOrientation(chrono.ChQuaternionD(1, 0, 0, 0))
UAZBUS.SetChassisVelocity(chrono.ChVectorD(0, 0, 0))
UAZBUS.SetChassisAcceleration(chrono.ChVectorD(0, 0, 0))
UAZBUS.SetChassisAngularVelocity(chrono.ChVectorD(0, 0, 0))
UAZBUS.SetChassisAngularAcceleration(chrono.ChVectorD(0, 0, 0))


mysystem.Add(UAZBUS.GetChassisBody())
mysystem.Add(UAZBUS.GetSuspension(0))
mysystem.Add(UAZBUS.GetSuspension(1))
mysystem.Add(UAZBUS.GetSuspension(2))
mysystem.Add(UAZBUS.GetSuspension(3))
mysystem.Add(UAZBUS.GetWheel(0))
mysystem.Add(UAZBUS.GetWheel(1))
mysystem.Add(UAZBUS.GetWheel(2))
mysystem.Add(UAZBUS.GetWheel(3))


terrain = chrono.ChBodyEasyBox(mysystem, 10, 10, 1, 1000, True, True)
terrain.SetPos(chrono.ChVectorD(0, -1, 0))
terrain.SetMaterial(chrono.ChMaterialSurfaceNSC())
terrain.GetMaterialSurface().SetFriction(0.9)
terrain.GetMaterialSurface().SetRestitution(0.01)
mysystem.Add(terrain)


driver = chrono_vehicle.ChIrrlichtDriver()
driver.SetVehicle(UAZBUS)
driver.SetSteeringDelta(0.04)
driver.SetThrottleDelta(0.01)
driver.SetBrakingDelta(0.1)
driver.Initialize()


application = chronoirr.ChVisualSystemIrrlicht()
application.SetWindowSize(1024, 768)
application.SetWindowTitle('PyChrono Vehicle Simulation')
application.AddTypicalLights()
application.AddSkyBox()
application.AddCamera(chrono.ChVectorD(0, 2, -3), chrono.ChVectorD(0, 1, 0))
application.SetSymbolicCameraSpeed(0.5)
application.SetShowLogo(False)
application.SetCameraSpeed(0.5)
application.SetCameraUp(chrono.ChVectorD(0, 1, 0))


while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()

    
    mysystem.DoStepDynamics(0.01)
    driver.Synchronize(0.01)
    application.Synchronize(0.01)
    application.DrawAll()

    
    for i in range(4):
        if UAZBUS.GetWheel(i).GetContactForce().Length() > 0:
            print('Collision detected!')