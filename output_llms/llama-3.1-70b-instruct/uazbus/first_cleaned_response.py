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
UAZBUS.SetSteeringVisualizationType(chrono_vehicle.VisualizationType_PRIMITIVES)
UAZBUS.SetWheelVisualizationType(chrono_vehicle.VisualizationType_PRIMITIVES)


UAZBUS.SetChassisPosition(chrono.ChVectorD(0, 0.5, 0))
UAZBUS.SetChassisOrientation(chrono.Q_from_AngX(chrono.CH_C_PI_2))


mysystem.Add(UAZBUS.GetPart(chrono_vehicle.ChPartID_CHASSIS))


terrain = chrono.ChBodyEasyBox(mysystem, 10, 1, 10, 1000, True, True)
terrain.SetPos(chrono.ChVectorD(0, -1, 0))
terrain.SetMaterial(chrono.ChMaterialSurfaceNSC())
terrain.GetMaterialSurface().SetFriction(0.9)
terrain.GetMaterialSurface().SetRestitution(0.01)


mysystem.Add(terrain)


driver = chrono_vehicle.ChIrrlichtDriverUAZBUS(UAZBUS)
driver.SetZSpeed(1.0)
driver.SetSteering(0.0)


application = chronoirr.ChVisualSystemIrrlicht()
application.SetWindowSize(1024, 768)
application.AddTypicalLights()
application.AddSkyBox()
application.AddLogo('logo_pychrono_alpha.png')
application.SetShowFPS(True)
application.SetCamera(chrono.ChVectorD(0, 1, -3), chrono.ChVectorD(0, 1, 0), chrono.ChVectorD(0, 1, 0))


camera = application.AddCamera(chrono.ChVectorD(0, 1, -3))
camera.SetCameraUp(chrono.ChVectorD(0, 1, 0))
camera.SetCameraLookAt(chrono.ChVectorD(0, 1, 0))


while application.Run():
    
    application.BeginScene()
    application.DrawAll()
    application.EndScene()

    
    mysystem.DoStepDynamics(0.01)

    
    driver.Advance(0.01)

    
    application.DoStep()