import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as chrono_vehicle


chrono.SetChronoDataPath('./data/')
system = chrono.ChSystemNSC()


vehicle = chrono_vehicle.ArticulatedVehicle(system)
vehicle.SetContactMethod(chrono_vehicle.ContactMethod_ELLIPSOID)
vehicle.SetChassisVisualization(chrono_vehicle.VisualizationType_PRIMITIVES)
vehicle.SetSuspensionVisualization(chrono_vehicle.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualization(chrono_vehicle.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualization(chrono_vehicle.VisualizationType_PRIMITIVES)


vehicle.SetChassisPosition(chrono.ChVectorD(0, 0.5, 0))
vehicle.SetChassisOrientation(chrono.ChQuaternionD(1, 0, 0, 0))


terrain = chrono.ChBodyEasyRigidBox(system, 10, 10, 0.5, 1000, True, True)
terrain.SetPos(chrono.ChVectorD(0, -0.5, 0))
terrain.SetMaterial(chrono.ChMaterialSurfaceNSC())


texture = chrono.ChTexture()
texture.SetTextureFilename(chrono.GetChronoDataPath() + 'textures/concrete.jpg')
terrain.AddVisualization(texture)


driver = chrono_vehicle.ChIrrlichtDriver(vehicle)


step_size = 0.01
time_end = 10
visualizer = chronoirr.ChVisualSystemIrrlicht()
visualizer.SetWindowSize(1024, 768)
visualizer.SetWindowTitle('ARTcar Vehicle Simulation')
visualizer.Initialize(system)

while visualizer.Run():
    vehicle.Synchronize(chrono.ChTime(0, 0.01))
    vehicle.Advance(step_size)
    visualizer.BeginScene()
    visualizer.DrawAll()
    visualizer.EndScene()
    chrono.ChUtils::Sleep(0.02)


system.Remove(vehicle)
system.Remove(terrain)