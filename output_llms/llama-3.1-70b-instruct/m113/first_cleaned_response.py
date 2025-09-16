import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as chrono_vehicle


chrono.SetChronoDataPath('./data/')


mysystem = chrono.ChSystemNSC()
mysystem.SetSolverType(chrono.ChSolver.Type_Acceleration)
mysystem.SetSolverMaxIterations(100)
mysystem.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT)
dt = 0.01
mysystem.SetDt(dt)


m113 = chrono_vehicle.M113(mysystem)
m113.SetChassisVisualizationType(chrono_vehicle.VisualizationType_PRIMITIVES)
m113.SetSuspensionVisualizationType(chrono_vehicle.VisualizationType_PRIMITIVES)
m113.SetWheelVisualizationType(chrono_vehicle.VisualizationType_PRIMITIVES)
m113.SetChassisCollisionType(chrono_vehicle.CollisionType_NONE)
m113.SetSuspensionCollisionType(chrono_vehicle.CollisionType_NONE)
m113.SetWheelCollisionType(chrono_vehicle.CollisionType_NONE)


m113.SetChassisPosition(chrono.ChVectorD(0, 0.5, 0))
m113.SetChassisOrientation(chrono.Q_from_AngX(0 * chrono.CH_C_DEG_TO_RAD))
m113.SetChassisVelocity(chrono.ChVectorD(0, 0, 0))
m113.SetChassisAngularVelocity(chrono.ChVectorD(0, 0, 0))


terrain = chrono.ChBodyEasyBox(mysystem, 10, 1, 10, 1000, True, True)
terrain.SetPos(chrono.ChVectorD(0, -1, 0))
terrain.SetMaterial(chrono.ChMaterialSurfaceNSC())
terrain.GetMaterial().SetFriction(0.9)
terrain.GetMaterial().SetRestitution(0.1)


driver = chrono_vehicle.ChIrrVehicleDriver(m113.GetVehicle())
driver.Set SteeringDelta(chrono.CH_C_PI / 4)
driver.SetSteeringSpeed(chrono.CH_C_PI / 2)
driver.SetThrottleDelta(1)
driver.SetThrottleSpeed(1)


vis = chronoirr.ChVisualSystemIrrlicht(mysystem)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("M113 Simulation")
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AddCamera(chrono.ChVectorD(0, 1, -3), chrono.ChVectorD(0, 1, 0))
vis.SetCameraUp(chrono.ChVectorD(0, 1, 0))
vis.SetCameraLookAt(chrono.ChVectorD(0, 1, 0))


while vis.Run():
    
    mysystem.Solve(dt)
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()
    vis.DoStep()