import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.tire as tire


chrono.SetChronoDataPath('./chrono_data/')
system = chrono.ChSystemNSC()


veh_hmmwv = veh.HMMWV()
veh_hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
veh_hmmwv.SetChassisCollisionModel(chrono.ChCollisionModelType.AABB)
veh_hmmwv.SetChassisSimplifiedCollisionModel(True)
veh_hmmwv.SetTireModel(tire.TMEASY)
veh_hmmwv.SetTireStepSize(0.01)
veh_hmmwv.SetTireVisualization(True)
veh_hmmwv.SetVehicleVisualization(veh.Visualization_PRIMITIVES)
veh_hmmwv.SetChassisVisualization(veh.Visualization_PRIMITIVES)
veh_hmmwv.SetSteeringVisualization(veh.Visualization_PRIMITIVES)
veh_hmmwv.SetSuspensionVisualization(veh.Visualization_PRIMITIVES)
veh_hmmwv.SetWheelVisualization(veh.Visualization_PRIMITIVES)
veh_hmmwv.SetTireVisualization(True)
veh_hmmwv.SetDriverType(veh.DriverType_INTERACTIVE)
veh_hmmwv.SetSteeringMode(veh.SteeringMode_LOCKEDDIFF)
veh_hmmwv.SetBrakeType(veh.BrakeType_SIMPLEBRAKE)
veh_hmmwv.SetBrakeTorque(1000)
veh_hmmwv.SetThrottleTorque(5000)
veh_hmmwv.SetThrottleMode(veh.ThrottleMode_TORQUE)
veh_hmmwv.SetThrottleInputMode(veh.ThrottleInputMode_ACCELERATOR)
veh_hmmwv.SetBrakeInputMode(veh.BrakeInputMode_BRAKEPEDAL)
veh_hmmwv.SetSteeringInputMode(veh.SteeringInputMode_STEERINGWHEEL)
veh_hmmwv.SetVehicleSpeed(0)
veh_hmmwv.SetVehicleDriveType(veh.VehicleDriveType_REARWHEEL_DRIVE)
veh_hmmwv.SetChassisFixed(False)
veh_hmmwv.SetChassisMass(2000)
veh_hmmwv.SetChassisInertiaXX(chrono.ChVectorD(100, 100, 100))
veh_hmmwv.SetChassisInertiaYY(chrono.ChVectorD(100, 100, 100))
veh_hmmwv.SetChassisInertiaZZ(chrono.ChVectorD(100, 100, 100))
veh_hmmwv.SetChassisPosition(chrono.ChVectorD(0, 0, 0.5))
veh_hmmwv.SetChassisRotation(chrono.Q_from_AngX(chrono.CH_C_PI / 2))
veh_hmmwv.AddToSystem(system)


terrain = veh.RigidTerrain(system)
terrain.SetName("terrain")
terrain.SetTexture(chrono.GetChronoDataFile("terrain/textures/tile4.jpg"))
terrain.SetMaterial(chrono.ChMaterialSurfaceNSC())
terrain.SetPlane(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngX(chrono.CH_C_PI / 2)))
terrain.AddToSystem(system)


application = chronoirr.ChVisualSystemIrrlicht()
application.SetWindowSize(1024, 768)
application.SetWindowTitle("HMMWV on Rigid Terrain")
application.AddTypicalLights()
application.AddSkyBox()
application.AddCamera(chrono.ChVectorD(0, 0, 1.5))
application.AddCamera(chrono.ChVectorD(0, 0, 3.5))
application.SetSymbolicCamera(chrono.ChVectorD(0, 0, 1.5))
application.SetShowLogo(False)
application.SetShowInfo(False)


application.SetTimestep(0.02)
application.SetTryRealtime(True)


while application.Run():
    application.BeginScene()
    application.DrawAll()
    application.EndScene()