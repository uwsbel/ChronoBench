import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import numpy as np


my_system = chrono.ChSystemNSC()
my_vehicle = veh.ChVehicle()
my_vehicle.Initialize(my_system)


vehicle_model = veh.HMMWV()
vehicle_model.SetContactMaterial(veh.ChMaterialSurfaceNSC(0.9, 0.5, 0.0001))
vehicle_model.SetChassisCollisionType(veh.ChVehicleCollisionType.NONE)
vehicle_model.SetTireType(veh.ChVehicleTire.TYPE_TMEASY)
vehicle_model.SetTireStepSize(0.01)


my_vehicle.SetChassisFixed(False)
my_vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.5), chrono.Q_from_AngZ(chrono.CH_C_PI_2)))


terrain = veh.SCMDeformableTerrain(my_system, 20, 20, 0.1)
terrain.SetPlasticFoundation(0.001)
terrain.SetPlasticYield(0.1)
terrain.SetPlasticHardening(0.1)
terrain.SetElasticity(0.01)
terrain.SetFriction(0.9)
terrain.SetRestitution(0.05)
terrain.SetDampingF(0.2)
terrain.SetDampingD(0.8)
terrain.SetNearFar(1, 10)
terrain.Initialize(vehicle_model)


height_map = np.zeros((20, 20))
terrain.SetHeightMap(height_map)


driver = veh.ChPathFollowerDriver(my_vehicle, 0.05, 1)
driver.SetTargetSpeed(20)


my_vehicle.SetContactMethod(veh.ChVehicle::CONTACT_METHOD_LINEAR)
my_vehicle.SetChassisFixed(False)
my_vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.5), chrono.Q_from_AngZ(chrono.CH_C_PI_2)))
my_vehicle.SetChassisVisualizationType(veh.ChVehicleVisualizationType.PRIMITIVES)


visual_system = chronoirr.ChIrrApp(my_system, 'PyChrono HMMWV Simulation', chronoirr.dimension2du(800, 600))
visual_system.AddTypicalSky()
visual_system.AddTypicalLogo(chronoirr.vector3df(10, 0, 0))
visual_system.AddTypicalCamera(chronoirr.vector3df(0, 0, 5), chronoirr.vector3df(0, 0, 0))
visual_system.AddLightWithShadow(chronoirr.vector3df(10, 10, 10), chronoirr.vector3df(0, 0, 0), 10, 10, 50, 200, 250, 250, 250)


while visual_system.GetDevice().run():
    visual_system.BeginScene()
    visual_system.DrawAll()
    visual_system.DoStepDynamics(0.01)
    visual_system.EndScene()

visual_system.GetDevice().closeDevice()