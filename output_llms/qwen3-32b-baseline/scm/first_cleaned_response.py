import pychrono as chrono
import pychrono.vehicle as vehicle
import pychrono.irr as chronoirr


my_system = chrono.ChSystemNSC()
my_system.Set_Gravity(chrono.ChVectorD(0, -9.81, 0))


hmmwv_vehicle = vehicle.HMMWV_Vehicle(my_system)
hmmwv_vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1), chrono.ChQuaternionD(1, 0, 0, 0)))
hmmwv_vehicle.SetChassisVisualizationType(vehicle.VisualizationType_MESH)
hmmwv_vehicle.SetSuspensionVisualizationType(vehicle.VisualizationType_MESH)
hmmwv_vehicle.SetSteeringVisualizationType(vehicle.VisualizationType_MESH)


tire_FL = vehicle.HMMWV_RigidTire()
tire_FR = vehicle.HMMWV_RigidTire()
tire_RL = vehicle.HMMWV_RigidTire()
tire_RR = vehicle.HMMWV_RigidTire()

tire_FL.Initialize(hmmwv_vehicle.GetFrontLeftWheel())
tire_FR.Initialize(hmmwv_vehicle.GetFrontRightWheel())
tire_RL.Initialize(hmmwv_vehicle.GetRearLeftWheel())
tire_RR.Initialize(hmmwv_vehicle.GetRearRightWheel())

tire_FL.SetVisualizationType(vehicle.VisualizationType_MESH)
tire_FR.SetVisualizationType(vehicle.VisualizationType_MESH)
tire_RL.SetVisualizationType(vehicle.VisualizationType_MESH)
tire_RR.SetVisualizationType(vehicle.VisualizationType_MESH)

hmmwv_vehicle.Initialize()


soil_parameters = vehicle.ChSoilParametersSCM()
soil_parameters.SetCohesion(0.5)  
soil_parameters.SetFriction(0.4)  
soil_parameters.SetDamping(5.0)   
soil_parameters.SetYoungModulus(2e6)  

scm_terrain = vehicle.SCMDeformableTerrain(my_system, soil_parameters)
scm_terrain.SetMovingPatch(True, hmmwv_vehicle.GetChassisBody(), 10, 10)  
scm_terrain.SetPlotSINK(True)  


application = chronoirr.ChIrrApp(my_system, 'HMMWV on SCM Deformable Terrain', chronoirr.dimension2du(800, 600))
application.SetSkyBox()
application.SetCamera(chronoirr.vector3df(0, 2, -6))
application.SetLightDirection(chronoirr.vector3df(0, -1, 0))
application.AddLogo()
application.AddTypicalLights()

driver = vehicle.ChInteractiveDriverIRR(hmmwv_vehicle.GetChassisBody(), application)
driver.SetSteeringClamp(1.0)
driver.SetThrottleClamp(1.0)
driver.SetBrakingClamp(1.0)


time_step = 1 / 50  
application.SetStepManage(True)
application.SetRealTimeUpdate(True, time_step)


while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    driver.Update()
    my_system.DoStepDynamics(time_step)
    application.EndScene()