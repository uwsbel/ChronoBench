import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.postprocess as postprocess
import os


chrono.SetChronoDataPath(os.getenv("CHRONO_DATA"))
chrono.Initialize()


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


visualization = chronoirr.ChVisualSystemIrrlicht()
visualization.AttachSystem(system)
visualization.SetWindowSize(1280, 720)
visualization.SetWindowTitle('Gator Vehicle Simulation')
visualization.Initialize()
visualization.AddTypicalSky()
visualization.AddTypicalLights()
visualization.AddLogo()
visualization.EnableShadows()


terrain = chrono.ChRigidTerrain()
terrain.SetPlane(chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 1, 0))
terrain.SetTexture(chrono.GetChronoDataFile('terrain/texture/grass.jpg'), 200, 200)
terrain.SetContactMaterial(0.9, 0.9, 0.5)
system.Add(terrain)


vehicle = chrono.ChVehicle()
vehicle.SetChassisBody(chrono.ChBodyEasyBox(1.5, 1.0, 0.5, 100, True, True, chrono.ChMaterialSurfaceNSC()))
vehicle.Initialize()


vehicle.GetChassisBody().SetPos(chrono.ChVectorD(0, 0.5, 0))
vehicle.GetChassisBody().SetRot(chrono.Q_from_AngAxis(chrono.CH_C_PI / 2, chrono.ChVectorD(0, 0, 1)))


vehicle.SetContactMethod(chrono.ChVehicle::ChContactMethod::NSC)


tire_model = chrono.ChTireHandlerTMeasy()
tire_model.SetTMeasyTireShapeFile(chrono.GetChronoDataFile('vehicle/tire/tmeasy/tread_terrain_truck.dat'))
tire_model.SetTMeasyTireParametersFile(chrono.GetChronoDataFile('vehicle/tire/tmeasy/parameters_truck.dat'))
vehicle.SetTireHandler(tire_model)


wheel_radius = 0.35
wheel_width = 0.2
wheel_mass = 20
wheel_inertia = chrono.ChVectorD(0.25, 0.25, 0.25)
wheel_material = chrono.ChMaterialSurfaceNSC(0.7, 0.3, 0.1, 0.001, 1.1, 1.1, 0.01)

for i in range(4):
    wheel = chrono.ChWheelBody()
    wheel.SetRadius(wheel_radius)
    wheel.SetWidth(wheel_width)
    wheel.SetMass(wheel_mass)
    wheel.SetInertia(wheel_inertia)
    wheel.SetMaterialSurface(wheel_material)
    wheel.SetChassisBody(vehicle.GetChassisBody())
    vehicle.AddWheel(wheel)


driver = chrono.ChDriver()
driver.Initialize(vehicle)
driver.SetSteering(0)
driver.SetThrottle(0)
driver.SetBraking(0)


run_simulation = True
while visualization.Run() and run_simulation:
    
    driver.SetSteering(chrono.CH_C_PI / 4)  
    driver.SetThrottle(0.5)  
    driver.SetBraking(0)  

    
    vehicle.Update(chrono.ChStreamOutAsciiFile('VEHICLE_POSITION.txt'))

    
    system.DoStepDynamics(1.0 / 50.0)  


visualization.Close()