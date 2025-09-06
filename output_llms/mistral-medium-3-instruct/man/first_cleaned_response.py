import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.vehicle.terrain as veh_terrain
import pychrono.vehicle.wheeled_vehicle as veh_wheeled
import numpy as np






step_size = 1e-3


tend = 20


camera_target = chrono.ChVectorD(0, 0, 1.75)






vehicle = veh_wheeled.WheeledVehicle(veh.GetDataPath() + "vehicles/man/")


vehicle.SetChassisFixed(False)
vehicle.SetChassisCollide(True)
vehicle.SetVisualizationType(veh.VisualizationType_MESH)
vehicle.SetContactMethod(veh.ContactMethod_SMC)


vehicle.SetSteeringLimits(chrono.ChVector2D(-0.5, 0.5))  


vehicle.GetEngine().SetGearRatios([4.0, 2.5, 1.67, 1.25, 1.0, 0.8])
vehicle.GetEngine().SetGearRatio(1)  
vehicle.GetEngine().SetMaxRPM(6000)
vehicle.GetEngine().SetMinRPM(800)
vehicle.GetEngine().SetMaxTorque(1000)


vehicle.GetBrake(0).SetMaxBrakingTorque(2000)
vehicle.GetBrake(1).SetMaxBrakingTorque(2000)
vehicle.GetBrake(2).SetMaxBrakingTorque(2000)
vehicle.GetBrake(3).SetMaxBrakingTorque(2000)


vehicle.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.5), chrono.QUNIT))






terrain = veh_terrain.RigidTerrain(vehicle.GetSystem())
terrain.SetContactFrictionCoefficient(0.8)
terrain.SetContactRestitution(0.1)
terrain.SetContactMaterialProperties(2e7, 0.3)


patch = terrain.AddPatch(chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(20, 20, 0))
patch.SetTexture(veh.GetDataPath() + "terrain/textures/grassy.png", 20, 20)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.8))






tire_FL = veh.TMeasyTire("FL")
tire_FR = veh.TMeasyTire("FR")
tire_RL = veh.TMeasyTire("RL")
tire_RR = veh.TMeasyTire("RR")


tire_FL.SetTireType(veh.TMeasyTire::TireModelType::TMEASY)
tire_FL.SetUnloadedRadius(0.5)
tire_FL.SetWidth(0.3)
tire_FL.SetCarryingStiffness(200000)
tire_FL.SetDampingRatio(0.05)
tire_FL.SetFrictionCoefficient(0.8)
tire_FL.SetRollingResistance(0.01)


tire_FR.SetFrom(tire_FL)
tire_RL.SetFrom(tire_FL)
tire_RR.SetFrom(tire_FL)


vehicle.AddTire(tire_FL, chrono.ChVectorD(1.5, 1.0, 0.5), chrono.ChVectorD(0, 1, 0))
vehicle.AddTire(tire_FR, chrono.ChVectorD(1.5, -1.0, 0.5), chrono.ChVectorD(0, 1, 0))
vehicle.AddTire(tire_RL, chrono.ChVectorD(-1.5, 1.0, 0.5), chrono.ChVectorD(0, 1, 0))
vehicle.AddTire(tire_RR, chrono.ChVectorD(-1.5, -1.0, 0.5), chrono.ChVectorD(0, 1, 0))






driver = veh.ChDriver(vehicle)


driver.SetSteeringDelta(0.1)
driver.SetThrottleDelta(0.1)
driver.SetBrakingDelta(0.1)






vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachVehicle(vehicle)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("MAN 10t Truck Simulation")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataPath() + "logo_pychrono_alpha.png")
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(5, 0, 2), chrono.ChVectorD(0, 0, 0))
vis.SetCameraTracking(chrono.ChVectorD(0, 0, 1.75), chrono.ChVectorD(5, 0, 0), 0.5)


vis.SetLightDirection(chrono.ChVectorD(1, -1, 1))
vis.SetLightIntensity(1.0)






time = 0


num_steps = int(chrono.ChMath.Ceil(tend / step_size))


while vis.Run() and time < tend:
    
    time = vehicle.GetSystem().GetChTime()

    
    steering = 0.0
    throttle = 0.0
    braking = 0.0

    
    if time > 2:
        steering = 0.2 * np.sin(time * 0.5)

    
    if time > 1:
        throttle = 0.5

    
    driver.SetSteering(steering)
    driver.SetThrottle(throttle)
    driver.SetBraking(braking)

    
    driver.Synchronize(time)
    vehicle.Synchronize(time, steering, throttle, braking)

    
    vehicle.Advance(step_size)

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    if chrono.ChMath.Mod(time, 1) < step_size:
        print("Time: {:.2f} s".format(time))
        print("  Speed: {:.2f} m/s".format(vehicle.GetSpeed()))
        print("  Engine RPM: {:.0f}".format(vehicle.GetEngine().GetCurrentRPM()))
        print("  Gear: {}".format(vehicle.GetEngine().GetCurrentGear()))
        print("  Steering: {:.2f}".format(steering))
        print("  Throttle: {:.2f}".format(throttle))
        print("  Braking: {:.2f}".format(braking))
        print("---------------------------------")