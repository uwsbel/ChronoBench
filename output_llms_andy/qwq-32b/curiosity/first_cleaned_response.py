import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


application = chronoirr.ChIrrApp(system, 'Curiosity Rover Simulation', chronoirr.dimension2du(1024, 768))
application.AddTypicalSky()
application.AddTypicalLogo()
application.AddTypicalCamera(chronoirr.vector3df(0, 5, 10))  
application.SetSymbolscale(0.01)
application.SetShowInfos(True)
application.SetLightIntensity(0.7)
application.SetShadows(True)


terrain = veh.RigidTerrain(system, chrono.ChMaterialSurfaceNSC(), False,
                          chrono.ChVectorD(0, 1, 0), 1,  
                          20, 20)  
application.AssetBind(terrain.GetGround())
application.AssetUpdate(terrain.GetGround())


vehicle = veh.Jeep()
vehicle.Initialize(terrain.GetGround(), chrono.ChVectorD(0, 0.2, 0), veh.ChJeep_InitData())
application.Add(vehicle.GetVisualization())


material = chrono.ChVisualMaterial()
material.SetKdTexture(chrono.GetChronoDataFile('textures/chronologo.jpg'))
vehicle.GetChassis().GetVisualShape(0).SetMaterial(0, material)


driver = veh.ChDriver()
vehicle.SetDriver(driver)


while application.GetDevice().run():
    time = system.GetChTime()
    
    
    key = application.GetDevice().getKey()
    if key == chronoirr.KEY_KEY_UP:
        driver.SetThrottle(min(driver.GetThrottle() + 0.1, 1.0))
    elif key == chronoirr.KEY_KEY_DOWN:
        driver.SetThrottle(max(driver.GetThrottle() - 0.1, -1.0))
    elif key == chronoirr.KEY_KEY_LEFT:
        driver.SetSteering(min(driver.GetSteering() + 0.1, 1.0))
    elif key == chronoirr.KEY_KEY_RIGHT:
        driver.SetSteering(max(driver.GetSteering() - 0.1, -1.0))
    
    
    driver.Synchronize(time)
    vehicle.Synchronize(time, driver)
    
    
    system.DoStepDynamics(0.01)
    
    
    application.BeginScene()
    application.DrawAll()
    application.EndScene()

import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


application = chronoirr.ChIrrApp(system, 'Curiosity Rover Simulation', chronoirr.dimension2du(1024, 768))
application.AddTypicalSky()
application.AddTypicalLogo()
application.AddTypicalCamera(chronoirr.vector3df(0, 5, 10))  
application.SetSymbolscale(0.01)
application.SetShowInfos(True)
application.SetLightIntensity(0.7)
application.SetShadows(True)


terrain = veh.RigidTerrain(system, chrono.ChMaterialSurfaceNSC(), False,
                          chrono.ChVectorD(0, 1, 0), 1,  
                          20, 20)  
application.AssetBind(terrain.GetGround())
application.AssetUpdate(terrain.GetGround())


vehicle = veh.Jeep()
vehicle.Initialize(terrain.GetGround(), chrono.ChVectorD(0, 0.2, 0), veh.ChJeep_InitData())
application.Add(vehicle.GetVisualization())


material = chrono.ChVisualMaterial()
material.SetKdTexture(chrono.GetChronoDataFile('textures/chronologo.jpg'))
vehicle.GetChassis().GetVisualShape(0).SetMaterial(0, material)


driver = veh.ChDriver()
vehicle.SetDriver(driver)


while application.GetDevice().run():
    time = system.GetChTime()
    
    
    key = application.GetDevice().getKey()
    if key == chronoirr.KEY_KEY_UP:
        driver.SetThrottle(min(driver.GetThrottle() + 0.1, 1.0))
    elif key == chronoirr.KEY_KEY_DOWN:
        driver.SetThrottle(max(driver.GetThrottle() - 0.1, -1.0))
    elif key == chronoirr.KEY_KEY_LEFT:
        driver.SetSteering(min(driver.GetSteering() + 0.1, 1.0))
    elif key == chronoirr.KEY_KEY_RIGHT:
        driver.SetSteering(max(driver.GetSteering() - 0.1, -1.0))
    
    
    driver.Synchronize(time)
    vehicle.Synchronize(time, driver)
    
    
    system.DoStepDynamics(0.01)
    
    
    application.BeginScene()
    application.DrawAll()
    application.EndScene()