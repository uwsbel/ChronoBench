import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr


chrono.SetChronoDataPath('/path/to/chrono/data/')


sys = chrono.ChSystemNSC()


ground = chrono.ChBodyEasyBox(sys,  
                              100, 100, 2,  
                              1000,  
                              True,  
                              True,  
                              chrono.ChMaterialSurfaceNSC())  
ground.SetBodyFixed(True)
sys.Add(ground)


vehicle = veh.ChPart('BMW_E90')
vehicle.SetMass(1500)
vehicle.SetInertiaXX(2500)
vehicle.SetInertiaYY(3000)
vehicle.SetInertiaZZ(2500)
sys.Add(vehicle)


tire_model = veh.ChTMEasy()
tire_model.SetTireParam(veh.ChTMEasy.PARAM_TIRE_RADIUS, 0.35)
tire_model.SetTireParam(veh.ChTMEasy.PARAM_TIRE_WIDTH, 0.2)
tire_model.SetTireParam(veh.ChTMEasy.PARAM_TIRE_ASPECT_RATIO, 0.7)
tire_model.SetTireParam(veh.ChTMEasy.PARAM_TIRE_MASS, 20)
vehicle.AddTire(tire_model, veh.ChPartLocation.FRONT_LEFT)
vehicle.AddTire(tire_model, veh.ChPartLocation.FRONT_RIGHT)
vehicle.AddTire(tire_model, veh.ChPartLocation.REAR_LEFT)
vehicle.AddTire(tire_model, veh.ChPartLocation.REAR_RIGHT)


driver = veh.ChIrrNodeDriver(sys)
driver.Initialize()
driver.SetSteeringIncrement(0.01)
driver.SetThrottleIncrement(0.01)
driver.SetBrakingIncrement(0.01)
sys.Add(driver)


app = chronoirr.ChIrrApp(sys, 'BMW E90 Simulation', chronoirr.dimension2du(800, 600))
app.AddTypicalSky()
app.AddTypicalCamera(chronoirr.vector3df(0, 0, 1))
app.AddTypicalLights()
app.SetChaseCamera(vehicle, 2, 2)


terrain = chronoirr.ChIrrNode(ground)
app.GetSceneManager().getRootSceneNode().addChild(terrain)
terrain.SetCustomMesh(chronoirr.GetChronoDataFile('textures/terrain.obj'))
terrain.SetMaterialTexture(0, chronoirr.GetChronoDataFile('textures/terrain_diffuse.png'))
terrain.SetMaterialTexture(1, chronoirr.GetChronoDataFile('textures/terrain_normal.png'))


logo = chronoirr.ChIrrNode(ground)
app.GetSceneManager().getRootSceneNode().addChild(logo)
logo.SetCustomMesh(chronoirr.GetChronoDataFile('textures/logo.obj'))
logo.SetMaterialTexture(0, chronoirr.GetChronoDataFile('textures/logo_diffuse.png'))
logo.SetMaterialTexture(1, chronoirr.GetChronoDataFile('textures/logo_normal.png'))


app.SetTimestep(0.01)
app.SetTryRealtime(True)
while app.GetDevice().run():
    app.BeginScene()
    app.DrawAll()
    app.EndScene()