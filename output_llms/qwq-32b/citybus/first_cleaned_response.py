import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))


terrain = veh.ChRigidTerrain()
terrain.Initialize(system, veh.ChCollisionModelType_VH)
terrain.SetColor(chrono.ChColor(0.8, 0.8, 0.8))
terrain.SetTexture(chrono.GetChronoDataFile('textures/concrete.jpg'), 200, 200)


citybus = veh.CityBus()
citybus.SetTireModel(veh.TireModelType.RIGID)  
citybus.SetChassisFixed(False)  
citybus.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1), chrono.QUNIT), terrain)


system.Add(citybus.GetChassis())


visualizer = irr.ChIrrApp(system, 'CityBus Simulation', irr.dimension2du(1024, 768))
visualizer.AddTypicalLights()
visualizer.AddTypicalLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
visualizer.AddTypicalSky()


visualizer.AddAsset(citybus.GetVisualization())
visualizer.AddAsset(terrain.GetPlane())


visualizer.SetChaseCamera(chrono.ChVectorD(0, -10, 3), citybus.GetChassis(), 5, 3)


visualizer.SetTimestep(0.02)  


while visualizer.Run():
    
    key = visualizer.GetKeyboard()
    steering = 0
    throttle = 0
    brake = 0
    
    if key.IsKeyDown(irr.KEY_KEY_LEFT):
        steering = -1
    elif key.IsKeyDown(irr.KEY_KEY_RIGHT):
        steering = 1
    
    if key.IsKeyDown(irr.KEY_KEY_UP):
        throttle = 1
    elif key.IsKeyDown(irr.KEY_KEY_DOWN):
        brake = 1
    
    
    citybus.SetDriverInputs(steering, throttle, brake)
    
    
    system.DoStepDynamics(visualizer.GetTimestep())
    
    
    visualizer.Render()