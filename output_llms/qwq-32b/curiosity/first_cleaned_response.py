import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr


sys = chrono.ChSystemNSC()
sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


ground = chrono.ChBodyEasyBox(20, 1, 20, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -1, 0))
ground.SetBodyFixed(True)
ground.GetCollisionModel().SetFamily(chrono.CHCollisionFamily_GROUND)
ground.GetCollisionModel().SetFamilyMaskNoCollisionWithFamily(chrono.CHCollisionFamily_VEHICLE)
sys.Add(ground)


ground_vis = chrono.ChVisualShapeBox(20, 1, 20)
ground_vis.SetTexture(chrono.GetChronoDataFile('textures/mars_ground.jpg'), 20, 20)
ground.AddVisualShape(ground_vis)


my_vehicle = veh.ChRover(sys)
my_vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
my_vehicle.SetWheelVisualizationType(veh.VisualizationType_PRIMITIVES)
my_vehicle.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0), chrono.QUNIT))


driver = veh.ChDriver()
my_vehicle.InitializeDriver(driver)


vis = irr.ChIrrApp(sys, 'Curiosity Rover Simulation', irr.dimension2du(1024, 768))
vis.AddLightWithDir(chrono.ChVectorD(0, 10, 0), chrono.ChVectorD(0, -1, 0), 200, 255, 255, 255, 1)
vis.AddAllTypicalLights()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'), 0.1, 0.0, 0.0)
vis.SetCamera(chrono.ChVectorD(0, 5, 10), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalCameraCallbacks()


vis.GetDevice().getSceneManager().setShadowColor(chrono.ChColor(0.0, 0.0, 0.0, 0.5))
vis.GetDevice().getSceneManager().setShadowEnable(True)


vis.Add(my_vehicle.GetVehicle())


step_size = 0.01


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    
    key = vis.GetDevice().getKeyInput()
    driver_inputs = driver.GetInputs()
    
    if key.isKeyDown(irr.KEY_RIGHT):
        driver_inputs.m_steering += 0.01
    elif key.isKeyDown(irr.KEY_LEFT):
        driver_inputs.m_steering -= 0.01
    if key.isKeyDown(irr.KEY_W):
        driver_inputs.m_throttle += 0.01
    elif key.isKeyDown(irr.KEY_S):
        driver_inputs.m_throttle -= 0.01
    
    
    driver_inputs.m_steering = max(-1, min(1, driver_inputs.m_steering))
    driver_inputs.m_throttle = max(-1, min(1, driver_inputs.m_throttle))
    
    driver.SetInputs(driver_inputs)
    
    my_vehicle.UpdateDriver()
    sys.DoStepDynamics(step_size)