import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr


my_system = chrono.ChSystemNSC()
my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


visualizor = chronoirr.ChVisualSystemIrrlicht()
visualizor.SetWindowSize(1280, 1024)
visualizor.SetWindowTitle('HMMWV on Deformable Terrain')
visualizor.AttachSystem(my_system)
visualizor.SetCameraPosition(chrono.ChVectorD(0, 5, 15))
visualizor.SetCameraLookAt(chrono.ChVectorD(0, 0, 0))
visualizor.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
visualizor.AddSkyBox()
visualizor.AddTypicalLights()
visualizor.Initialize()


hmmwv = veh.HMMWV()
hmmwv.SetChassisCollisionType(veh.ChassisCollisionType_MESH)
hmmwv.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.5), chrono.QUNIT))
hmmwv.SetPowertrainType(veh.PowertrainModelType_SIMPLE)
hmmwv.SetSteeringType(veh.SteeringType_PITMAN_ARM)
hmmwv.SetTireType(veh.TireModelType_RIGID)
hmmwv.SetInitFwdVel(0)
hmmwv.Initialize(my_system)


driver = veh.StandardDriver()
hmmwv.SetDriver(driver)


terrain = veh.DeformableTerrain(my_system)
material = veh.ChDeformableTerrainMaterial()
material.mu = 10.0  
material.young_modulus = 1e7  
material.poisson_ratio = 0.3
material.density = 1500  
material.damping = 0.1  


patch = terrain.AddPatch(material, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT), 40, 40)
patch.SetTexture(chrono.GetChronoDataFile("textures/grass.jpg"))
terrain.SetDrawColorMap(True)  
terrain.Initialize()


time = 0.0
time_step = 0.01  

visualizor.BeginRun()
while visualizor.Run():
    
    key = visualizor.GetKeyboard()
    driver.SetThrottle(key.IsKeyDown(chronoirr.KEY_UP))
    driver.SetBraking(key.IsKeyDown(chronoirr.KEY_DOWN))
    steering = 0
    if key.IsKeyDown(chronoirr.KEY_LEFT):
        steering = -1
    elif key.IsKeyDown(chronoirr.KEY_RIGHT):
        steering = 1
    driver.SetSteering(steering)
    
    
    driver.Synchronize(time)
    hmmwv.Synchronize(time)
    
    
    my_system.DoStepDynamics(time_step)
    time += time_step
    
    
    chassis = hmmwv.GetChassis()
    cam_pos = chassis.GetPos() + chrono.ChVectorD(0, 5, 15)
    visualizor.GetCamera().SetPos(cam_pos)
    visualizor.GetCamera().SetLookAt(chassis.GetPos())
    
    
    visualizor.BeginScene()
    visualizor.Render()
    visualizor.EndScene()