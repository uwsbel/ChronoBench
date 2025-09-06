import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens
import numpy as np


chrono.SetChronoDataPath('PYCHRONO_DATA_DIR')


sys = chrono.ChSystemNSC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Gator Vehicle Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3, 6))
vis.AddTypicalLights()


gator = veh.Gator(sys)
gator.SetContactMethod(chrono.ChContactMethod_NSC)
gator.SetChassisCollisionType(veh.Gator.ChassisCollisionType_BOX)
gator.SetChassisFixed(False)
gator.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.5), chrono.QUNIT))
gator.Initialize()


chassis = gator.GetChassisBody()
chassis_shape = chrono.ChVisualShapeBox(chrono.ChVector3d(1.5, 0.8, 3.0))
chassis_shape.SetColor(chrono.ChColor(0.8, 0.2, 0.2))  
chassis.AddVisualShape(chassis_shape)


terrain = veh.RigidTerrain(sys)
patch = terrain.AddPatch(chrono.ChMaterialSurfaceNSC(), 
                        chrono.ChCoordsysd(), 
                        100, 100)  
patch.SetColor(chrono.ChColor(0.3, 0.6, 0.3))  
terrain.Initialize()


driver = veh.ChPathFollowerDriver(gator, 
                                 veh.ChPath(),  
                                 0.1,           
                                 10)            
driver.Initialize()


manager = sens.ChSensorManager(sys)


manager.scene.AddPointLight(chrono.ChVector3f(10, 10, 10), chrono.ChColor(1, 1, 1), 1000)
manager.scene.AddPointLight(chrono.ChVector3f(-10, 10, 10), chrono.ChColor(1, 1, 1), 1000)


camera = sens.ChCameraSensor(
    gator.GetChassisBody(),  
    60.0,  
    chrono.ChFrameD(chrono.ChVector3d(0, 1.5, 1.0),  
                   chrono.Q_from_AngAxis(chrono.CH_PI_2, chrono.ChVector3d(0, 1, 0))),  
    1280,  
    720,   
    chrono.CH_C_PI / 3  
)
camera.SetName("Gator Camera")
manager.AddSensor(camera)


time_step = 1e-3
end_time = 10.0

while vis.Run() and sys.GetChTime() < end_time:
    
    driver.Synchronize(sys.GetChTime())
    terrain.Synchronize(sys.GetChTime())
    gator.Synchronize(sys.GetChTime(), driver.GetInputs(), terrain)
    manager.Update()
    
    
    sys.DoStepDynamics(time_step)
    
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()