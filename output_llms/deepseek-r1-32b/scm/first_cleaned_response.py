importchrono
import math
import time


system =chrono.ChSystemNSC()
system.SetSolverType(chrono.ChSolverType.SOLVER_BARZILAI_BORWEIN)
system.SetSolverMaxIterations(100)
system.SetSolverTolerance(1e-6)
system.SetRealTimeSimulation(True)


application =chrono.ChIrrApp(system, "HMMWV on Deformable Terrain", chrono.CH,dimensions=(800,600))
application.SetCameraPosition(chrono.ChVectorD(5,5,2))
application.SetCameraRotation(chrono.ChVectorD(0,0,0))


hmmwv =chrono.ChHMMWV()
hmmwv.SetChassisPos(chrono.ChVectorD(0,0,1))
hmmwv.SetChassisRot(chrono.ChQuaternionD(1,0,0,0))
hmmwv.Initialize()


terrain =chrono.ChSCMTerrain()
terrain.SetSize(20,20)  
terrain.SetPosition(chrono.ChVectorD(0,0,0))
terrain.SetSoilParameters(
    density=1500,  
    friction=0.5,
    cohesion=1000,
    tangent_modulus=1e6,
    damping=0.1
)
terrain.EnableMovingPatch(True)
terrain.SetMovingPatchDimensions(5,5)  
terrain.SetMovingPatchOffset(0,0)  
terrain.EnableSinkageVisualization(True)
terrain.SetSinkageColorRange(0,0.3)  
terrain.Initialize()


hmmwv.GetChassis().SetCollide(True)
hmmwv.GetChassis().SetRender(True)
for wheel in hmmwv.GetWheelSet().GetWheelList():
    wheel.Get Rim().SetCollide(True)
    wheel.Get Rim().SetRender(True)
    wheel.Get Tire().SetCollide(True)
    wheel.Get Tire().SetRender(True)


driver =chrono.ChDriver(hmmwv.GetDriver())
driver.SetMaxThrottle(1)
driver.SetMaxSteering(1)
driver.SetMaxBraking(1)


def on_key_press(event):
    if event.key == 'w':
        driver.SetThrottle(1)
    elif event.key == 's':
        driver.SetThrottle(-1)
    elif event.key == 'a':
        driver.SetSteering(1)
    elif event.key == 'd':
        driver.SetSteering(-1)
    elif event.key == 'q':
        driver.SetBraking(1)
    elif event.key == 'e':
        driver.SetBraking(-1)

application.SetKeyDownCallback(on_key_press)


light =chrono.ChLight()
light.SetPosition(chrono.ChVectorD(5,5,5))
light.SetIntensity(1)
application.AddLight(light)

dir_light =chrono.ChDirLight()
dir_light.SetDirection(chrono.ChVectorD(-1,-1,-1))
dir_light.SetIntensity(0.5)
application.AddLight(dir_light)


application.Begin()
while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.EndScene()
    
    
    driver.Update()
    system.DoStepDynamics(1/50)  
    
    
    chassis_pos =hmmwv.GetChassis().GetPos()
    terrain.SetMovingPatchOffset(chassis_pos.x, chassis_pos.z)
    
    
    terrain.UpdateVisualization()
    time.sleep(0.02)  
    
application.End()