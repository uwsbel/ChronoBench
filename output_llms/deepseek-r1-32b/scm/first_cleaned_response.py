importchrono
import math
import time


system = chrono.ChSystem()
system.SetSolverType(chrono.ChSolverType.SOLVER_BARZILAI_BORWEIN)
system.SetSolverMaxIterations(100)
system.SetSolverTolerance(1e-6)


irr = chrono.ChIrrApp(system, "HMMWV on Deformable Terrain", chrono.CHT,dimensions=(800,600))
irr.SetCameraPosition(chrono.ChVectorD(5, 5, 5))
irr.SetCameraRotation(chrono.ChVectorD(-45, 45, 0))


hmmwv = chrono.ChHMMWV()
hmmwv.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1), chrono.ChQuaternionD(1, 0, 0, 0)))
hmmwv.SetRigidTires()
hmmwv.SetVisualizationType(chrono.ChVisualizationType.Mesh)
system.Add(hmmwv)


terrain = chrono.ChSCMTerrain()
terrain.SetSize(20, 20)
terrain.SetPosition(chrono.ChVectorD(0, 0, 0))
terrain.SetSoilParameters(1e3, 0.5, 1e6, 1e3, 1e3)
terrain.SetMovingPatch(True, chrono.ChVectorD(2, 2, 0.5))
terrain.EnableSinkageVisualization(True)
system.Add(terrain)


driver = chrono.ChInteractiveDriverIRR()
driver.SetSteering(0)
driver.SetThrottle(0)
driver.SetBraking(0)
driver.SetVehicle(hmmwv)


def on_key_press(key):
    if key == 'left':
        driver.SetSteering(-0.5)
    elif key == 'right':
        driver.SetSteering(0.5)
    elif key == 'up':
        driver.SetThrottle(0.5)
    elif key == 'down':
        driver.SetBraking(0.5)

def on_key_release(key):
    if key in ['left', 'right']:
        driver.SetSteering(0)
    elif key in ['up', 'down']:
        driver.SetThrottle(0)
        driver.SetBraking(0)


irr.AddKeyDownCallback(on_key_press)
irr.AddKeyUpCallback(on_key_release)


dt = 0.02
time_step = 0
while irr.Run():
    
    driver.Update()
    
    
    system.DoStepDynamics(dt)
    
    
    irr.DrawAll()
    irr.DrawSCMTerrain(terrain)
    
    
    time_step += 1
    if time_step % 50 == 0:
        print(f"Time: {system.GetChTime():.2f}s")
    
    
    if irr.GetKey(chrono.KEY_UP):
        driver.SetThrottle(0.5)
    elif irr.GetKey(chrono.KEY_DOWN):
        driver.SetBraking(0.5)
    else:
        driver.SetThrottle(0)
        driver.SetBraking(0)
    
    
    if irr.GetKey(chrono.KEY_LEFT):
        driver.SetSteering(-0.5)
    elif irr.GetKey(chrono.KEY_RIGHT):
        driver.SetSteering(0.5)
    else:
        driver.SetSteering(0)