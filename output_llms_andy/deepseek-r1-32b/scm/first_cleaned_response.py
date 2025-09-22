importchrono
import math
import time


system = chrono.ChSystem()
system.SetSolverType(chrono.ChSolverType.SOLVER_BARZILAI_BORWEIN)
system.SetSolverMaxIterations(100)
system.SetSolverTolerance(1e-6)


vis = chrono.ChIrrApp(system, "HMMWV on Deformable Terrain", chrono.CH,dimensions(800,600))
vis.SetCamera(chrono.ChVector(0, 5, 10), chrono.ChVector(0,0,0))
vis.AddTypicalLights()


hmmwv = chrono.ChHMMWV()
hmmwv.Initialize(chrono.ChCoordsysD(chrono.ChVector(0,0,1), chrono.ChQuaternion(1,0,0,0)), 
                  chrono.ChHMMWV::TIRE_RIGID)
hmmwv.SetMeshVisualization(True)
system.Add(hmmwv)


terrain = chrono.ChSCMTerrain()
terrain.SetSize(20,20)
terrain.SetPosition(chrono.ChVector(0,0,0))
terrain.SetSoilParameters(1500, 0.5, 1e7, 1e6)
terrain.SetMovingPatchEnabled(True)
terrain.SetMovingPatchDimensions(5,5)
terrain.SetSinkageVisualizationEnabled(True)
terrain.SetSinkageVisualizationType(chrono.ChSCMTerrain.SINKAGE_COLOR)
system.Add(terrain)


driver = chrono.ChDriver()
driver.SetSteering(0)
driver.SetThrottle(0)
driver.SetBraking(0)
hmmwv.SetDriver(driver)


def on_key_press(event):
    if event.Key == chrono.KEY_UP:
        driver.SetThrottle(0.5)
    elif event.Key == chrono.KEY_DOWN:
        driver.SetBraking(0.5)
    elif event.Key == chrono.KEY_LEFT:
        driver.SetSteering(-0.5)
    elif event.Key == chrono.KEY_RIGHT:
        driver.SetSteering(0.5)

def on_key_release(event):
    driver.SetThrottle(0)
    driver.SetBraking(0)
    driver.SetSteering(0)

vis.SetKeyDownCallback(on_key_press)
vis.SetKeyUpCallback(on_key_release)


dt = 0.02
time = 0

while vis.Run():
    
    driver.Update(time, dt)
    
    
    hmmwv.Update(time, dt)
    
    
    system.DoStepDynamics(dt)
    
    
    vis.Render()
    
    time += dt
    time.sleep(0.02)