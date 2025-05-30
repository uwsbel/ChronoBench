import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens
import math
import time


chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVectorD(0, 0, 0.5)
initRot = chrono.ChQuaternionD(1, 0, 0, 0)
gator = veh.Gator()
gator.SetContactMethod(chrono.ChContactMethod_NSC)
gator.SetChassisFixed(False)
gator.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot))
gator.Initialize()


gator.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
gator.SetWheelVisualizationType(veh.VisualizationType_MESH)
gator.SetTireVisualizationType(veh.VisualizationType_PRIMITIVES)
gator.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
gator.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)


terrain = veh.RigidTerrain(gator.GetSystem())
terrain_mat = chrono.ChMaterialSurfaceNSC()
patch = terrain.AddPatch(
    terrain_mat,
    chrono.ChVectorD(0, 0, 0),
    chrono.ChVectorD(0, 0, 1),
    100, 100
)
patch.SetTexture(chrono.GetChronoDataFile("terrain/textures/grass.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


driver = veh.ChDriverInteractive(gator)
driver.Initialize()


manager = sens.ChSensorManager(gator.GetSystem())


manager.scene.AddPointLight(chrono.ChVectorF(10, 0, 10), chrono.ChColor(1, 1, 1), 500)
manager.scene.AddPointLight(chrono.ChVectorF(-10, 0, 10), chrono.ChColor(1, 1, 1), 500)


cam_offset = chrono.ChVectorF(0.5, 0, 1.2)
cam_rot = chrono.Q_from_AngAxis(math.radians(15), chrono.ChVectorF(0, 1, 0))
camera = sens.ChCameraSensor(
    gator.GetChassisBody(),  
    30.0,                    
    chrono.ChFrameD(cam_offset, cam_rot),  
    1280, 720,               
    math.radians(60)         
)
camera.SetName("Camera Sensor")
camera.PushFilter(sens.ChFilterRGBA8Access())  
manager.AddSensor(camera)


step_size = 1e-3
render_step = 0.05  
time_end = 5.0
time_sim = 0
render_time = 0

print("Simulation started. Press W/A/S/D to drive, SPACE to brake, Q to quit.")

while time_sim < time_end:
    
    driver.Synchronize(time_sim)
    driver.Advance(step_size)

    
    terrain.Synchronize(time_sim)
    gator.Synchronize(time_sim, driver.GetInputs(), terrain)
    
    
    terrain.Advance(step_size)
    gator.Advance(step_size)

    
    manager.Update()

    
    time_sim += step_size

    
    if time_sim >= render_time:
        print(f"Time: {time_sim:.2f} s")
        render_time += render_step

print("Simulation finished.")