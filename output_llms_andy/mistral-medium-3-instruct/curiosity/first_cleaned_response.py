import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np


chrono.SetChronoDataPath("PATH_TO_CHRONO_DATA")  


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  




terrain_body = chrono.ChBodyEasyBox(20, 0.5, 20, 1000, True, True)
terrain_body.SetPos(chrono.ChVectorD(0, -1, 0))
terrain_body.SetBodyFixed(True)
terrain_body.GetVisualModel().ClearModel()
terrain_mesh = chrono.ChTriangleMeshConnected()
terrain_mesh.LoadWavefrontMesh(chrono.GetChronoDataFile("terrain/flat_terrain.obj"))
terrain_body.GetVisualModel().AddTriangleMesh(terrain_mesh)
terrain_body.GetCollisionModel().ClearModel()
terrain_body.GetCollisionModel().AddTriangleMesh(terrain_mesh, False, False, chrono.ChVectorD(0, 0, 0), chrono.ChMatrix33D(1))
terrain_body.GetCollisionModel().BuildModel()
system.Add(terrain_body)


rover_body = chrono.ChBodyEasyBox(1.5, 0.5, 1, 200, True, True)
rover_body.SetPos(chrono.ChVectorD(0, 0.5, 0))
system.Add(rover_body)


wheel_radius = 0.25
wheel_width = 0.2
wheel_positions = [
    chrono.ChVectorD(0.7, 0.2, 0.4),
    chrono.ChVectorD(-0.7, 0.2, 0.4),
    chrono.ChVectorD(0.7, 0.2, -0.4),
    chrono.ChVectorD(-0.7, 0.2, -0.4)
]

wheels = []
for i, pos in enumerate(wheel_positions):
    wheel = chrono.ChBodyEasyCylinder(wheel_radius, wheel_width, 10, True, True)
    wheel.SetPos(pos)
    system.Add(wheel)
    wheels.append(wheel)

    
    motor = chrono.ChLinkMotorRotationSpeed()
    motor.Initialize(wheel, rover_body, chrono.ChFrameD(pos))
    motor.SetMotorFunction(chrono.ChFunction_Const(1.0))  
    system.Add(motor)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Curiosity Rover Simulation")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(5, 2, 0), chrono.ChVectorD(0, 0, 0))
vis.AddLight(chrono.ChVectorD(5, 5, -5), chrono.ChVectorD(0, 0, 0), 10, chrono.ChColor(1, 1, 1))
vis.AddLight(chrono.ChVectorD(-5, 5, 5), chrono.ChVectorD(0, 0, 0), 10, chrono.ChColor(1, 1, 1))
vis.SetShadows(True)
vis.AttachSystem(system)


terrain_texture = chrono.ChTexture()
terrain_texture.SetTextureFilename(chrono.GetChronoDataFile("textures/terrain.jpg"))
terrain_body.GetVisualModel().AddTexture(terrain_texture)


rover_texture = chrono.ChTexture()
rover_texture.SetTextureFilename(chrono.GetChronoDataFile("textures/metal.jpg"))
rover_body.GetVisualModel().AddTexture(rover_texture)


time_step = 0.01
max_time = 20.0


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    current_time = system.GetChTime()
    motor_speed = 1.0 + 0.5 * np.sin(current_time * 2.0)

    for motor in system.Get_otherphysicslist():
        if isinstance(motor, chrono.ChLinkMotorRotationSpeed):
            motor.SetMotorFunction(chrono.ChFunction_Const(motor_speed))

    system.DoStepDynamics(time_step)

    if system.GetChTime() > max_time:
        break