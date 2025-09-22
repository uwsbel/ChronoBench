import pychrono


system = pychrono.ChSystemSMC()


ground_body = pychrono.ChBodyEasyBox(10, 0.1, 10, 1000, True, False, system)
ground_body.SetPos(pychrono.ChVectorD(0, -1, 0))

pendulum_body = pychrono.ChBodyEasySphere(1, 1000, True, False, system)
pendulum_body.SetPos(pychrono.ChVectorD(0, 5, 0))


pendulum_body.SetInertia(pychrono.ChVectorD(1, 10, 1))
revolute_joint = pychrono.ChLinkRevolute()
revolute_joint.Initialize(ground_body, pendulum_body, pychrono.ChVectorD(0, 0, 0), pychrono.ChVectorD(0, 0, 0))


system.AddLink(revolute_joint)


visualization_system = pychrono.ChVisualSystemIrrlicht()
visualization_system.SetWindowSize(800, 600)
visualization_system.SetWindowTitle('Simple Pendulum Simulation')
visualization_system.SetWindowPosition(50, 50)
visualization_system.SetRenderMode(pychrono.ChVisualizationGLFW.e_RENDER_SHADED_FLAT)
visualization_system.Initialize()
visualization_system.AddCamera(pychrono.ChVectorD(0, 10, -10), pychrono.ChVectorD(0, 0, 0), pychrono.ChVectorD(0, -1, 0))

while visualization_system.Run():
    system.DoStepDynamics(0.01)
    visualization_system.BeginScene()
    visualization_system.Render()
    visualization_system.EndScene()

    
    if system.GetChTime() % 1 < 0.01:
        print(f'Time: {system.GetChTime():.2f}, Position: {pendulum_body.GetPos().x():.2f}, Velocity: {pendulum_body.GetPos_dt().x():.2f}')