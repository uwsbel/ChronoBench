import pychrono


system = pychrono.ChSystemSMC()


ground_body = system.AddGroundBody()
pendulum = system.AddBody(pychrono.ChBodyEasyBox(0.1, 1, 0.1, 1000, True, True))


pendulum.SetPos(pychrono.ChVectorD(0, 5, 0))
pendulum.SetInertia(pychrono.ChVectorD(1, 1, 1))

revolute_joint = pychrono.ChLinkRevolute()
revolute_joint.Initialize(ground_body, pendulum, pychrono.ChVectorD(0, 0, 0), pychrono.ChVectorD(0, 0, 0))
system.AddLink(revolute_joint)


visualization = pychrono.ChVisualSystemIrrlicht()
visualization.AttachSystem(system)
visualization.SetWindowSize(800, 600)
visualization.SetWindowTitle('Simple Pendulum Simulation')
visualization.SetRenderMode(pychrono.ChVisualization.eRENDER_SHAPES)

while visualization.Run():
    system.DoStepDynamics(0.01)
    visualization.BeginScene(True, True, pychrono.ChColor(0.4, 0.4, 0.4))
    visualization.Render()
    visualization.EndScene()

    if visualization.GetKeyPressed() == ord('Q'):
        break

    print(f'Pendulum position: {pendulum.GetPos()}, velocity: {pendulum.GetPos_dt()}')