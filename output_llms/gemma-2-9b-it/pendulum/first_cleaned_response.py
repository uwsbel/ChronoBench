from pychrono import ChSystem, ChBody, ChLink, ChMaterialSurface, ChVectorD, ChQuaternionD, ChFrame
from pychrono.irrlicht import ChIrrApp


system = ChSystem()


ground = ChBody(mass=1e6)
ground.SetPos(ChVectorD(0, 0, 0))
system.AddBody(ground)


pendulum_mass = 1.0
pendulum_inertia = ChVectorD(0.1, 0.1, 0.1)  
pendulum = ChBody(mass=pendulum_mass)
pendulum.SetPos(ChVectorD(0, 1, 0))
pendulum.SetInertia(pendulum_inertia)
system.AddBody(pendulum)


joint = ChLink(ground, pendulum, ChFrame(ChVectorD(0, 0, 0), ChQuaternionD(1, 0, 0, 0)))
joint.SetPos(ChVectorD(0, 0, 0))
joint.SetAxis(ChVectorD(0, 0, 1))  
system.AddLink(joint)


app = ChIrrApp(system, 'Pendulum Simulation', windowSize=ChVectorD(800, 600))
app.AddCamera(ChVectorD(5, 5, 5))
app.SetChaseTarget(pendulum)
app.AssetBindAll()


dt = 0.01
while app.GetTimer() < 10:
    system.DoStepDynamics(dt)
    
    
    print(f"Time: {app.GetTimer()}, Pendulum Angle: {pendulum.GetPos().y}")
    
    app.DrawAll()
    app.GetDevice().sleep()