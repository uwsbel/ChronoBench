import chrono as ch
import chrono.irrlicht as irr


ch.ChEngine.Initialize()


app = irr.ChIrrApp()


ground = ch.ChBody()
ground.SetPos(ch.ChVector3d(0, 0, -10))
ground.SetFixed(True)


pendulum = ch.ChBody()
pendulum.SetMass(1.0)
pendulum.SetPos(ch.ChVector3d(5, 0, 0))
pendulum.SetRot(ch.ChQuaterniond(1, 0, 0, 0))


inertia = ch.ChMatrix33d(1, 0, 0, 0, 1, 0, 0, 0, 1)
inertia(0, 0) = 0.1
inertia(1, 1) = 0.1
inertia(2, 2) = 0.1
pendulum.SetInertia(inertia)


joint = ch.ChRevolute()
joint.Init(ground, pendulum, ch.ChFrame3d(ch.ChVector3d(0, 0, 0), ch.ChQuaterniond(1, 0, 0, 0)))







scene = irr.ChIrrScene()
scene.AddBody(ground)
scene.AddBody(pendulum)
scene.AddLink(joint)


scene.SetCameraPos(ch.ChVector3d(0, -20, 10))
scene.SetCameraTarget(ch.ChVector3d(0, 0, 0))


time = 0
dt = 0.02
while time < 30 and app.Run():
    
    pendulum.Update()
    joint.Update()
    
    
    print(f"Time: {time:.2f}, Pendulum Position: {pendulum.GetPos()}, Pendulum Velocity: {pendulum.GetPos_dt()}")
    
    
    time += dt

    
    scene.Render()