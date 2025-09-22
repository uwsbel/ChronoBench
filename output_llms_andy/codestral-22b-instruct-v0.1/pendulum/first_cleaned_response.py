import pychrono as chrono
import pychrono.irrlicht as irr


system = chrono.ChSystemNSC()


ground = chrono.ChBodyEasyBox(10, 1, 1, 1000, True, True)
system.Add(ground)


pendulum = chrono.ChBodyEasySphere(0.5, 1000, True, True)
pendulum.SetPos(chrono.ChVectorD(0, -2, 0))
system.Add(pendulum)


joint = chrono.ChLinkLockRevolute()
joint.Initialize(ground, pendulum, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngX(chrono.CH_C_PI_2)))
system.Add(joint)


pendulum.SetWvel_par(chrono.ChVectorD(0, 0, 1))


app = irr.ChIrrApp(system, 'Pendulum Simulation', irr.dimension2du(800, 600))
app.AddTypicalLogo()
app.AddTypicalCamera(irr.vector3df(0, -3, -6))
app.AddTypicalLights()
app.AssetBindAll()
app.AssetUpdateAll()


while app.GetDevice().run():
    app.BeginScene()
    app.DrawAll()
    app.DoStep()
    app.EndScene()

    
    pendulum_pos = pendulum.GetPos()
    pendulum_vel = pendulum.GetPos_dt()
    print(f"Position: {pendulum_pos.x}, {pendulum_pos.y}, {pendulum_pos.z}")
    print(f"Velocity: {pendulum_vel.x}, {pendulum_vel.y}, {pendulum_vel.z}")

    
    chrono.ChTime.Sleep(1/60)