import pychrono.core as chrono
import pychrono.irrlicht as chronoirr


chrono.SetChronoDataPath("/path/to/chrono/data/")


sys = chrono.ChSystemNSC()


ground = chrono.ChBodyEasyBox(sys,  
                              100, 100, 1,  
                              1000,  
                              True,  
                              True,  
                              0.5)  
ground.SetBodyFixed(True)  
sys.Add(ground)


pendulum = chrono.ChBodyEasyCylinder(sys,  
                                     0.1,  
                                     1.0,  
                                     1.0,  
                                     True,  
                                     True,  
                                     0.5)  
pendulum.SetMass(1.0)  
pendulum.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))  
sys.Add(pendulum)


rev_joint = chrono.ChLinkRevolute()
rev_joint.Initialize(ground, pendulum, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0.5)))  
sys.Add(rev_joint)


pendulum.SetPos(chrono.ChVectorD(0, 0, 1.0))  
pendulum.SetVel(chrono.ChVectorD(0, 0, 0))  


app = chronoirr.ChIrrApp(sys, "Simple Pendulum", chronoirr.dimension2du(800, 600))


app.AddTypicalSky()
app.AddTypicalLights()
app.AddCamera(chrono.ChVectorD(0, 0, 2.0))
app.AssetBindAll()


app.SetTimestep(0.01)
app.SetTryRealtime(True)


def LogPendulumState():
    print(f"Time: {sys.GetChTime():.2f} s")
    print(f"Pendulum position: {pendulum.GetPos().x:.2f}, {pendulum.GetPos().y:.2f}, {pendulum.GetPos().z:.2f}")
    print(f"Pendulum velocity: {pendulum.GetVel().x:.2f}, {pendulum.GetVel().y:.2f}, {pendulum.GetVel().z:.2f}")


while app.GetDevice().run():
    app.BeginScene()
    app.DrawAll()
    app.DoStep()
    app.EndScene()
    if sys.GetChTime() % 0.1 < 0.01:  
        LogPendulumState()