import pychrono as chrono
import pychrono.irrlicht as chronoirr

def main():
    
    chrono.SetChronoDataPath("")              
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

    
    ground = chrono.ChBody()
    ground.SetBodyFixed(True)
    system.Add(ground)

    
    L = 2.0                                   
    bob_radius = 0.1
    bob_density = 1000                       
    bob = chrono.ChBodyEasySphere(bob_radius, bob_density, True, True)
    
    bob.SetPos(chrono.ChVectorD(0, -L, 0))
    system.Add(bob)

    
    revolute = chrono.ChLinkLockRevolute()
    pivot_csys = chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT)
    revolute.Initialize(bob, ground, pivot_csys)
    system.AddLink(revolute)

    
    app = chronoirr.ChIrrApp(system, "Simple Pendulum", chronoirr.dimension2du(800, 600))
    app.AddTypicalSky()
    app.AddTypicalCamera(chronoirr.vector3df(1.5, 1.5, 4))
    app.AddTypicalLights()
    app.SetTimestep(0.01)
    app.AssetBindAll()
    app.AssetUpdateAll()

    
    time_step = 0.01
    log_interval = 0.1
    next_log_time = 0.0

    while app.GetDevice().run():
        app.BeginScene()
        app.DrawAll()
        app.EndScene()

        system.DoStepDynamics(time_step)

        t = system.GetChTime()
        if t >= next_log_time:
            
            pos = bob.GetPos()
            vel = bob.GetPos_dt()
            print(f"time = {t:6.3f}   pos = ({pos.x: .3f}, {pos.y: .3f}, {pos.z: .3f})"
                  f"   vel = ({vel.x: .3f}, {vel.y: .3f}, {vel.z: .3f})")
            next_log_time += log_interval

if __name__ == "__main__":
    main()