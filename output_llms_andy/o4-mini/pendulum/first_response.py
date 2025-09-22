# simple_pendulum.py
import pychrono as chrono
import pychrono.irrlicht as chronoirr

def main():
    # 1) Initialize the Chrono system
    chrono.SetChronoDataPath("")              # not strictly needed if you don't load extra assets
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

    # 2) Create the ground (fixed) body
    ground = chrono.ChBody()
    ground.SetBodyFixed(True)
    system.Add(ground)

    # 3) Create the pendulum bob as a simple sphere
    L = 2.0                                   # pendulum length
    bob_radius = 0.1
    bob_density = 1000                       # kg/m^3
    bob = chrono.ChBodyEasySphere(bob_radius, bob_density, True, True)
    # Position the bob so that the pivot is at (0,0,0) and the bob is at (0, -L, 0)
    bob.SetPos(chrono.ChVectorD(0, -L, 0))
    system.Add(bob)

    # 4) Revolute joint between bob and ground at the origin, axis = global Z
    revolute = chrono.ChLinkLockRevolute()
    pivot_csys = chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT)
    revolute.Initialize(bob, ground, pivot_csys)
    system.AddLink(revolute)

    # 5) Set up Irrlicht visualization
    app = chronoirr.ChIrrApp(system, "Simple Pendulum", chronoirr.dimension2du(800, 600))
    app.AddTypicalSky()
    app.AddTypicalCamera(chronoirr.vector3df(1.5, 1.5, 4))
    app.AddTypicalLights()
    app.SetTimestep(0.01)
    app.AssetBindAll()
    app.AssetUpdateAll()

    # 6) Simulation loop: draw, step, and log every 0.1 s
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
            # Get pendulum bob position and velocity (linear velocity of COM)
            pos = bob.GetPos()
            vel = bob.GetPos_dt()
            print(f"time = {t:6.3f}   pos = ({pos.x: .3f}, {pos.y: .3f}, {pos.z: .3f})"
                  f"   vel = ({vel.x: .3f}, {vel.y: .3f}, {vel.z: .3f})")
            next_log_time += log_interval

if __name__ == "__main__":
    main()