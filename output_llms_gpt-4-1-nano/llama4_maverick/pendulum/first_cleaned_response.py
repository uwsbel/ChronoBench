import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np

def main():
    
    sys = chrono.ChSystemNSC()

    
    ground = chrono.ChBody()
    ground.SetBodyFixed(True)
    sys.Add(ground)

    
    pendulum = chrono.ChBody()
    pendulum.SetMass(1.0)  
    pendulum.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))  
    pendulum.SetPos(chrono.ChVectorD(0, -1, 0))  
    pendulum.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))  
    sys.Add(pendulum)

    
    rev_joint = chrono.ChLinkRevolute()
    rev_joint.Initialize(ground, pendulum, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.Q_ROTATE_Z_TO_Y))
    sys.Add(rev_joint)

    
    app = chronoirr.ChIrrApp(sys, 'Simple Pendulum', chronoirr.dimension2du(800, 600))
    app.AddTypicalSky()
    app.AddTypicalCamera(chronoirr.vector3df(0, 0, 3))
    app.AddTypicalLights()
    app.AssetBindAll()
    app.AssetUpdateAll()

    
    timestep = 0.01
    while app.GetDevice().run():
        app.BeginScene()
        app.DrawAll()
        sys.DoStepDynamics(timestep)
        app.EndScene()

        
        if sys.GetChTime() % 0.1 < timestep:
            pos = pendulum.GetPos()
            vel = pendulum.GetPos_dt()
            print(f'Time: {sys.GetChTime():.2f}, Position: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f}), Velocity: ({vel.x:.2f}, {vel.y:.2f}, {vel.z:.2f})')

    return 0

if __name__ == '__main__':
    main()