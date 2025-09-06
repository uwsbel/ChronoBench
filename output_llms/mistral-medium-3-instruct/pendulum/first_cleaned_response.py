import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np
import time


chrono.SetChronoDataPath("PATH_TO_CHRONO_DATA")  


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  




ground = chrono.ChBodyEasyBox(4, 0.5, 4, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -1, 0))
ground.SetBodyFixed(True)
system.Add(ground)


pendulum_length = 2.0
bob_radius = 0.1
bob_mass = 1.0


bob_inertia = (2.0/5.0) * bob_mass * bob_radius**2

pendulum = chrono.ChBodyEasySphere(bob_radius, bob_mass, True, True)
pendulum.SetPos(chrono.ChVectorD(0, 0, 0))
pendulum.SetInertiaXX(chrono.ChVectorD(bob_inertia, bob_inertia, bob_inertia))
system.Add(pendulum)


pivot_point = chrono.ChVectorD(0, 0, 0)
joint = chrono.ChLinkLockRevolute()
joint.Initialize(ground, pendulum, chrono.ChFrameD(pivot_point))
system.Add(joint)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('Simple Pendulum Simulation')
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(5, 2, 0))
vis.AddTypicalLights()


class PendulumApp(chronoirr.ChIrrApp):
    def __init__(self, system, vis):
        super().__init__(system, vis)
        self.time_step = 0.01
        self.log_interval = 0.1  
        self.last_log_time = 0

    def Advance(self, step):
        
        current_time = self.GetDevice().getTimer().getTime() / 1000.0
        if current_time - self.last_log_time >= self.log_interval:
            
            pos = self.system.Get_bodylist()[1].GetPos()
            vel = self.system.Get_bodylist()[1].GetPos_dt()

            print(f"Time: {current_time:.2f}s")
            print(f"Position: ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f})")
            print(f"Velocity: ({vel.x:.3f}, {vel.y:.3f}, {vel.z:.3f})")
            print("---")

            self.last_log_time = current_time

        
        self.DoStepDynamics(self.time_step)


app = PendulumApp(system, vis)


pendulum.SetPos_dt(chrono.ChVectorD(0.5, 0, 0))  


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    app.Advance(0.01)