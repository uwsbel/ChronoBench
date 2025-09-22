import chrono as ch
from chrono_core import *
from chrono_irrlicht import *


ch.InitChronic(chrono_data_path='.')


world = ch.ChWorld()


ground = ch.ChBodyEasyBox(m=1, pos=[0, 0, 0], sz=[10, 0.1, 10], color=[1, 0, 0])
ground.SetBodyFixed()
world.Add(ground)


beam = ch.ChBodyEasyBeam(m=1, pos=[0, 0.5, 0], length=5, radius=0.1, color=[0, 1, 0])
world.Add(beam)


motor = ch.ChMotor()
motor.SetForce([0, 0, 0])
motor.SetMaxForce([100, 100, 100])
motor.SetPos([0, 0, 0])
motor.SetTarget([0, 0, 0])
beam.AddMotor(motor)


def custom_motor_function(beam, motor, t):
    
    force = [10 * math.sin(t), 0, 0]
    motor.SetForce(force)
custom_motor_function(beam, motor, 0)


beam.SetMaterialDefId(1)
beam.SetDensity(1)
beam.SetCrossSection(0.1, 0.1)
beam.SetYoungModulus(1e6)
beam.SetPoissonRatio(0.3)


constraint = ch.ChPoint2PointConstraint(beam, ground, [0, 0, 0], [0, 0, 0])
world.Add(constraint)


solver = ch.ChSolverNewton()
timestepper = ch.ChTimestepperRungeKutta()


irrlicht = ch.ChIrrlichtInit()
irrlicht.SetDeviceType(irrlicht.EDT_OPENGL)
irrlicht.SetWindowFlags(irrlicht.EFW_CENTERED | irrlicht.EFW_RESIZABLE)
irrlicht.SetClientSize(800, 600)
irrlicht.SetTitle('Beam Buckling Simulation')
irrlicht.AddFrameCallback(irrlicht.IE_FRAME_POSTSTEP, lambda: irrlicht.DrawAll())


t = 0
dt = 0.01
while t < 10:
    world.DoStep(dt)
    t += dt
    irrlicht.BeginScene()
    irrlicht.DrawAll()
    irrlicht.EndScene()


world.Destroy()
irrlicht.Shutdown()