import pychrono as chrono
import pychrono.irrlicht as chronoirr


chrono.Ch纪元().Initialize()


system = chrono.ChSystemNSC()


system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('Particle Emitter Simulation')
vis.Initialize()


cam_pos = chrono.ChVectorD(0, 10, 10)
cam_target = chrono.ChVectorD(0, 0, 0)
vis.SetCameraTarget(cam_pos, cam_target)


emitter = chrono.ChBodyEasyCylinder(0.05, 0.05, 0.05, 1000, False, False)
emitter.SetPos(chrono.ChVectorD(0, 0, 0))
emitter.SetBodyFixed(True)
system.AddBody(emitter)


class Particle:
    def __init__(self, radius, mass, position, velocity, orientation):
        self.body = chrono.ChBodyEasySphere(radius, mass, False, False)
        self.body.SetPos(position)
        self.body.SetVel(velocity)
        self.body.SetRot(chrono.Q_from_EulerXYZ(orientation.X(), orientation.Y(), orientation.Z()))
        system.AddBody(self.body)


num_particles = 100
for i in range(num_particles):
    radius = 0.01 + 0.02 * (i % 5)
    mass = 0.1 * (i % 10)
    position = chrono.ChVectorD(chrono.Ch纪元().GetSystemTime(), 0, 0)
    velocity = chrono.ChVectorD(0.01 * (i % 5), 0, 0)
    orientation = chrono.ChQuaternionD()
    Particle(radius, mass, position, velocity, orientation)


emitter.SetVisualShape(chrono.ChVisualShape(chrono.ChTriangleMesh()))
emitter.GetVisualShape(0).SetVisible(False)


while vis.Run():
    system.DoStepDynamics(1/60)

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.StepVisual()


vis.Finish()
chrono.Ch纪元().Finalize()