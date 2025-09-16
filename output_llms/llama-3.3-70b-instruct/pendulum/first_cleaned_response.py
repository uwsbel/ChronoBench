import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.postprocess as postprocess


sys = chrono.ChSystemNSC()


contact_material = chrono.ChMaterialSurfaceNSC()
contact_material.SetFriction(0.5)
contact_material.SetDampingF(0.2)
contact_material.SetCompliance(1e-6)
contact_material.SetComplianceT(1e-6)


ground = chrono.ChBodyEasyBox(sys, 10, 1, 10, 1000, True, True, contact_material)
ground.SetPos(chrono.ChVectorD(0, -5, 0))
ground.SetBodyFixed(True)
sys.Add(ground)


pendulum = chrono.ChBodyEasySphere(sys, 1, 10, contact_material)
pendulum.SetPos(chrono.ChVectorD(0, 5, 0))
sys.Add(pendulum)


revolute_joint = chrono.ChLinkRevolute()
revolute_joint.Initialize(ground, pendulum, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 1, 0))
sys.Add(revolute_joint)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AddCamera(chrono.ChVectorD(0, 0, 10))
vis.AddTypicalLights()


class PendulumReporter(postprocess.ChReporter):
    def __init__(self, pendulum):
        super().__init__()
        self.pendulum = pendulum

    def Report(self, frame):
        pos = self.pendulum.GetPos()
        vel = self.pendulum.GetPos_D()
        print(f"Frame {frame}: Position = ({pos.x():.2f}, {pos.y():.2f}, {pos.z():.2f}), Velocity = ({vel.x():.2f}, {vel.y():.2f}, {vel.z():.2f})")

reporter = PendulumReporter(pendulum)
sys.GetReportCallback().AddReporter(reporter)


time_step = 0.01
duration = 10


for frame in range(int(duration / time_step)):
    sys.DoStepDynamics(time_step)
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()

    
    if frame % 10 == 0:
        reporter.Report(frame)