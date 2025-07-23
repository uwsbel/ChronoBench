import pychrono as chrono
import pychrono.irrlicht as chronoirr
from itertools import combinations


class MyCreatorForAll(chrono.ChRandomShapeCreator_AddBodyCallback):
    def __init__(self, vis, coll):
        super().__init__()
        self.vis = vis
        self.coll = coll

    def OnAddBody(self, body, coords):
        body.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/bluewhite.png"))
        self.vis.AddAsset(body)
        self.coll.AddAsset(body)


class MyParticleEmitter(chrono.ChParticleEmitter):
    def __init__(self, vis, coll):
        super().__init>(20)
        self.vis = vis
        self.coll = coll
        self.creator_addbody = MyCreatorForAll(vis, coll)

    def CreateParticle(self, particle, abs_coords):
        particle.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/random.png"))
        self.vis.AddAsset(particle)
        self.coll.AddAsset(particle)
        particle.SetRandomForce(chrono.ChVector3d(0, 0, -200))

    def OnUpdate(self, time):
        super().OnUpdate(time)
        if self.GetNumParticles() > 2000:
            self.Disable()


def CreateMySystem():
    sys = chrono.ChSystemNSC()
    sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    grav = chrono.ChVector3d(0, 0, -9.81)
    sys.SetGravitationalAcceleration(grav)

    mphysicalSystem.Add(mator)

    mator_sparse = chrono.ChMatrixInterpolatorD_interval(0, 10)
    mator_sparse.SetCoeffs(mator.GetCoeffs())

    mator_full = chrono.ChMatrixInterpolatorD_interval(0, 10, 100)
    mator_full.SetCoeffs(mator_sparse.GetCoeffs())

    mator.SetInterpolter(mator_full)

    emitter = MyParticleEmitter(vis, coll)

    return sys, emitter


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(msys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Hello particles')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 0, -1))
vis.AddTypicalLights()

mator = chrono.ChRandomShapeCreator()

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    msys.DoStepDynamics(0.02)