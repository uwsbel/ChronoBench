import pychrono as chrono
import pychrono.irrlicht as chronoirr
from itertools import combinations

# -------------------------------------------------------
# Callback to texture & visualize each emitted particle
# -------------------------------------------------------
class MyCreatorForAll(chrono.ChRandomShapeCreator_AddBodyCallback):
    def __init__(self, vis_sys, coll_sys):
        super().__init__()
        self.vis = vis_sys
        self.coll = coll_sys

    def OnAddBody(self, body, coords, creator):
        # apply a texture, bind to the Irrlicht and collision visualization
        body.GetVisualShape(0).SetTexture(
            chrono.GetChronoDataFile("textures/bluewhite.png")
        )
        self.vis.BindItem(body)
        self.coll.BindItem(body)
        body.SetUseGyroTorque(False)


# -------------------------
# 1) Create the Chrono NSC system
# -------------------------
sys = chrono.ChSystemNSC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
coll = sys.GetCollisionSystem()

# no global gravity, we'll add pairwise attraction manually
sys.Set_G_acc(chrono.ChVector3d(0, 0, 0))

# -------------------------
# 2) Common material for spheres
# -------------------------
sphere_mat = chrono.ChContactMaterialNSC()
sphere_mat.SetFriction(0.2)

# -------------------------------------------------
# 3) Create three spheres (radius=2.1, density=1800)
# -------------------------------------------------
# Sphere 1
sphere1 = chrono.ChBodyEasySphere(2.1, 1800, True, True, sphere_mat)
sphere1.SetPos(chrono.ChVector3d(1, 1, 0))
sphere1.SetPos_dt(chrono.ChVector3d(0.5, 0, 0.1))     # initial velocity
sphere1.GetVisualShape(0).SetTexture(
    chrono.GetChronoDataFile("textures/concrete.jpg")
)
sys.Add(sphere1)

# Sphere 2
sphere2 = chrono.ChBodyEasySphere(2.1, 1800, True, True, sphere_mat)
sphere2.SetPos(chrono.ChVector3d(-10, -10, 0))
sphere2.SetPos_dt(chrono.ChVector3d(-0.5, 0, -0.1))
sphere2.GetVisualShape(0).SetTexture(
    chrono.GetChronoDataFile("textures/concrete.jpg")
)
sys.Add(sphere2)

# Sphere 3
sphere3 = chrono.ChBodyEasySphere(2.1, 1800, True, True, sphere_mat)
sphere3.SetPos(chrono.ChVector3d(0, 20, 0))
sphere3.SetPos_dt(chrono.ChVector3d(0, -0.5, 0.2))
sphere3.GetVisualShape(0).SetTexture(
    chrono.GetChronoDataFile("textures/concrete.jpg")
)
sys.Add(sphere3)

# -------------------------
# 4) Particle emitter (unchanged)
# -------------------------
emitter = chrono.ChParticleEmitter()
emitter.SetParticlesPerSecond(2000)
emitter.SetUseParticleReservoir(True)
emitter.SetParticleReservoirAmount(200)

# positioner
emitter_positions = chrono.ChRandomParticlePositionOnGeometry()
# corrected: use ChFrameD() not ChFramed()
emitter_positions.SetGeometry(chrono.ChBox(50, 50, 50), chrono.ChFrameD())
emitter.SetParticlePositioner(emitter_positions)

# aligner
emitter_rotations = chrono.ChRandomParticleAlignmentUniform()
emitter.SetParticleAligner(emitter_rotations)

# linear velocity
mvelo = chrono.ChRandomParticleVelocityAnyDirection()
mvelo.SetModulusDistribution(chrono.ChUniformDistribution(0.0, 0.5))
emitter.SetParticleVelocity(mvelo)

# angular velocity
mangvelo = chrono.ChRandomParticleVelocityAnyDirection()
mangvelo.SetModulusDistribution(chrono.ChUniformDistribution(0.0, 0.2))
emitter.SetParticleAngularVelocity(mangvelo)

# shape creator
mcreator_spheres = chrono.ChRandomShapeCreatorSpheres()
mcreator_spheres.SetDiameterDistribution(chrono.ChZhangDistribution(0.6, 0.23))
mcreator_spheres.SetDensityDistribution(chrono.ChConstantDistribution(1600))
emitter.SetParticleCreator(mcreator_spheres)

# -------------------------
# 5) Irrlicht visualization
# -------------------------
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Three‐Body + Particles")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 14, -20))
vis.AddTypicalLights()

# bind a callback so that every new particle gets textured & visualized
callback = MyCreatorForAll(vis, coll)
emitter.RegisterAddBodyCallback(callback)

# solver settings
sys.SetSolverType(chrono.ChSolver.Type_PSOR)
sys.GetSolver().AsIterative().SetMaxIterations(40)

# -------------------------
# 6) Main simulation loop
# -------------------------
step_size = 1e-2
G_constant = 6.674e-3  # scaled grav. constant

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # emit and add new particles
    emitter.EmitParticles(sys, step_size)

    # clear custom accumulators, compute pairwise gravity
    for body in sys.Get_bodylist():
        body.Empty_forces_accumulators()

    # Kinetic & potential energy reporting
    KE = 0.0
    for b in sys.Get_bodylist():
        m = b.GetMass()
        v = b.GetPosDt()  # linear velocity
        KE += 0.5 * m * v.Length2()

    PE = 0.0
    for A, B in combinations(sys.Get_bodylist(), 2):
        d = B.GetPos() - A.GetPos()
        r = d.Length()
        PE += -G_constant * A.GetMass() * B.GetMass() / r

    # apply mutual attraction forces
    for A, B in combinations(sys.Get_bodylist(), 2):
        d = B.GetPos() - A.GetPos()
        r = d.Length()
        if r > 1e-6:
            Fmag = G_constant * A.GetMass() * B.GetMass() / (r * r)
            Fvec = d * (Fmag / r)
            A.AccumulateForce(Fvec, A.GetPos(), False)
            B.AccumulateForce(-Fvec, B.GetPos(), False)

    print(f"KE = {KE:.6f}, PE = {PE:.6f}, Total = {KE+PE:.6f}")

    # advance dynamics
    sys.DoStepDynamics(step_size)