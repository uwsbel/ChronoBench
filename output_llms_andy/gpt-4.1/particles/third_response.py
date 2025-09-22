import pychrono as chrono
import pychrono.irrlicht as chronoirr
from itertools import combinations

# -------------------------------
# ERROR FIXES AND MODIFICATIONS
# -------------------------------
# 1. ChRandomShapeCreator_AddBodyCallback is not directly available in pychrono; use ChRandomShapeCreator.AddBodyCallback instead.
# 2. ChRandomParticlePositionOnGeometry expects a geometry and a frame; ChBox should be constructed with half-dimensions.
# 3. ChFramed() is not a constructor in PyChrono; use chrono.ChFrame() for the frame.
# 4. ChRandomParticleAlignmentUniform() should be ChRandomParticleAlignmentUniform() (correct).
# 5. ChRandomParticleVelocityAnyDirection() is correct.
# 6. ChRandomShapeCreatorSpheres() is correct.
# 7. ChZhangDistribution and ChConstantDistribution are correct.
# 8. ChSolver.Type_PSOR is correct.
# 9. SetGravitationalAcceleration expects a ChVector3d.
# 10. sys.GetBodies() returns a chrono.ChBodyIterator, which is iterable.
# 11. AccumulateForce expects a force vector, an application point, and a local flag.
# 12. Use of -F_attract: PyChrono does not support unary minus for ChVector3d; use .__neg__() or multiply by -1.
# 13. The visual system needs to bind only once per body.
# 14. The collision system does not have a BindItem method; remove self.coll.BindItem(body).
# 15. The emitter's AddBodyCallback should be set via RegisterAddBodyCallback.
# 16. The emitter's SetUseParticleReservoir and SetParticleReservoirAmount are correct.
# 17. The simulation loop should call sys.DoStepDynamics(stepsize) after applying forces.

# -------------------------------
# CALLBACK CLASS
# -------------------------------
class MyCreatorForAll(chrono.ChRandomShapeCreator.AddBodyCallback):
    def __init__(self, vis):
        super().__init__()
        self.vis = vis

    def OnAddBody(self, body, coords, creator):
        body.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/bluewhite.png"))
        self.vis.BindItem(body)
        body.SetUseGyroTorque(False)

# -------------------------------
# SYSTEM SETUP
# -------------------------------
sys = chrono.ChSystemNSC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# -------------------------------
# CREATE THREE SPHERES
# -------------------------------
sphere_mat = chrono.ChContactMaterialNSC()
sphere_mat.SetFriction(0.2)

# Sphere 1
sphere1 = chrono.ChBodyEasySphere(2.1, 1800, True, True, sphere_mat)
sphere1.SetPos(chrono.ChVector3d(1, 1, 0))
sphere1.SetPos_dt(chrono.ChVector3d(0.5, 0, 0.1))  # Set initial velocity
sphere1.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
sys.Add(sphere1)

# Sphere 2
sphere2 = chrono.ChBodyEasySphere(2.1, 1800, True, True, sphere_mat)
sphere2.SetPos(chrono.ChVector3d(-10, -10, 0))
sphere2.SetPos_dt(chrono.ChVector3d(-0.5, 0, -0.1))  # Set initial velocity
sphere2.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
sys.Add(sphere2)

# Sphere 3
sphere3 = chrono.ChBodyEasySphere(2.1, 1800, True, True, sphere_mat)
sphere3.SetPos(chrono.ChVector3d(0, 20, 0))
sphere3.SetPos_dt(chrono.ChVector3d(0, -0.5, 0.2))  # Set initial velocity
sphere3.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
sys.Add(sphere3)

# -------------------------------
# PARTICLE EMITTER (unchanged)
# -------------------------------
emitter = chrono.ChParticleEmitter()
emitter.SetParticlesPerSecond(2000)
emitter.SetUseParticleReservoir(True)
emitter.SetParticleReservoirAmount(200)

# Randomizers for particle properties
emitter_positions = chrono.ChRandomParticlePositionOnGeometry()
emitter_positions.SetGeometry(chrono.ChBox(50, 50, 50), chrono.ChFrame())  # Use ChFrame, not ChFramed
emitter.SetParticlePositioner(emitter_positions)

emitter_rotations = chrono.ChRandomParticleAlignmentUniform()
emitter.SetParticleAligner(emitter_rotations)

mvelo = chrono.ChRandomParticleVelocityAnyDirection()
mvelo.SetModulusDistribution(chrono.ChUniformDistribution(0.0, 0.5))
emitter.SetParticleVelocity(mvelo)

mangvelo = chrono.ChRandomParticleVelocityAnyDirection()
mangvelo.SetModulusDistribution(chrono.ChUniformDistribution(0.0, 0.2))
emitter.SetParticleAngularVelocity(mangvelo)

mcreator_spheres = chrono.ChRandomShapeCreatorSpheres()
mcreator_spheres.SetDiameterDistribution(chrono.ChZhangDistribution(0.6, 0.23))
mcreator_spheres.SetDensityDistribution(chrono.ChConstantDistribution(1600))
emitter.SetParticleCreator(mcreator_spheres)

# -------------------------------
# VISUALIZATION
# -------------------------------
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Particle emitter demo')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 14, -20))
vis.AddTypicalLights()

mcreation_callback = MyCreatorForAll(vis)
emitter.RegisterAddBodyCallback(mcreation_callback)

sys.SetSolverType(chrono.ChSolver.Type_PSOR)
sys.GetSolver().AsIterative().SetMaxIterations(40)
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

# -------------------------------
# SIMULATION LOOP
# -------------------------------
stepsize = 1e-2

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    emitter.EmitParticles(sys, stepsize)

    for body in sys.Get_bodylist():
        body.EmptyAccumulators()

    G_constant = 6.674e-3  # Modified gravitational constant

    # Calculate the total kinetic energy of the system
    kinetic_energy = 0
    for body in sys.Get_bodylist():
        mass = body.GetMass()
        velocity = body.GetPos_dt()
        kinetic_energy += 0.5 * mass * velocity.Length2()

    # Calculate the total potential energy of the system (using gravitational potential)
    potential_energy = 0
    body_list = list(sys.Get_bodylist())
    for abodyA, abodyB in combinations(body_list, 2):
        D_attract = abodyB.GetPos() - abodyA.GetPos()
        r_attract = D_attract.Length()
        if r_attract > 1e-8:  # Avoid division by zero
            potential_energy += -G_constant * (abodyA.GetMass() * abodyB.GetMass()) / r_attract

    # Total energy of the system
    total_energy = kinetic_energy + potential_energy

    # Apply mutual gravitational forces
    for abodyA, abodyB in combinations(body_list, 2):
        D_attract = abodyB.GetPos() - abodyA.GetPos()
        r_attract = D_attract.Length()
        if r_attract > 1e-8:
            f_attract = G_constant * (abodyA.GetMass() * abodyB.GetMass()) / (r_attract ** 2)
            F_attract = (D_attract / r_attract) * f_attract
            abodyA.Accumulate_force(F_attract, abodyA.GetPos(), False)
            abodyB.Accumulate_force(F_attract * -1, abodyB.GetPos(), False)  # Use * -1 for negation

    # Print the energy values
    print(
        f"Kinetic Energy: {kinetic_energy:.6f}, Potential Energy: {potential_energy:.6f}, Total Energy: {total_energy:.6f}")

    sys.DoStepDynamics(stepsize)