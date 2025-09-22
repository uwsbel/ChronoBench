import pychrono as chrono
import pychrono.irrlicht as chronoirr
from itertools import combinations

# Callback class for actions to perform on each newly created particle
class MyCreatorForAll(chrono.ChRandomShapeCreator_AddBodyCallback):
    def __init__(self, vis_system):
        super().__init__()
        self.vis_system = vis_system

    def OnAddBody(self, body, coords, creator):
        # Set a texture for the visual shape of the created body.
        # This assumes the body has at least one visual shape, and we're texturing the first one.
        # For spheres created by ChRandomShapeCreatorSpheres, this is a safe assumption.
        if body.GetVisualModel() and body.GetVisualModel().GetNumShapes() > 0:
            visual_shape = body.GetVisualShape(0)
            if visual_shape: # Ensure the visual shape pointer is valid
                visual_shape.SetTexture(chrono.GetChronoDataFile("textures/bluewhite.png"))
        
        # Bind the newly created body to the Irrlicht visualization system so it can be rendered.
        self.vis_system.BindItem(body)
        
        # Corrected Error: Removed 'self.coll.BindItem(body)'
        # ChCollisionSystem (referenced by 'coll' in the original script) does not have a 'BindItem' method.
        # Collision aspects are handled internally when a body with collision models is added to the ChSystem.

        # Set gyroscopic torque usage (as in original script)
        body.SetUseGyroTorque(False)

# Create a Chrono physical system
sys = chrono.ChSystemNSC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
# The variable 'coll' (sys.GetCollisionSystem()) is not strictly needed for the rest of the script
# after the correction in MyCreatorForAll.

# Create a large central sphere body (e.g., a planet or a large attractor)
sphere_mat = chrono.ChContactMaterialNSC()
sphere_mat.SetFriction(0.2) # Example friction for the large sphere
# ChBodyEasySphere parameters: radius, density, enable visualization, enable collision, material
msphereBody = chrono.ChBodyEasySphere(2.1, 1800, True, True, sphere_mat)
msphereBody.SetPos(chrono.ChVector3d(1, 1, 0)) # Initial position of the large sphere
# Set a specific texture for the large sphere's visual shape
if msphereBody.GetVisualModel() and msphereBody.GetVisualModel().GetNumShapes() > 0:
    visual_shape = msphereBody.GetVisualShape(0)
    if visual_shape:
        visual_shape.SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
sys.Add(msphereBody) # Add the large sphere to the system

# Create a particle emitter
emitter = chrono.ChParticleEmitter()
emitter.SetParticlesPerSecond(2000) # Rate of particle emission
emitter.SetUseParticleReservoir(True) # Use a reservoir to ensure particles can always be emitted
emitter.SetParticleReservoirAmount(200) # Number of particles in the reservoir

# Configure randomizers for particle properties:

# Position: Particles are created on the surface of a large box.
# ChBox constructor takes half-dimensions, so this is a 100x100x100 box.
emitter_positions = chrono.ChRandomParticlePositionOnGeometry()
emitter_positions.SetGeometry(chrono.ChBox(50, 50, 50), chrono.ChFramed())
emitter.SetParticlePositioner(emitter_positions)

# Alignment (Rotation): Particles get a uniform random initial rotation.
emitter_rotations = chrono.ChRandomParticleAlignmentUniform()
emitter.SetParticleAligner(emitter_rotations)

# Linear Velocity: Particles get a random velocity vector with modulus in [0.0, 0.5].
mvelo = chrono.ChRandomParticleVelocityAnyDirection()
mvelo.SetModulusDistribution(chrono.ChUniformDistribution(0.0, 0.5))
emitter.SetParticleVelocity(mvelo)

# Angular Velocity: Particles get a random angular velocity vector with modulus in [0.0, 0.2].
mangvelo = chrono.ChRandomParticleVelocityAnyDirection() # Same class can be used for angular velocity
mangvelo.SetModulusDistribution(chrono.ChUniformDistribution(0.0, 0.2))
emitter.SetParticleAngularVelocity(mangvelo)

# --- Instruction 1: Replace the Particle Creator ---
# The original ChRandomShapeCreatorConvexHulls is replaced with ChRandomShapeCreatorSpheres.

# Create a sphere shape creator for particles
mcreator_spheres = chrono.ChRandomShapeCreatorSpheres()
# Configure diameter distribution for the spheres using ChZhangDistribution.
mcreator_spheres.SetDiameterDistribution(chrono.ChZhangDistribution(0.6, 0.23))
# Configure density distribution for the spheres using ChConstantDistribution.
mcreator_spheres.SetDensityDistribution(chrono.ChConstantDistribution(1600))
# Set this sphere creator for the emitter.
emitter.SetParticleCreator(mcreator_spheres)
# --- End of Instruction 1 ---

# Create the Irrlicht visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('PyChrono Particle Emitter: N-Body Gravity & Energy') # Updated window title
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 14, -20)) # Camera position
vis.AddTypicalLights()

# Register the callback with the emitter.
# MyCreatorForAll will be called for each particle created by the emitter.
mcreation_callback = MyCreatorForAll(vis) # Pass the visualization system to the callback
emitter.RegisterAddBodyCallback(mcreation_callback)

# Simulation settings
sys.SetSolverType(chrono.ChSolver.Type_PSOR) # Solver type
sys.GetSolver().AsIterative().SetMaxIterations(40) # Max iterations for the solver
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0)) # Disable Chrono's global gravity; custom gravity is implemented

# Simulation loop parameters
stepsize = 1e-2 # Timestep for the simulation
# gravity_eps is a small distance to prevent division by zero or extremely large forces
# in gravitational calculations. Pairs of bodies closer than this distance will not
# have their gravitational interaction computed by the simple 1/r^2 and 1/r formulas.
# This is a form of "softening" or a cutoff.
gravity_eps = 1e-3 

# For controlling print frequency of energy values
print_interval = 0.1  # Print energy information approximately every 0.1 simulation seconds
last_print_time = -print_interval # Initialize to allow printing at t=0

while vis.Run():
    current_sim_time = sys.GetChTime()

    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Emit new particles into the system based on the time elapsed (stepsize)
    emitter.EmitParticles(sys, stepsize)

    # --- Instruction 3: Adjust Position of G_constant ---
    # G_constant is defined inside the loop, just before it's used for energy/force calculations.
    # Units of G_constant depend on the units chosen for mass, length, and time.
    # E.g., if mass in kg, length in m, time in s, then G = 6.674e-11 N m^2 / kg^2.
    # The value 6.674e-3 is a scaled version for simulation purposes.
    G_constant = 6.674e-3

    # --- Instruction 2: Add Energy Calculations ---
    kinetic_energy = 0.0
    potential_energy = 0.0
    
    # Get a list of all bodies currently in the system
    all_bodies_list = sys.GetBodies() # This returns a SWIG-wrapped std::vector, iterable in Python

    # Calculate Kinetic Energy of all bodies
    for body in all_bodies_list:
        if body.IsActive() and body.GetMass() > 0:
            lin_vel = body.GetLinVel()
            kinetic_energy += 0.5 * body.GetMass() * lin_vel.Length2()
            # Note: Rotational kinetic energy (0.5 * I * omega^2) is not included here for simplicity,
            # but could be added for more comprehensive energy tracking.

    # Clear force and torque accumulators for all bodies before applying new forces
    for body in all_bodies_list:
        body.EmptyAccumulators()

    # Calculate Gravitational Forces and Potential Energy between all unique pairs of bodies
    body_pairs = combinations(all_bodies_list, 2)

    for body_A, body_B in body_pairs:
        # Ensure bodies are valid and have mass (though typically they should from the emitter/setup)
        if not body_A or not body_B or body_A.GetMass() <= 0 or body_B.GetMass() <= 0:
            continue

        D_vec = body_B.GetPos() - body_A.GetPos() # Vector from body_A to body_B
        r_sq = D_vec.Length2() # Squared distance
        
        # Corrected Error: Prevent division by zero and instability from
print("error happened with only start ```python")