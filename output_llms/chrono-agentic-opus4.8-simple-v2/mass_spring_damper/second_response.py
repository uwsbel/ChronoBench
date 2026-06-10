import pychrono.core as chrono                                       # core PyChrono module
import pychrono.irrlicht as chronoirr                                # Irrlicht renderer

rest_length = 1.5                                                    # natural (unstretched) spring length [m]
spring_coef = 50                                                     # spring stiffness k [N/m]
damping_coef = 1                                                     # viscous damping c [N*s/m]

# Custom force functor: evaluates spring force from k and c custom parameters.
class MySpringForce(chrono.ForceFunctor):                           # extends ChLinkTSDA::ForceFunctor
    def __init__(self):                                            # constructor
        chrono.ForceFunctor.__init__(self)                         # MUST call base ctor
    def evaluate(self, time, rest_len, length, vel, link):         # 9.0.0 override signature
        force = -spring_coef * (length - rest_len) - damping_coef * vel   # Hooke + linear damping
        return force                                               # return scalar force along the spring

sys = chrono.ChSystemNSC()                                          # non-smooth contact rigid-body system
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))        # no gravity: pure spring dynamics

# Ground body: fixed reference that anchors both springs and carries the marker spheres.
ground = chrono.ChBody()                                            # plain rigid body
ground.SetFixed(True)                                               # immovable anchor
ground.EnableCollision(False)                                       # no contact in this demo
sys.AddBody(ground)                                                 # register the ground

sph_1 = chrono.ChVisualShapeSphere(0.1)                            # marker sphere for spring_1 anchor
ground.AddVisualShape(sph_1, chrono.ChFramed(chrono.ChVector3d(-1, 0, 0)))   # left anchor at (-1,0,0)
sph_2 = chrono.ChVisualShapeSphere(0.1)                            # marker sphere for spring_2 anchor
ground.AddVisualShape(sph_2, chrono.ChFramed(chrono.ChVector3d(1, 0, 0)))    # right anchor at (1,0,0)

# body_1: first oscillating mass, hangs from the ground via spring_1 (direct coefficients).
body_1 = chrono.ChBody()                                            # dynamic mass body
body_1.SetMass(1)                                                   # mass [kg]
body_1.SetInertiaXX(chrono.ChVector3d(1, 1, 1))                     # diagonal inertia [kg*m^2]
body_1.SetPos(chrono.ChVector3d(-1, -3, 0))                         # initial position (stretched below anchor)
body_1.EnableCollision(False)                                       # no contact
sys.AddBody(body_1)                                                # register body_1
box_1 = chrono.ChVisualShapeBox(0.4, 0.4, 0.4)                     # cube visual for the mass
body_1.AddVisualShape(box_1)                                       # attach cube to body_1

# body_2: second oscillating mass, mirrors body_1 but driven by the custom force functor.
body_2 = chrono.ChBody()                                            # dynamic mass body
body_2.SetMass(1)                                                   # mass [kg]
body_2.SetInertiaXX(chrono.ChVector3d(1, 1, 1))                     # diagonal inertia [kg*m^2]
body_2.SetPos(chrono.ChVector3d(1, -3, 0))                          # initial position (mirror of body_1)
body_2.EnableCollision(False)                                       # no contact
sys.AddBody(body_2)                                                # register body_2
box_2 = chrono.ChVisualShapeBox(0.4, 0.4, 0.4)                     # cube visual for the mass
body_2.AddVisualShape(box_2)                                       # attach cube to body_2

# spring_1: ground <-> body_1 with direct spring/damping coefficients.
spring_1 = chrono.ChLinkTSDA()                                      # translational spring-damper-actuator
spring_1.Initialize(body_1, ground, True,                          # body-local attachment frames
                    chrono.ChVector3d(0, 0, 0),                    # attach at body_1 origin
                    chrono.ChVector3d(-1, 0, 0))                   # attach at ground left anchor
spring_1.SetRestLength(rest_length)                                # natural length
spring_1.SetSpringCoefficient(spring_coef)                         # k = 50 N/m
spring_1.SetDampingCoefficient(damping_coef)                       # c = 1 N*s/m
sys.AddLink(spring_1)                                              # register spring_1
spring_1.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 15))  # coil visual: radius, resolution, turns

# spring_2: ground <-> body_2 using the custom force functor instead of direct coefficients.
spring_2 = chrono.ChLinkTSDA()                                      # translational spring-damper-actuator
spring_2.Initialize(body_2, ground, True,                          # body-local attachment frames
                    chrono.ChVector3d(0, 0, 0),                    # attach at body_2 origin
                    chrono.ChVector3d(1, 0, 0))                    # attach at ground right anchor
spring_2.SetRestLength(rest_length)                                # natural length
force_functor = MySpringForce()                                    # instantiate the custom functor
spring_2.RegisterForceFunctor(force_functor)                       # use functor to compute the force
sys.AddLink(spring_2)                                              # register spring_2
spring_2.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 15))  # coil visual: radius, resolution, turns

vis = chronoirr.ChVisualSystemIrrlicht()                           # create Irrlicht visual system
vis.AttachSystem(sys)                                              # bind it to the physical system
vis.SetWindowSize(1280, 720)                                       # window resolution
vis.SetWindowTitle("Mass-Spring-Damper")                          # window title
vis.Initialize()                                                  # create the device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))  # PyChrono logo
vis.AddSkyBox()                                                    # sky box backdrop
vis.AddCamera(chrono.ChVector3d(0, 0, 6), chrono.ChVector3d(0, -2, 0))   # eye looks at the masses
vis.AddTypicalLights()                                             # standard two-light setup

time_step = 1e-3                                                   # integration step [s]
sim_end = 20.0                                                     # total simulated time [s]
render_fps = 50.0                                                  # target frames per second
render_every = max(1, round(1.0 / (render_fps * time_step)))      # physics steps per rendered frame
while vis.Run() and sys.GetChTime() < sim_end:                    # render once per frame until sim_end
    vis.BeginScene()                                              # start frame
    vis.Render()                                                  # draw the scene
    vis.EndScene()                                                # end frame
    for _ in range(render_every):                                # advance physics in a batch
        sys.DoStepDynamics(time_step)                            # step the dynamics
        if sys.GetChTime() >= sim_end:                          # stop at the horizon
            break
