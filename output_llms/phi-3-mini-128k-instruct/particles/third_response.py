```python
import pychrono as chrono
import pychrono.irrlicht as chronoirr
from itertools import combinations

class MyCreatorForAll(chrono.ChRandomShapeCreator_AddBodyCallback):
    def __init__(self, vis, coll):
        super().__init__()
        self.vis = vis
        self.coll = coll

    def OnAddBody(self, body, coords, creator):
        body.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/bluewhite.png"))
        self.vis.BindItem(body)
        self.coll.BindItem(body)
        body.SetUseGyroTorque(False)

# Create a Chrono physical system
sys = chrono.ChSystemNSC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
coll = sys.GetCollisionSystem()

# Create a sphere body
sphere_mat = chrono.ChContactMaterialNSC()
sphere_mat.SetFriction(0.2)
msphereBody = chrono.ChBodyEasySphere(2.1, 1800, True, True, sphere_mat)
msphereBody.SetPos(chrono.ChVector3d(1, 1, 0))
msphereBody.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
sys.Add(msphereBody)

# Create an emitter
emitter = chrono.ChParticleEmitter()
emitter.SetParticlesPerSecond(2000)
emitter.SetUseParticleReservoir(True)
emitter.SetParticleReservoirAmount(200)

# Randomizers for particle properties
emitter_positions = chrono.ChRandomParticlePositionOnGeometry()
emitter_positions.SetGeometry(chrono.ChBox(50, 50, 50), chrono.ChFramed())
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

# Create the Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Particle emitter demo')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 14, -20))
vis.AddTypicalLights()

mcreation_callback = MyCreatorForAll(vis, coll)
emitter.RegisterAddBodyCallback(mcreation_callback)

sys.SetSolverType(chrono.ChSolver.Type_PSOR)
sys.GetSolver().AsIterative().SetMaxIterations(40)
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

# Additional spheres for a three-body simulation
msphereBody2 = chrono.ChBodyEasySphere(2.1, 1800, True, True, sphere_mat)
msphereBody2.SetPos(chrono.ChVector3d(-10, -10, 0))
msphereBody2.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
sys.Add(msphereBody2)

msphereBody3 = chrono.ChBodyEasySphere(2.1, 1800, True, True, sphere_mat)
msphereBody3.SetPos(chrono.ChVector3d(0, 20, 0))
msphereBody3.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
sys.Add(msphereBody3)

# Set initial velocities for the three spheres
msphereBody.SetVel(chrono.ChVectorD(0.5, 0, 0.1))
msphereBody2.SetVel(chrono.ChVectorD(-0.5, 0, -0.1))
msphereBody3.SetVel(chrono.ChVectorD(0, -0.5, 0.2))

# Simulation loop
stepsize = 1e-2

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    emitter.EmitParticles(sys, stepsize)

    for body in sys.GetBodies():
        body.EmptyAccumulators()

    G_constant = 6.674e-3  # Modified gravitational constant

    # Calculate the total kinetic energy of the system
    kinetic_energy = 0
    for body in sys.GetBodies():
        mass = body.GetMass()
        velocity = body.GetPosDt()
        kinetic_energy += 0.5 * mass * velocity.Length2()

    # Calculate the total potential energy of the system (using gravitational potential)
    potential_energy = 0
    for abodyA, abodyB in combinations(sys.GetBodies(), 2):
        D_attract = abodyB.GetPos() - abodyA.GetPos()
        r_attract = D_attract.Length()
        potential_energy += -G_constant * (abodyA.GetMass() * abodyB.GetMass()) / r_attract

    # Total energy of the system
    total_energy = kinetic_energy + potential_energy
    mlist = list(combinations(sys.GetBodies(),                _000












0000)


00)
0,00,00000l00s,
,
_


,       



0000_       )0)
00000000.,,_,,0,,,,,
3,2,0,
,,
,3,0,


)
)


,,
)
s0,020,s,,0000_0000.0,o
0,00000.
50,06,
0)
1.0.

l)
.
)
)
0,
,



,0,
,0,0)
)
)

0)
,)
)
)
)
f)
)00)
,0)1)
)
)0)
0)
)
0)0)0)   0)0)
)
)
)   )   )   )

)
)
)
)
)0)
   ):
)0)   0)1)
   l),_0))_))))s)
)
)
   
)
00.
)
)

   s))
0)0)
)
00)
)
               


   0)   )   0)



00,
r.re)
   



00)


320)
)))





40)
0)00)










0,
   0)   
         30()
         

         0   
   
   0)

00,   
   00,
   0)
   

         
   )
)s)
)1)
)
)0)
0,000)





   0,

   0,0,
   




,
   5,
,0,

      
   0,0,01,2,   ,
,
,   0,22d4   
   ,





,
)

,
)
)
0)
0)




0,







0000   



2


   0000         0.00.

   



00,





4)




)_










   0,

0,




00
















l
0030
000034.00

3,0,0,00
303,

300,
l.
.l,
2,


.
34,
0,
0
c2,

   00000






   

   
00.




0
e.

00.

   


r.




   
   0


   



)
)




















0)
,
d,




   




      

   

         
         




   
   
l)

0

l
r)
)


)
)r)
)
)

)
)
)
)



)



)

0

0)
)
0)
4





)
)


)





)

   




4)

0
   0)



0




)



)
)








4)
)














   

         
   


   





)
000
   
   03,
0.0)
   0         3.0   0   



,0,0,
   0,0,24,








0,
0)

)

,
   

   


      ,
               0


   0,


   0,
0,0,


0,0,
   0,
      
,
,

   


0)
0,
,
,















,
,












   0,



)


,0,


   
00   
   0,
   
   
   

   
   




   )
   0)