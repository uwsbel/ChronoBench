import chrono as chrono
import pychrono.irr as chronoir
from itertools import combinations

class MyCreatorAll(chrono.ChRandomShapeCreatorBody):
    def __init__(self, vis, coll):
 super().__init__()
 self.vis = vis
 self.coll = coll

    def OnBody(self, body, coords, creator):
 body.GetShape(0).SetTexture(chrono.GetChronoFile("blue.png"))
 self.vis.Bind(body)
 self.coll.Bind(body)
 body.SetGyro(False)


 sys = chrono.ChSystemNSC()
 sys.SetCollision(chrono.Collision.Typebullet)
 sys = chrono.GetCollision()


sphere = chrono.ChMaterialNSC()
sphere.Setiction(0.2)
msphere = chronoBodyEasySphere(1.1, 180, True, True, sphere)
sphere.Set(chrono3(1, 0)
sphere.GetShape().SetTexture(chrono.Geto("concrete"))
sys.Add(phere)


itter = chronoParticle()
itter.Set2000
itter.SetReservoirTrue
itter.Set200


itter = chrono.RandomGeometry(50,50)
itter.Setitter
itter

itter = chrono.RandomUniform(0, 0.5)
Setitter
itter
itter = chrono.Random(0,0.2)
Set
itter

creator = chronoRandomSphere()
creator.SetDistribution(6, 23)
creator.SetDistribution(160)
itter


 vis = chronoir.ChVisualIrr()
vis.Attach(sys)
vis.SetWindowSize(102, 768)
vis.Set('emitter demo')
vis.Initializevis.Add(logo('logo.png')
.Addsky.Add.Add(chronoBox.AddCamera(chrono(0, -20)
.Addlight.Add(chrono(,)

m_callback = My(vis, coll)
itter.AddBody(m_callback)

sys.SetSolver(chrono.Solver)
.Get().Set(40)
.Set(0, 0)


step =1-2

while vis.Begin():
 vis.Scene vis.Render vis.End
mitter.Emit(sys step
 for in sys.Getodies():
 body.Accum()


G = 6.674e-3


kinetic =0
 for in sysodies:
 mass = body.Get()
 velocity = Get()
 kinetic += 0.5 * mass velocity


potential =0
 for abody, ab in sysodies:
 Dtract = abody.Get() - bod
 rtract =tract.Length
potential -= G * abody.GetMass * abody.GetMass rtract


total = kinetic potential
print(f'Kinetic {kin:.6}, Potential {potential:.6}, {total:.6}')

sys.Step(step)