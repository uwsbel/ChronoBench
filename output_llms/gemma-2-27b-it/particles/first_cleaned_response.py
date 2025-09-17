import pychrono as chrono
import pychrono.irrlicht as chronoirr






mysystem = chrono.ChSystemNSC()
mysystem.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


material = chrono.ChMaterialSurfaceNSC()
material.SetFriction(0.3)
material.SetRestitution(0.2)


myapplication = chronoirr.ChIrrApp(mysystem, "Particle Attraction", chronoirr.dimension2du(800, 600))
myapplication.AddTypicalSky()
myapplication.AddTypicalCamera(chrono.ChVectorD(0, 5, 5))
myapplication.SetShowInfos(True)


emitter = chrono.ChParticleEmitter(mysystem)
emitter.SetPos(chrono.ChVectorD(0, 10, 0))
emitter.SetFlowRate(10)  
emitter.SetParticleRadius(0.1)
emitter.SetMaterial(material)


emitter.SetRandomShape(True)
emitter.SetRandomPos(chrono.ChVectorD(-2, 2, -2), chrono.ChVectorD(2, 2, 2))
emitter.SetRandomVel(chrono.ChVectorD(-1, 0, -1), chrono.ChVectorD(1, 0, 1))
emitter.SetRandomRot(chrono.ChQuaternionD(1, 0, 0, 0), chrono.ChQuaternionD(0, 1, 0, 0))






class ParticleAttraction:
    def __init__(self, strength):
        self.strength = strength

    def ApplyForce(self, particle1, particle2):
        r = particle2.GetPos() - particle1.GetPos()
        dist = r.Length()
        if dist > 0:
            force = self.strength * particle1.GetMass() * particle2.GetMass() / dist**2
            force_vec = r.GetNormalized() * force
            particle1.AddForce(force_vec)
            particle2.AddForce(-force_vec)

attraction = ParticleAttraction(10)


for i in range(mysystem.Get_Nbodies()):
    for j in range(i + 1, mysystem.Get_Nbodies()):
        particle1 = mysystem.Get_Body(i)
        particle2 = mysystem.Get_Body(j)
        if isinstance(particle1, chrono.ChParticle) and isinstance(particle2, chrono.ChParticle):
            mysystem.AddForce(chrono.ChForceCustom(particle1, particle2, attraction.ApplyForce))






myapplication.SetTimestep(0.01)
whilemyapplication.GetDevice().run():
    myapplication.BeginScene()
    myapplication.DrawAll()
    mysystem.DoStepDynamics(myapplication.GetTimestep())
    myapplication.EndScene()