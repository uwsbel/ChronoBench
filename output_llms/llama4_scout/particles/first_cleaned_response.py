import chrono as ch
import chrono.irrlicht as irr
import random
import math


ch.ChEngine.Initialize()


app = irr.ChIrrApp()


step_size = 0.01
frame_rate = 60


system = ch.ChSystemNSC()


emitter = ch.ChParticleEmitter()


emitter.SetEmissionRate(100)  
emitter.SetInitialSpeed(10)
emitter.SetInitialSize(0.1)
emitter.SetRadius(1)


system.Add(emitter)


class CustomGravity(ch.ChForce):
    def __init__(self, body):
        super(CustomGravity, self).__init__(body)

    def ComputeForce(self, body, M, x, v, t):
        F = ch.ChVectorD(0, 0, 0)
        for other_body in system.GetBodies():
            if other_body != body:
                r = other_body.GetPos() - body.GetPos()
                dist = r.Length()
                if dist > 0:
                    G = 0.1  
                    F += G * body.GetMass() * other_body.GetMass() * r / (dist ** 3)
        return F


def ApplyCustomGravity(body):
    force = CustomGravity(body)
    body.AddForce(force)


for body in system.GetBodies():
    ApplyCustomGravity(body)


for _ in range(1000):
    particle = ch.ChBodyEasySphere(1, 1000)
    particle.SetPos(ch.ChVectorD(random.uniform(-10, 10), random.uniform(-10, 10), random.uniform(-10, 10)))
    particle.SetRot(ch.Q_from_AxisAngle(ch.ChVectorD(1, 0, 0), random.uniform(0, 2 * math.pi)))
    particle.SetVelPos(ch.ChVectorD(random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1)))
    system.Add(particle)


vis = irr.ChIrrVisualization(system)


vis.SetVisibilityFlags(irr.ChIrrVisibilityFlags.VIS_BODY_FRAMES | irr.ChIrrVisibilityFlags.VIS_COLLISION_SHAPES)


camera = irr.ChIrrCamera()


camera.SetPosition(ch.ChVectorD(0, 0, 10))
camera.SetTarget(ch.ChVectorD(0, 0, 0))


vis.Add(camera)


app.SetSimulationStep(step_size)
app.SetFrameRate(frame_rate)
app.Run()

import chrono as ch
import chrono.irrlicht as irr
import random
import math

def main():
    
    ch.ChEngine.Initialize()

    
    app = irr.ChIrrApp()

    
    step_size = 0.01
    frame_rate = 60

    
    system = ch.ChSystemNSC()

    
    emitter = ch.ChParticleEmitter()

    
    emitter.SetEmissionRate(100)  
    emitter.SetInitialSpeed(10)
    emitter.SetInitialSize(0.1)
    emitter.SetRadius(1)

    
    system.Add(emitter)

    
    class CustomGravity(ch.ChForce):
        def __init__(self, body):
            super(CustomGravity, self).__init__(body)

        def ComputeForce(self, body, M, x, v, t):
            F = ch.ChVectorD(0, 0, 0)
            for other_body in system.GetBodies():
                if other_body != body:
                    r = other_body.GetPos() - body.GetPos()
                    dist = r.Length()
                    if dist > 0:
                        G = 0.1  
                        F += G * body.GetMass() * other_body.GetMass() * r / (dist ** 3)
            return F

    
    for _ in range(1000):
        particle = ch.ChBodyEasySphere(1, 1000)
        particle.SetPos(ch.ChVectorD(random.uniform(-10, 10), random.uniform(-10, 10), random.uniform(-10, 10)))
        particle.SetRot(ch.Q_from_AxisAngle(ch.ChVectorD(1, 0, 0), random.uniform(0, 2 * math.pi)))
        particle.SetVelPos(ch.ChVectorD(random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1)))
        system.Add(particle)

    
    vis = irr.ChIrrVisualization(system)

    
    vis.SetVisibilityFlags(irr.ChIrrVisibilityFlags.VIS_BODY_FRAMES | irr.ChIrrVisibilityFlags.VIS_COLLISION_SHAPES)

    
    camera = irr.ChIrrCamera()

    
    camera.SetPosition(ch.ChVectorD(0, 0, 10))
    camera.SetTarget(ch.ChVectorD(0, 0, 0))

    
    vis.Add(camera)

    
    app.SetSimulationStep(step_size)
    app.SetFrameRate(frame_rate)
    app.Run()

if __name__ == "__main__":
    main()