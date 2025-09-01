import pychrono.core as chrono
import pychrono.irrlicht as chronoirr


sys = chrono.ChSystem()


vis = chronoirr.ChVisualSystem()
vis.SetWindowSize(800, 600)
vis.SetWindowTitle("Mass-Spring-Damper System")
vis.AddSkyBox()
vis.AddTypicalLights()


vis.AddCamera(chrono.ChVectorD(0, 0, 5), chrono.ChVectorD(0, 0, 0))


ground = chrono.ChBodyEasyBox(sys, 10, 1, 10, 1000, True, False)
ground.SetPos(chrono.ChVectorD(0, -5, 0))
sys.Add(ground)


mass = chrono.ChBodyEasySphere(sys, 1, 1000, True, False)
mass.SetPos(chrono.ChVectorD(0, 0, 0))
sys.Add(mass)


link = chrono.ChLinkTSDA()
link.Initialize(ground, mass, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 0))
sys.Add(link)


link.SetSpringCoefficient(100)
link.SetDampingCoefficient(5)
link.SetUnstretchedLength(5)


vis.AddSphere(mass.GetPos(), mass.GetRadius(), chrono.ChVectorD(1, 0, 0))
vis.AddLine(chrono.ChVectorD(0, -5, 0), mass.GetPos(), chrono.ChVectorD(0, 1, 0))


vis.Run()

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

def main():
    
    sys = chrono.ChSystem()

    
    vis = chronoirr.ChVisualSystem()
    vis.SetWindowSize(800, 600)
    vis.SetWindowTitle("Mass-Spring-Damper System")
    vis.AddSkyBox()
    vis.AddTypicalLights()

    
    vis.AddCamera(chrono.ChVectorD(0, 0, 5), chrono.ChVectorD(0, 0, 0))

    
    ground = chrono.ChBodyEasyBox(sys, 10, 1, 10, 1000, True, False)
    ground.SetPos(chrono.ChVectorD(0, -5, 0))
    sys.Add(ground)

    
    mass = chrono.ChBodyEasySphere(sys, 1, 1000, True, False)
    mass.SetPos(chrono.ChVectorD(0, 0, 0))
    sys.Add(mass)

    
    link = chrono.ChLinkTSDA()
    link.Initialize(ground, mass, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 0))
    sys.Add(link)

    
    link.SetSpringCoefficient(100)
    link.SetDampingCoefficient(5)
    link.SetUnstretchedLength(5)

    
    vis.AddSphere(mass.GetPos(), mass.GetRadius(), chrono.ChVectorD(1, 0, 0))
    vis.AddLine(chrono.ChVectorD(0, -5, 0), mass.GetPos(), chrono.ChVectorD(0, 1, 0))

    
    vis.Run()

if __name__ == "__main__":
    main()