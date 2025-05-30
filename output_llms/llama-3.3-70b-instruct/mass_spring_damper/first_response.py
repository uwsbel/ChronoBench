# Import necessary modules
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# Create the simulation system
sys = chrono.ChSystem()

# Create the Irrlicht visualization window
vis = chronoirr.ChVisualSystem()
vis.SetWindowSize(800, 600)
vis.SetWindowTitle("Mass-Spring-Damper System")
vis.AddSkyBox()
vis.AddTypicalLights()

# Create the camera
vis.AddCamera(chrono.ChVectorD(0, 0, 5), chrono.ChVectorD(0, 0, 0))

# Create the ground body
ground = chrono.ChBodyEasyBox(sys, 10, 1, 10, 1000, True, False)
ground.SetPos(chrono.ChVectorD(0, -5, 0))
sys.Add(ground)

# Create the mass body
mass = chrono.ChBodyEasySphere(sys, 1, 1000, True, False)
mass.SetPos(chrono.ChVectorD(0, 0, 0))
sys.Add(mass)

# Create the mass-spring-damper connection
link = chrono.ChLinkTSDA()
link.Initialize(ground, mass, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 0))
sys.Add(link)

# Set default linear spring-damper parameters for the ChLinkTSDA connection
link.SetSpringCoefficient(100)
link.SetDampingCoefficient(5)
link.SetUnstretchedLength(5)

# Add visual shapes for the mass and spring
vis.AddSphere(mass.GetPos(), mass.GetRadius(), chrono.ChVectorD(1, 0, 0))
vis.AddLine(chrono.ChVectorD(0, -5, 0), mass.GetPos(), chrono.ChVectorD(0, 1, 0))

# Run the simulation
vis.Run()

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

def main():
    # Create the simulation system
    sys = chrono.ChSystem()

    # Create the Irrlicht visualization window
    vis = chronoirr.ChVisualSystem()
    vis.SetWindowSize(800, 600)
    vis.SetWindowTitle("Mass-Spring-Damper System")
    vis.AddSkyBox()
    vis.AddTypicalLights()

    # Create the camera
    vis.AddCamera(chrono.ChVectorD(0, 0, 5), chrono.ChVectorD(0, 0, 0))

    # Create the ground body
    ground = chrono.ChBodyEasyBox(sys, 10, 1, 10, 1000, True, False)
    ground.SetPos(chrono.ChVectorD(0, -5, 0))
    sys.Add(ground)

    # Create the mass body
    mass = chrono.ChBodyEasySphere(sys, 1, 1000, True, False)
    mass.SetPos(chrono.ChVectorD(0, 0, 0))
    sys.Add(mass)

    # Create the mass-spring-damper connection
    link = chrono.ChLinkTSDA()
    link.Initialize(ground, mass, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 0))
    sys.Add(link)

    # Set default linear spring-damper parameters for the ChLinkTSDA connection
    link.SetSpringCoefficient(100)
    link.SetDampingCoefficient(5)
    link.SetUnstretchedLength(5)

    # Add visual shapes for the mass and spring
    vis.AddSphere(mass.GetPos(), mass.GetRadius(), chrono.ChVectorD(1, 0, 0))
    vis.AddLine(chrono.ChVectorD(0, -5, 0), mass.GetPos(), chrono.ChVectorD(0, 1, 0))

    # Run the simulation
    vis.Run()

if __name__ == "__main__":
    main()