import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

def main():
    #print("Copyright (c) 2017 projectchrono.org\nChrono version: ", CHRONO_VERSION , "\n\n")

    #  Create the system
    sys = chrono.ChSystemNSC()

    # Add a collision material
    material = chrono.ChContactMaterialNSC()
    material.SetFriction(0.3)
    material.SetCompliance(0.01)
    sys.GetContactMaterial0().SetMaterial0(material)

    # Create the ground body
    ground = chrono.ChBody()
    ground.SetFixed(True)
    sys.Add(ground)

    # Create the mass
    m1 = chrono.ChBody()
    m1.SetMass(100)
    m1.SetName('mass1')
    m1.SetInertiaXX(chrono.ChVector3d(1, 1, 1))
    m1.EnableCollision(True)
    m1.GetVisualShape(0).SetTexture(veh.GetDataFile('logo_pychrono_alpha.png'))
    sys.Add(m1)

    # Create the link
    link = chrono.ChLinkTSDA()
    link.SetSpringsDampers(100, 0.5, 0, 0)
    link.Initialize(m1, ground, chrono.ChFramed(0, 0, 0.15, 1, 0, 0))
    sys.Add(link)

    # Create the Irrlicht visualization
    vis = irr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1024,768)
    vis.SetWindowTitle('Mass-Spring-Damper')
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0, 0, 1.5), chrono.ChVector3d(0, 0.5, 0))
    vis.AddTypicalLights()

    vis.AddLineMarker(chrono.ChLine(-0.1, 0, 0, 0.1, 0, 0), 0.1, 'red')

    # Simulation loop
    time = 0
    time_step = 1e-3
    time_end = 10

    while (vis.Run()) :
        time = sys.GetChTime()
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        sys.DoStepDynamics(time_step)


# Main
main()