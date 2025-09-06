import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea
import numpy as np


chrono.SetChronoDataPath("PATH_TO_CHRONO_DATA")  


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  



beam_length = 1.0  
beam_radius = 0.05  
beam_Young_modulus = 2.1e11  
beam_density = 7850  
beam_poisson_ratio = 0.3


beam_mesh = fea.ChMesh()


node0 = fea.ChNodeFEAxyzrot(chrono.ChVectorD(0, 0, 0))
node1 = fea.ChNodeFEAxyzrot(chrono.ChVectorD(beam_length/2, 0, 0))
node2 = fea.ChNodeFEAxyzrot(chrono.ChVectorD(beam_length, 0, 0))
beam_mesh.AddNode(node0)
beam_mesh.AddNode(node1)
beam_mesh.AddNode(node2)


beam_element = fea.ChElementBeamIGA()
beam_element.SetNodes(node0, node1, node2)


beam_section = fea.ChBeamSectionAdvanced()
beam_section.SetDiameter(beam_radius)
beam_section.SetYoungModulus(beam_Young_modulus)
beam_section.SetDensity(beam_density)
beam_section.SetPoissonRatio(beam_poisson_ratio)
beam_element.SetSection(beam_section)

beam_mesh.AddElement(beam_element)


system.Add(beam_mesh)


flywheel_radius = 0.2  
flywheel_mass = 10.0   
flywheel_inertia = chrono.ChVectorD(0.5*flywheel_mass*flywheel_radius**2,  
                                    0.5*flywheel_mass*flywheel_radius**2,  
                                    flywheel_mass*flywheel_radius**2)      


flywheel = chrono.ChBodyEasyCylinder(beam_radius*2, flywheel_radius, flywheel_density)
flywheel.SetPos(chrono.ChVectorD(beam_length/2, 0, 0))
flywheel.SetMass(flywheel_mass)
flywheel.SetInertiaXX(flywheel_inertia)


link_flywheel = chrono.ChLinkLockLock()
link_flywheel.Initialize(flywheel, node1)
system.Add(link_flywheel)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(node0, node2)  
motor.SetMotorFunction(chrono.ChFunction_Const(10.0))  
system.Add(motor)



vis = chronoirr.ChIrrApp(system, "Jeffcott Rotor Simulation", chronoirr.dimension2du(1024, 768))
vis.AddTypicalLogo()
vis.AddTypicalSky()
vis.AddTypicalLights()
vis.AddTypicalCamera(chrono.ChVectorD(beam_length*1.5, 0, 0))
vis.SetTimestep(0.01)


beam_vis = fea.ChVisualizationFEAmesh(beam_mesh)
beam_vis.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_NODE_SPEED_NORM)
beam_vis.SetColorscaleMinMax(0.0, 5.0)
beam_vis.SetSmoothFaces(True)
beam_vis.SetWireframe(False)
beam_mesh.AddAsset(beam_vis)


flywheel_vis = chrono.ChColorAsset(0.5, 0.5, 0.8)
flywheel.AddAsset(flywheel_vis)


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.DoStep()
    vis.EndScene()

    
    time = system.GetChTime()
    if time > 0.1 and (int(time*10) % 10 == 0):  
        print(f"Time: {time:.2f}s")
        print(f"Flywheel position: {flywheel.GetPos()}")
        print(f"Motor speed: {motor.GetMotorFunction().Get_y(time)} rad/s")