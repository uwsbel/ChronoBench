import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np


chrono.SetChronoDataPath('PATH_TO_CHRONO_DATA')  


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('ANCF Beam Simulation')
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 0.5, 2))
vis.AddTypicalLights()


timestep = 0.001
sim_time = 5.0
output_fps = 60
output_step = 1.0 / output_fps


num_elements = 10
length = 1.0  
radius = 0.02  
density = 1000.0  
young_modulus = 1e6  
poisson_ratio = 0.3  


beam = chrono.ChBeamANCF()
beam.SetNumElements(num_elements)
beam.SetElementLength(length / num_elements)
beam.SetBeamRadius(radius)
beam.SetDensity(density)
beam.SetYoungModulus(young_modulus)
beam.SetPoissonRatio(poisson_ratio)


nodes = []
for i in range(num_elements + 1):
    x = i * length / num_elements
    pos = chrono.ChVectorD(x, 0, 0)
    node = chrono.ChNodeFEAxyzD()
    node.SetPos(pos)
    nodes.append(node)
    system.Add(node)


for i in range(num_elements):
    beam.AddElement(nodes[i], nodes[i+1])


system.AddLink(chrono.ChLinkLockLock(
    chrono.ChLinkLockLock.TypeLinkLockLock.LOCKLOCK_LOCKLOCK_LOCKLOCK,
    nodes[0].Variables(),
    chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT)
))


system.Add(beam)


for node in nodes:
    vis.AddPointLight(node.GetPos(), 1.0, chrono.ChColor(1, 0, 0))

beam_asset = chrono.ChBeamVisualizationANCF()
beam_asset.SetBeam(beam)
vis.AddVisualModel(beam_asset)


current_time = 0
while vis.Run():
    vis.BeginScene()
    vis.Render()

    
    system.DoStepDynamics(timestep)
    current_time += timestep

    
    if current_time >= output_step:
        print(f"Time: {current_time:.2f} s")
        for i, node in enumerate(nodes):
            pos = node.GetPos()
            print(f"  Node {i}: ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f})")
        output_step += 1.0 / output_fps

    vis.EndScene()

    if current_time >= sim_time:
        break