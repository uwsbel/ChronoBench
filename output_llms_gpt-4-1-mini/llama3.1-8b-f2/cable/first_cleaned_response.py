import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.cascade as cascade
from OCC.Core import TopoDS







var_materials : bool = False



def main() :
    

    
    

    
    beam = cascade.ANCFBeamCable(assembly, TopoDS.TopoDS_CompSolid(rod1), L, E, A, 3, True, False)
    beam.SetDamping(0)
    beam.Initialize()

    
    ground = chrono.ChBodyEasyBox(10, 10, 1, 1000, True, True)
    ground.SetPos(chrono.ChVector3d(0,0,0))
    ground.SetFixed(True)
    ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile('textures/blue.png'))
    ground.Initialize()

    
    block = chrono.ChBodyEasyBox(1,1,1, 1000, True, True)
    block.SetPos(chrono.ChVector3d(0,-10,0))
    block.SetRot(chrono.ChQuaterniond(1,0,0,0))
    block.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile('textures/red.png'))
    block.Initialize()

    
    link = chrono.ChLinkLockpin()
    link.Initialize(block, assembly, chrono.ChFramed(chrono.ChVector3d(0,0.5,0), chrono.ChQuaterniond(1,0,0,0)))
    assembly.AddLink(link)

    
    vis = cascade.ChCascadeVisualSystemIrrlicht()
    vis.SetWindowTitle('ANCFSHEAR')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 0.75), 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AttachSystem(assembly.GetSystem())

    
    driver = cascade.ChCascadeInteractiveDriverIRR(vis)

    
    driver.SetCameraOrbitSpeed(0.2)
    driver.SetCameraZoomSpeed(0.2)
    driver.SetChaseCameraSpeed(4.0)
    driver.Initialize()

    
    
    while vis.Run() :

        time = assembly.GetSystem().GetChTime()

        
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        
        driver_inputs = driver.GetInputs()

        
        driver.Synchronize(time)
        ground.Synchronize(time)
        block.Synchronize(time)
        beam.Synchronize(time, driver_inputs)
        vis.Synchronize(time, driver_inputs)

        
        driver.Advance(step_size)
        ground.Advance(step_size)
        block.Advance(step_size)
        beam.Advance(step_size)
        vis.Advance(step_size)

    return 0
  







p0 = chrono.ChVector3d(0, 0, 0.75)


L = 1.0


A = 0.02
J = 1.0e-6


E = 2e7


render_step_size = 1.0 / 50  


step_size = 1e-3


collision_system_precision = 1e-3


assembly = cascade.ChCascadeAssembly()


assembly.AddBody(ground)
assembly.AddBody(block)
assembly.AddBody(beam)


assembly.Initialize()


main()