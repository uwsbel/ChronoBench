import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import math

def main():
    
    
    
    step_size = 0.005
    fps = 50
    render_step = 1.0 / fps

    
    
    
    system = chrono.ChSystemNSC()
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    system.SetGravity(chrono.ChVectorD(0, 0, -9.81))

    
    
    
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChMaterialSurfaceNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysD(), 200, 100)
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    
    
    
    bus = veh.CityBus()
    bus.SetContactMethod(chrono.ChContactMethod_NSC)
    bus.SetChassisFixed(False)
    bus.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1.0), chrono.Q_from_AngY(0)))
    bus.SetTireType(veh.TireModelType_TMEASY)
    bus.SetTireStepSize(step_size)
    bus.Initialize()

    
    bus.SetChassisVisualizationType(veh.VisualizationType_MESH)
    bus.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    bus.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    bus.SetWheelVisualizationType(veh.VisualizationType_MESH)
    bus.SetTireVisualizationType(veh.VisualizationType_MESH)

    
    
    
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle('CityBus Simulation')
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVectorD(10, 10, 2.0), chrono.ChVectorD(0, 0, 0.7))
    vis.AddTypicalLights()
    vis.EnableShadows()

    
    
    
    driver = veh.ChIrrGuiDriver(vis)
    driver.SetSteeringDelta(0.02)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.06)
    driver.Initialize()

    
    
    
    realtime_timer = chrono.ChRealtimeStepTimer()
    frame_number = 0

    while vis.Run():
        time = system.GetChTime()
        
        
        driver_inputs = driver.GetInputs()
        bus.Synchronize(time, driver_inputs, terrain)
        terrain.Synchronize(time)
        
        
        system.DoStepDynamics(step_size)
        
        
        if frame_number % int(render_step / step_size) == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
        
        
        cam_target = bus.GetChassis().GetPos()
        vis.GetActiveCamera().setTarget(chronoirr.vector3df(cam_target.x, cam_target.y, cam_target.z))
        cam_pos = bus.GetChassis().GetPos() + chrono.ChVectorD(-10, 0, 3)
        vis.GetActiveCamera().setPosition(chronoirr.vector3df(cam_pos.x, cam_pos.y, cam_pos.z))
        
        frame_number += 1
        realtime_timer.Spin(step_size)

if __name__ == "__main__":
    main()