import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math

def main():
    
    
    
    chrono.SetChronoDataPath(chrono.GetChronoDataPath() + 'vehicle/')
    system = chrono.ChSystemNSC()
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    system.SetSolverType(chrono.ChSolver.Type_PSOR)
    system.SetSolverMaxIterations(150)
    system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))

    
    
    
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChMaterialSurfaceNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch = terrain.AddPatch(patch_mat, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 1), 200, 200)
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    
    
    
    bus = veh.CityBus()
    bus.SetContactMethod(chrono.ChContactMethod_NSC)
    bus.SetChassisCollisionType(veh.CollisionType_NONE)
    bus.SetChassisFixed(False)
    bus.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1.0), chrono.QUNIT))
    bus.SetTireType(veh.TireModelType_TMEASY)
    bus.SetTireStepSize(0.001)
    bus.Initialize()

    
    bus.SetChassisVisualizationType(veh.VisualizationType_MESH)
    bus.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    bus.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    bus.SetWheelVisualizationType(veh.VisualizationType_MESH)
    bus.SetTireVisualizationType(veh.VisualizationType_MESH)

    
    
    
    vis = irr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle('CityBus Simulation')
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVectorD(0, 10, 2), chrono.ChVectorD(0, 0, 1))
    vis.AddTypicalLights()
    vis.AddLightWithShadow(chrono.ChVectorD(-10, 10, 10), chrono.ChVectorD(0, 0, 0), 50, 10, 50, 60, 512)
    vis.EnableShadows()

    
    
    
    driver = veh.ChIrrGuiDriver(vis)
    driver.SetSteeringDelta(0.02)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.06)
    driver.Initialize()

    
    
    
    step_size = 0.02  
    realtime_timer = chrono.ChRealtimeStepTimer()
    bus.GetVehicle().EnableRealtime(True)

    while vis.Run():
        time = system.GetChTime()
        
        
        driver_inputs = driver.GetInputs()
        bus.GetVehicle().Synchronize(time, driver_inputs, terrain)
        terrain.Synchronize(time)
        
        
        vis.BeginScene()
        vis.Render()
        vis.RenderFrame(chrono.ChFrameD(), 2.0)  
        vis.EndScene()
        system.DoStepDynamics(step_size)
        realtime_timer.Spin(step_size)

        
        cam_pos = bus.GetVehicle().GetChassis().GetPos() + chrono.ChVectorD(-10, 5, 3)
        vis.GetActiveCamera().setTarget(irr.vector3df(bus.GetVehicle().GetChassis().GetPos().x,
                                                    bus.GetVehicle().GetChassis().GetPos().y,
                                                    bus.GetVehicle().GetChassis().GetPos().z))
        vis.GetActiveCamera().setPosition(irr.vector3df(cam_pos.x, cam_pos.y, cam_pos.z))

if __name__ == '__main__':
    main()