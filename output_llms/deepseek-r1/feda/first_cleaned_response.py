import os
import math
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

def main():
    
    
    
    chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), 'data/'))
    contact_method = chrono.ChContactMethod_SMC
    system = chrono.ChSystemSMC()
    system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))

    
    
    
    feda = veh.FEDA_Vehicle(False, contact_method)
    feda.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.5), chrono.Q_from_AngZ(0)))
    feda.Initialize()
    feda.SetChassisVisualizationType(veh.VisualizationType_MESH)
    feda.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
    feda.SetSteeringVisualizationType(veh.VisualizationType_MESH)
    feda.SetWheelVisualizationType(veh.VisualizationType_MESH)

    
    tire_vis = veh.VisualizationType_MESH
    for axle in [veh.LEFT, veh.RIGHT]:
        wheel = feda.GetWheel(axle, veh.FRONT)
        tire = veh.RigidTire("tire", 0.5)
        tire.Initialize(wheel)
        tire.SetVisualizationType(tire_vis)
        feda.GetSystem().Add(tire)

    
    
    
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChMaterialSurfaceSMC()
    patch = terrain.AddPatch(patch_mat, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 1), 100, 100)
    patch.SetTexture(veh.GetDataFile("terrain/textures/ground_1.png"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    
    
    
    vis = irr.ChIrrApp(system, "FEDA Vehicle Simulation", irr.dimension2du(1280, 720))
    vis.AddTypicalLights()
    vis.AddTypicalCamera(irr.vector3df(0, 0, 1))
    vis.EnableGrid(False)
    vis.AttachVehicle(feda.GetVehicle())

    
    vis.GetActiveCamera().setTarget(irr.vector3df(0, 0, 0))
    vis.GetActiveCamera().setPosition(irr.vector3df(-5, -8, 2))

    
    
    
    driver = veh.ChInteractiveDriverIRR(vis)
    driver.SetSteeringDelta(0.02)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.02)
    driver.Initialize()

    
    
    
    step_size = 0.02
    render_steps = math.ceil(1 / (50 * step_size))

    realtime_timer = chrono.ChRealtimeStepTimer()
    while vis.GetDevice().run():
        time = system.GetChTime()

        
        driver.Synchronize(time)
        feda.Synchronize(time, driver.GetInputs(), terrain)
        terrain.Synchronize(time)
        vis.Synchronize("", driver.GetInputs())

        
        system.DoStepDynamics(step_size)
        vis.Advance(step_size)

        
        if system.GetStepCount() % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.GetGUIEnvironment().drawAll()
            vis.EndScene()

        realtime_timer.Spin(step_size)

if __name__ == "__main__":
    main()