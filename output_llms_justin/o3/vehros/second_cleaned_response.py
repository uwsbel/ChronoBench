import pychrono as ch
from   pychrono import irrlicht as chronoirr          
import pychrono.vehicle as veh
import pychrono.ros     as chros


def main():
    
    
    
    ch.SetChronoDataPath(ch.GetChronoDataPath())                 
    veh.SetDataPath(ch.GetChronoDataPath() + 'vehicle/')         

    contact_method = ch.ChContactMethod_NSC

    
    
    
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(contact_method)

    
    hmmwv.SetChassisCollisionType(veh.ChassisCollisionType_NONE)
    hmmwv.SetChassisFixed(False)

    init_loc  = ch.ChVectorD(0.0, 0.0, 1.6)
    init_rot  = ch.ChQuaternionD(1.0, 0.0, 0.0, 0.0)
    hmmwv.SetInitPosition(ch.ChCoordsysD(init_loc, init_rot))

    hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
    hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
    hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
    hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
    hmmwv.SetTireType(veh.TireModelType_TMEASY)
    hmmwv.SetTireStepSize(1e-3)

    
    hmmwv.SetChassisVisualizationType     (veh.VisualizationType_PRIMITIVES)
    hmmwv.SetSuspensionVisualizationType  (veh.VisualizationType_PRIMITIVES)
    hmmwv.SetSteeringVisualizationType    (veh.VisualizationType_PRIMITIVES)
    hmmwv.SetWheelVisualizationType       (veh.VisualizationType_PRIMITIVES)
    hmmwv.SetTireVisualizationType        (veh.VisualizationType_MESH)

    hmmwv.Initialize()

    
    
    
    terrain = veh.RigidTerrain(hmmwv.GetSystem())
    patch_mat = ch.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)

    patch = terrain.AddPatch(
        patch_mat,
        ch.ChCoordsysD(ch.ChVectorD(0.0, 0.0, 0.0), ch.ChQuaternionD(1.0, 0.0, 0.0, 0.0)),
        100.0, 100.0)

    
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 100, 100)
    terrain.Initialize()

    
    
    
    driver = veh.ChDriver(hmmwv.GetVehicle())
    driver.Initialize()

    
    
    
    ros_manager = chros.ChROSPythonManager()
    ros_manager.RegisterHandler(chros.ChROSClockHandler())
    ros_manager.RegisterHandler(
        chros.ChROSDriverInputsHandler(25, driver, "~/input/driver_inputs"))
    ros_manager.RegisterHandler(
        chros.ChROSBodyHandler  (25, hmmwv.GetChassisBody(), "~/output/hmmwv/state"))
    ros_manager.Initialize()

    
    
    
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(hmmwv.GetSystem())
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("HMMWV + ROS demo")
    vis.Initialize()
    vis.AddLogo()
    vis.AddSkyBox()
    vis.AddTypicalLights()
    vis.AddTypicalCamera(ch.ChVectorD(0.0, -6.0, 3.0))
    vis.SetCameraVertical(ch.ChVectorD(0, 0, 1))

    
    
    
    time_step   = 1e-3
    time_end    = 30.0
    render_step = 10          
    out_step    = 0

    hmmwv.GetVehicle().EnableRealtime(True)

    while vis.Run() and (hmmwv.GetSystem().GetChTime() < time_end):
        time = hmmwv.GetSystem().GetChTime()

        
        driver_inputs = driver.GetInputs()
        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)

        
        driver.Advance(time_step)
        terrain.Advance(time_step)
        hmmwv.Advance(time_step)

        
        if not ros_manager.Update(time, time_step):
            break

        
        if out_step % render_step == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
        out_step += 1

    
    ros_manager.Shutdown()



if __name__ == "__main__":
    main()