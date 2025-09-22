import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh







viskin_camera = True

class MyReportContact(chrono.ReportContactCallback):
    def __init__(self):
        chrono.ReportContactCallback.__init__(self)
    def OnReportContact(self,vA,vB,cA,dist,rad,force,torque,modA,modB):
        bodA = chrono.CastContactable_to_ChBody(modA)
        bodB = chrono.CastContactable_to_ChBody(modB)
        print('  contact (A: {}, B: {})'.format(bodA.GetName(), bodB.GetName()))
        return True

def main():
    

    
    hmmwv = veh.HMMWV_Reduced()
    hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)
    hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
    hmmwv.SetChassisFixed(False) 
    hmmwv.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 1.6), chrono.ChQuaterniond(1, 0, 0, 0)))
    hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
    hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
    hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
    hmmwv.SetTireType(veh.TireModelType_TMEASY)
    hmmwv.SetTireStepSize(tire_step_size)
    hmmwv.Initialize()

    hmmwv.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv.SetWheelVisualizationType(veh.VisualizationType_NONE)
    hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

    hmmwv.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    
    ground = chrono.ChBody()
    ground.SetFixed(True)
    ground.SetPos(chrono.ChVector3d(0,0,-0.1))

    
    groundMat = chrono.ChMatrix3d()
    groundMat.SetRotZ(-myramp_angle)
    ground.SetRotation(groundMat)
    ground.Initialize()

    triangle = chrono.ChTriangle(
        chrono.ChVector3d(-1, 1.5, 0),
        chrono.ChVector3d(3, 0.65,0),
        chrono.ChVector3d(0,-0.2,0))
    mesh = chrono.ChTriangleMesh()
    mesh.AddTriangle(triangle)
    ground.AddMesh(mesh)
    ground.SetMeshVisualMaterial(chrono.ChVisualMaterialData(1.0, 0.9, 0.9, 99))
    ground.GetVisualShape(0).SetTexture(veh.GetChronoDataFile('textures/tile4.jpg'))

    
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(hmmwv.GetSystem())
    vis.SetWindowSize(1280,1024)
    vis.SetWindowTitle('HMMWV')
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(100,100,100), chrono.ChVector3d(0,tire_radius,1.6))
    vis.AddTypicalLights()
    vis.AddLightWithShadow(chrono.ChVector3d(20,20,100), chrono.ChVector3d(0,0,0), 30, 3, 1000, 1000)

    
    sky_packed = chrono.GetChronoDataFile('skybox/dresden.jpg')
    if vis.EnableRealtimeSky():
        vis.SetSkyBoxImage(sky_packed);

    
    try:
        os.mkdir(out_dir)
    except:
        print("Output directory already exists");

    
    hmmwv.ExportComponentList(out_dir + "/component_list.json");

    
    driver = veh.ChInteractiveDriverIRR(vis)

    
    steering_time = 1.0  
    throttle_time = 1.0  
    braking_time = 0.3   
    driver.SetSteeringDelta(render_step_size / steering_time)
    driver.SetThrottleDelta(render_step_size / throttle_time)
    driver.SetBrakingDelta(render_step_size / braking_time)

    
    driver.SetRenderStepSize(render_step_size)

    
    driver.Initialize()

    
    hmmwv.GetVehicle().EnableRealtime(True)
    while vis.Run() :
        time = hmmwv.GetSystem().GetChTime()

        
        if (time >= t_end):
            vis.Quit()

        
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        
        driver_inputs = driver.GetInputs()

        
        driver.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, driver.GetInputRates())
        vis.Synchronize(time, driver_inputs)

        
        driver.Advance(step_size)
        hmmwv.Advance(step_size)
        vis.Advance(step_size)

if __name__ == "__main__":
    main()