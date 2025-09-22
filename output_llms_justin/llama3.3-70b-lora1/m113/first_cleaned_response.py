import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math as m



def main():
    

    
    

    m113 = veh.M113()
    m113.SetContactMethod(chrono.ChContactMethod_SMC)
    m113.SetChassisCollisionType(veh.CollisionType_NONE)
    m113.SetChassisFixed(False) 
    m113.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(-10, -2, 0.6), chrono.ChQuaterniond(1, 0, 0, 0)))
    m113.SetEngineType(veh.EngineModelType_SHAFTS);
    m113.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS);
    m113.SetDriveType(veh.DrivelineTypeWV_AWD)

    m113.SetTrackShoeWidth(.4)
    m113.SetTrackShoeLength(.2)
    m113.SetSuspensionSpringRestLength(.8)
    m113.SetAdhesionType(veh.TrackAdhesionType_SIMPLE);

    m113.Initialize()

    
    

    terrain = veh.RigidTerrain(m113.GetSystem())
    if (contact_method == chrono.ChContactMethod_NSC):
        patch_mat = chrono.ChContactMaterialNSC()
        patch_mat.SetFriction(0.9)
        patch_mat.SetRestitution(0.9)
    elif (contact_method == chrono.ChContactMethod_SMC):
        patch_mat = chrono.ChContactMaterialSMC()
        patch_mat.SetFriction(0.9)
        patch_mat.SetRestitution(0.9)
    elif (contact_method == chrono.ChContactMethod_SHARP):
        patch_mat = chrono.ChContactMaterialSMC()
        patch_mat.SetFriction(0.9)
        patch_mat.SetRestitution(0.9)
    patch = terrain.AddPatch(patch_mat, 
                             chrono.CSYSNORM, 
                             300, 200)
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    
    

    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('M113')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AttachVehicle(m113.GetVehicle())

    
    

    driver = veh.ChInteractiveDriverIRR(vis)
    driver.SetSteeringDelta(0.02)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.02)
    driver.Initialize()

    
    

    
    print( "VEHICLE MASS:      " , m113.GetVehicle().GetMass())
    print( "TRACK SHOE WIDTH:  " , m113.GetTrackShoeWidth())
    print( "SUSPENSION REST LENGTH: " , m113.GetSuspensionSpringRestLength() )

    while vis.Run() :
        time = m113.GetSystem().GetChTime()

        
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        
        driver_inputs = driver.GetInputs()

        
        driver.Synchronize(time)
        terrain.Synchronize(time)
        m113.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        
        driver.Advance(step_size)
        terrain.Advance(step_size)
        m113.Advance(step_size)
        vis.Advance(step_size)

    return 0







veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


chassis_collision_type = veh.CollisionType_NONE;


contact_method = chrono.ChContactMethod_SMC


step_size = 5e-4;

main()