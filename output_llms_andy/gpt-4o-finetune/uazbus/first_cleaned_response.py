import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

def main():
    
    sys = chrono.ChSystemNSC()
    
    
    ground_mat = chrono.ChContactMaterialNSC()
    ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
    ground.SetPos(chrono.ChVector3d(0, 0, -1.1))
    ground.SetFixed(True)
    ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
    sys.Add(ground)
    
    
    init_pos = chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.5), chrono.QuatFromAngleZ(0))
    vehicle = veh.UAZBUS()
    vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
    vehicle.SetChassisFixed(False)
    vehicle.SetInitPosition(init_pos)
    vehicle.SetTireType(veh.TireModelType_TMEASY)
    vehicle.SetTireStepSize(1e-3)
    vehicle.Initialize()
    vehicle.SetChassisVisualization(veh.VisualizationType_MESH)
    vehicle.SetSuspensionVisualization(veh.VisualizationType_MESH)
    vehicle.SetSteeringVisualization(veh.VisualizationType_MESH)
    vehicle.SetWheelVisualization(veh.VisualizationType_MESH)
    vehicle.SetTireVisualization(veh.VisualizationType_MESH)
    vehicle.GetSystem().Add(chrono.ChForce(chrono.ChDOF6_3))
    
    
    terrain = veh.RigidTerrain(vehicle.GetSystem())
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch = terrain.AddPatch(patch_mat, 
        chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),  // Center and orientation
        50,    // Size X
        50)    // Size Y
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    patch.SetVisibility(True)
    terrain.Initialize()
    
    
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('UAZBUS vehicle')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 0.75), 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AttachVehicle(vehicle.GetVehicle())
    
    
    driver = veh.ChInteractiveDriverIRR(vis)
    driver.SetSteeringDelta(0.02)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.06)
    driver.Initialize()
    
    
    print( "UAZBUS vehicle model loaded:")
    print( "  Vehicle mass:       " ,  vehicle.GetVehicle().GetMass())
    print( "  Chassis COM:        " ,  vehicle.GetVehicle().GetCom().GetX(),  vehicle.GetVehicle().GetCom().GetY(),  vehicle.GetVehicle().GetCom().GetZ())
    print( "  Tire radius:        " ,  vehicle.GetVehicle().GetTire(0).GetRadius())
    print( "  Wheelbase:          " ,  vehicle.GetVehicle().GetWheelbase())
    print( "  Max steering angle: " ,  vehicle.GetVehicle().GetMaxSteeringAngle())
    
    
    time_step = 1e-3
    tire_step_size = 1e-3
    
    
    step_number = 0
    render_step_size = 1.0 / 50  
    render_steps = math.ceil(render_step_size / time_step)
    
    
    vehicle.GetVehicle().EnableRealtime(True)
    
    
    while vis.Run() :
        time = vehicle.GetSystem().GetChTime()
        
        if step_number % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            
        
        driver_inputs = driver.GetInputs()
        
        
        driver.Synchronize(time)
        terrain.Synchronize(time)
        vehicle.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)
        
        driver.Advance(time_step)
        terrain.Advance(time_step)
        vehicle.Advance(time_step)
        
        step_number += 1
        
    return 0

if __name__ == "__main__":
    main()