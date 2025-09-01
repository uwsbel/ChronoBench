import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math as m




def main():
    

    

    
    car = veh.ARTcar()
    car.SetContactMethod(chrono.ChContactMethod_NSC)
    car.SetChassisFixed(False)
    car.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(-10, -2, 0.5), chrono.ChQuaterniond(1, 0, 0, 0)))
    car.SetEngineType(veh.EngineModelType_SIMPLE)
    car.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
    car.SetTireType(veh.TireModelType_RIGID)
    car.Initialize()

    

    terrain = veh.RigidTerrain(car.GetSystem())
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    terrain.AddPatch(patch_mat, chrono.CSYSNORM, 200, 200)
    terrain.Initialize()

    
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('dart')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 0.2), 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AttachVehicle(car.GetVehicle())

    
    driver = veh.ChInteractiveDriverIRR(vis)
    driver.SetSteeringDelta(0.06)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.06)
    driver.Initialize()

    

    
    render_steps = m.ceil(render_step_size / step_size)

    
    step_number = 0

    car.GetVehicle().EnableRealtime(True)

    while vis.Run() :
        time = car.GetSystem().GetChTime()

        
        if (step_number % render_steps == 0) :
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            

        
        driver_inputs = driver.GetInputs()

        
        driver.Synchronize(time)
        terrain.Synchronize(time)
        car.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        
        driver.Advance(step_size)
        terrain.Advance(step_size)
        car.Advance(step_size)
        vis.Advance(step_size)

        
        step_number += 1

    return 0







veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


step_size = 2e-3;
render_step_size = 1.0 / 50  

main()