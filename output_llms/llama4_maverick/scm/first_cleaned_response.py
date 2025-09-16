import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import numpy as np






out_dir = "HMMWV_SCM_DEMO"


step_size = 2e-3


render_step_size = 1.0 / 50  






init_loc = chrono.ChVectorD(0, 0, 1.0)
init_orient = chrono.Q_from_AngZ(0)

vehicle = veh.HMMWV_Full(
    init_loc,
    init_orient,
    "HMMWV",
    veh.HMMWV_VehicleType::kNone,
    veh.HMMWV_TireType::kRigid,
    True,
)


tire_vis_type = veh.VisualizationType_MESH


powertrain_vis_type = veh.VisualizationType_NONE


vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.SetTireVisualizationType(tire_vis_type)


vehicle.Initialize()






terrain_params = veh.SCMDeformableTerrain::Params()
terrain_params.setSoilParameters(
    2e6,   
    0,     
    1.0,   
    0,     
    30,    
    1000,  
    2e8,   
    3e4    
)

terrain = veh.SCMDeformableTerrain(vehicle.GetSystem())
terrain.SetSoilParameters(
    terrain_params.Kphi,
    terrain_params.Kc,
    terrain_params.n,
    terrain_params.C,
    terrain_params.phi,
    terrain_params.K,
    terrain_params.elastic_K,
    terrain_params.damping_R,
)


terrain.EnableMovingPatch(vehicle.GetChassis(), init_loc, 5, 5, 0.2)


terrain.EnableVisualization(True)
terrain.EnableVisualizationSinkage(True)






vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV SCM Demo")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVectorD(0.0, 0.0, 1.75), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddLightDirectional()
vis.AttachVehicle(vehicle.GetVehicle())






driver = veh.ChIrrGuiDriver(vis)


driver.SetSteering(0)
driver.SetThrottle(0)
driver.SetBraking(0)






realtime_timer = chrono.ChRealtimeStepTimer()
while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    
    driver_inputs = driver.GetInputs()
    vehicle.SetDriverInputs(driver_inputs)

    
    vehicle.Synchronize(time, driver_inputs, terrain)
    terrain.Synchronize(time)

    
    vis.Synchronize("", driver_inputs)

    
    vehicle.Advance(step_size)
    terrain.Advance(step_size)

    
    realtime_timer.Spin(step_size)


if __name__ == "__main__":
    main()

import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import numpy as np

def main():
    
    
    

    
    out_dir = "HMMWV_SCM_DEMO"

    
    step_size = 2e-3

    
    render_step_size = 1.0 / 50  

    
    
    

    
    init_loc = chrono.ChVectorD(0, 0, 1.0)
    init_orient = chrono.Q_from_AngZ(0)

    vehicle = veh.HMMWV_Full(
        init_loc,
        init_orient,
        "HMMWV",
        veh.HMMWV_VehicleType::kNone,
        veh.HMMWV_TireType::kRigid,
        True,
    )

    
    tire_vis_type = veh.VisualizationType_MESH

    
    powertrain_vis_type = veh.VisualizationType_NONE

    
    vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetTireVisualizationType(tire_vis_type)

    
    vehicle.Initialize()

    
    
    

    
    terrain_params = veh.SCMDeformableTerrain::Params()
    terrain_params.setSoilParameters(
        2e6,   
        0,     
        1.0,   
        0,     
        30,    
        1000,  
        2e8,   
        3e4    
    )

    terrain = veh.SCMDeformableTerrain(vehicle.GetSystem())
    terrain.SetSoilParameters(
        terrain_params.Kphi,
        terrain_params.Kc,
        terrain_params.n,
        terrain_params.C,
        terrain_params.phi,
        terrain_params.K,
        terrain_params.elastic_K,
        terrain_params.damping_R,
    )

    
    terrain.EnableMovingPatch(vehicle.GetChassis(), init_loc, 5, 5, 0.2)

    
    terrain.EnableVisualization(True)
    terrain.EnableVisualizationSinkage(True)

    
    
    

    
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle("HMMWV SCM Demo")
    vis.SetWindowSize(1280, 720)
    vis.SetChaseCamera(chrono.ChVectorD(0.0, 0.0, 1.75), 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddLightDirectional()
    vis.AttachVehicle(vehicle.GetVehicle())

    
    
    

    
    driver = veh.ChIrrGuiDriver(vis)

    
    driver.SetSteering(0)
    driver.SetThrottle(0)
    driver.SetBraking(0)

    
    
    

    
    realtime_timer = chrono.ChRealtimeStepTimer()
    while vis.Run():
        time = vehicle.GetSystem().GetChTime()

        
        driver_inputs = driver.GetInputs()
        vehicle.SetDriverInputs(driver_inputs)

        
        vehicle.Synchronize(time, driver_inputs, terrain)
        terrain.Synchronize(time)

        
        vis.Synchronize("", driver_inputs)

        
        vehicle.Advance(step_size)
        terrain.Advance(step_size)

        
        realtime_timer.Spin(step_size)

if __name__ == "__main__":
    main()