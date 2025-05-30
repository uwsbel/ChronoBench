import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math
import os

def main():
    
    chrono.SetChronoDataPath("C:/Program Files/Chrono/data/")
    
    
    print("Copyright (c) 2017 projectchrono.org")
    print("Chrono version: ", chrono.CHRONO_VERSION)

    
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))

    
    contact_method = chrono.ChContactMethod_NSC
    contact_vis = False

    
    terrain = veh.RigidTerrain(system)
    
    
    patch_mat = chrono.ChMaterialSurfaceNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    
    
    terrain_length = 200.0
    terrain_width = 200.0
    terrain_height = 0.1
    
    patch = terrain.AddPatch(patch_mat, 
                           chrono.ChCoordsysD(chrono.ChVectorD(0, 0, -terrain_height/2), 
                                            chrono.QUNIT),
                           terrain_length, 
                           terrain_width)
    
    
    patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    
    terrain.Initialize()

    
    initLoc = chrono.ChVectorD(0, 0, 0.5)
    initRot = chrono.ChQuaternionD(1, 0, 0, 0)

    
    gator = veh.Gator()
    gator.SetContactMethod(contact_method)
    gator.SetChassisCollisionType(veh.CollisionType_NONE)
    gator.SetChassisFixed(False)
    gator.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot))
    gator.SetTireType(veh.TireModelType_TMEASY)
    gator.SetTireStepSize(1e-3)
    gator.Initialize()

    
    gator.SetChassisVisualizationType(veh.VisualizationType_MESH)
    gator.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    gator.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    gator.SetWheelVisualizationType(veh.VisualizationType_MESH)
    gator.SetTireVisualizationType(veh.VisualizationType_MESH)

    
    driver = veh.ChInteractiveDriverIRR()

    
    steering_time = 1.0  
    throttle_time = 1.0  
    braking_time = 0.3   
    
    driver.SetSteeringDelta(1.0 / steering_time)
    driver.SetThrottleDelta(1.0 / throttle_time)
    driver.SetBrakingDelta(1.0 / braking_time)

    
    driver.Initialize()

    
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle("Gator Vehicle Simulation")
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVectorD(0.0, 0.0, 0.75), 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AttachVehicle(gator.GetVehicle())

    
    step_size = 1e-3
    tire_step_size = step_size
    render_step_size = 1.0 / 50  

    
    render_steps = int(math.ceil(render_step_size / step_size))

    
    time = 0
    render_frame = 0

    print("Vehicle mass: ", gator.GetVehicle().GetMass())

    
    while vis.Run():
        time = gator.GetSystem().GetChTime()

        
        if render_frame % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        
        driver_inputs = driver.GetInputs()

        
        driver.Synchronize(time)
        terrain.Synchronize(time)
        gator.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        
        driver.Advance(step_size)
        terrain.Advance(step_size)
        gator.Advance(step_size)
        vis.Advance(step_size)

        
        render_frame += 1

if __name__ == "__main__":
    main()