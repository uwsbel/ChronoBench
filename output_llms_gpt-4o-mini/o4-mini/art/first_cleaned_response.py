import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.vehicle.rigidterrain as rigidterrain
import pychrono.irrlicht as irr
from pychrono_vehicle.driver import ChIrrGuiDriver

def main():
    
    
    
    
    chrono.SetChronoDataPath("/usr/local/share/chrono/data/")  
    
    
    

    
    system = chrono.ChSystemNSC()

    
    
    
    contact_method = chrono.ContactMethod_NSC

    
    init_loc = chrono.ChVectorD(0, 0.04, 0)       
    init_rot = chrono.ChQuaternionD(1, 0, 0, 0)   

    
    vehicle_json = veh.GetDataFile("ART/vehicle.json")
    vehicle = veh.ArticulatedTrackedVehicle(vehicle_json, contact_method)

    
    vehicle.Initialize(system, chrono.ChCoordsysD(init_loc, init_rot))

    
    vehicle.SetChassisVisualizationType(veh.VisualizationType.MESH)
    vehicle.SetTrackAssemblyVisualizationType(veh.VisualizationType.MESH)

    
    
    
    terrain = rigidterrain.RigidTerrain(system)

    
    patch = terrain.AddPatch(
        chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT),
        200,          
        100,          
        0             
    )

    
    texture_file = veh.GetDataFile("terrain/grass.jpg")
    patch.SetTexture(texture_file, 16, 16)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    patch.SetContactFrictionCoefficient(0.9)

    terrain.Initialize()

    
    
    
    app = irr.ChIrrApp(
        system,
        "ARTcar on Rigid Terrain",
        irr.dimension2du(1280, 720),
        irr.VerticalDir_Z
    )

    app.AddTypicalLogo()
    app.AddTypicalLights()
    app.AddTypicalCamera(
        irr.vector3df(-20, 10, 5),  
        irr.vector3df(0, 0, 0)      
    )

    
    app.AssetBindAll()
    app.AssetUpdateAll()

    
    
    
    driver = ChIrrGuiDriver(app)
    
    driver.SetSteeringDelta(0.02)
    driver.SetThrottleDelta(0.05)
    driver.SetBrakingDelta(0.05)
    driver.Initialize()

    
    
    
    step_size     = 1e-3         
    render_fps    = 50.0
    render_step   = 1.0 / render_fps
    time          = 0.0
    next_render   = 0.0

    while app.GetDevice().run():
        time = system.GetChTime()

        
        if time >= next_render:
            app.BeginScene(True, True, irr.SColor(255, 140, 161, 192))
            app.DrawAll()      
            app.EndScene()
            next_render += render_step

        
        driver.Synchronize(time)

        
        terrain.Synchronize(time)
        vehicle.Synchronize(time, driver.GetInputs(), terrain)
        app.Synchronize("", driver.GetInputs())

        
        driver.Advance(step_size)
        terrain.Advance(step_size)
        vehicle.Advance(step_size)
        app.Advance(step_size)

        

    

if __name__ == "__main__":
    main()