import pychrono as chrono
import pychrono.vehicle as veh
import numpy as np


def create_custom_terrain():
    terrain = chrono.ChSystemNSC()
    terrain.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

    
    collision_mesh = chrono.ChTriangleMeshConnected()
    collision_mesh.LoadWavefrontOBJ("Highway_col.obj")
    collision_mesh.Transform(chrono.ChMatrix33d(), chrono.ChVector3d(0, 0, 0))
    visual_mesh = chrono.ChTriangleMeshConnected()
    visual_mesh.LoadWavefrontOBJ("Highway_vis.obj")
    visual_mesh.Transform(chrono.ChMatrix33d(), chrono.ChVector3d(0, 0, 0))

    
    terrain_body = chrono.ChBody()
    terrain_body.SetName("Custom Mesh Terrain")
    terrain_body.SetFixed(True)
    terrain_body.EnableCollision(False)
    terrain_body.GetVisualShape(0).SetMesh(visual_mesh)
    terrain_body.GetVisualShape(0).SetColor(chrono.ChColor(0.6, 0.6, 0.6))
    terrain_body.GetVisualShape(0).SetOpacity(0.5)

    
    collision_shape = chrono.ChCollisionShape()
    collision_shape.SetName("Custom Mesh Collision")
    collision_shape.SetMesh(collision_mesh)
    collision_shape.SetColor(chrono.ChColor(0.1, 0.1, 0.1))
    terrain_body.AddCollisionShape(collision_shape)

    terrain.Add(terrain_body)
    return terrain


def create_hmmwv_vehicle():
    veh.SetVehicleDataPath(chrono.GetChronoDataPath() + "vehicle/hmmwv/")

    
    vehicle = veh.HMMWV_Full()
    vehicle.Initialize(chrono.ChVector3d(0, 0, 0.5), chrono.ChQuaterniond(1, 0, 0, 0), veh.ChVehicleSide.LEFT)

    
    vehicle.SetTireType(veh.TireModelType.TMEASY)
    vehicle.SetTireStepSize(0.01)

    
    vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
    vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
    vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetSteeringVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)

    
    driver = veh.ChInteractiveDriverIRR(vehicle)
    driver.SetThrottleDriver(veh.ChThrottleDriver(vehicle.GetVehicle(), 0.0, 1.0))
    driver.SetSteeringDriver(veh.ChSteeringDriver(vehicle.GetVehicle(), -1.0, 1.0))
    driver.SetBrakingDriver(veh.ChBrakingDriver(vehicle.GetVehicle(), 0.0, 1.0))
    driver.SetSteeringController(veh.ChSteeringController(vehicle.GetVehicle(), -1.0, 1.0))
    driver.SetThrottleController(veh.ChThrottleController(vehicle.GetVehicle(), 0.0, 1.0))
    driver.SetBrakingController(veh.ChBrakingController(vehicle.GetVehicle(), 0.0, 1.0))

    return vehicle, driver


def run_simulation():
    
    terrain = create_custom_terrain()
    vehicle, driver = create_hmmwv_vehicle()
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

    
    system.Add(vehicle.GetVehicleBody())

    
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle("PyChrono HMMWV on Custom Mesh Terrain")
    vis.SetWindowSize(1024, 768)
    vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.5), 6.0, 0.5)
    vis.Initialize()
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0, 0, 1.5), chrono.ChVector3d(0, 0, 0))
    vis.AddVehicle(vehicle)

    
    time = 0.0
    step = 0.001
    while vis.Run():
        time += step
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        vis.Synchronize(time)
        vis.Advance(step)

        
        driver.Synchronize(time)
        driver.Advance(step)
        vehicle.Update()


if __name__ == "__main__":
    run_simulation()