import os
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

def main():
    # Initialize Chrono system
    chrono.SetChronoDataPath(os.path.join(os.environ.get('CHRONO_DATA_DIR', ''), 'data/'))
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))

    # Create ground body with texture
    ground = chrono.ChBodyEasyBox(200, 200, 1, 1000)
    ground.SetPos(chrono.ChVectorD(0, 0, -1))
    ground.SetBodyFixed(True)
    
    # Add ground texture
    ground_texture = chrono.ChTexture()
    ground_texture.SetTextureFilename(chrono.GetChronoDataFile("textures/concrete.jpg"))
    ground.AddAsset(ground_texture)
    system.Add(ground)

    # Create and initialize MAN Kat truck
    truck = veh.MAN_5t_Vehicle()
    truck.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.5)))
    truck.Initialize(veh.ChVisualSystem_Type_IRRLICHT)
    truck.SetChassisCollisionType(veh.CollisionType_NONE)
    truck.SetChassisFixed(False)
    truck.SetTireType(veh.TireModelType_TMEASY)

    # Set visualization modes
    truck.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    truck.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    truck.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    truck.SetWheelVisualizationType(veh.VisualizationType_MESH)
    truck.SetTireVisualizationType(veh.VisualizationType_MESH)

    # Create driver system
    driver = veh.ChIrrGuiDriver(truck.GetVehicle())
    driver.SetSteeringDelta(0.02)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.06)

    # Initialize Irrlicht visualization
    vis = irr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle('MAN 10t Truck Simulation')
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVectorD(0, -10, 2), chrono.ChVectorD(0, 0, 1))
    vis.AddTypicalLights()
    vis.AddLightWithShadow(chrono.ChVectorD(10, 10, 10), chrono.ChVectorD(0, 0, 0), 50, 5, 50, 50, 512)

    # Initialize the truck's powertrain and tires
    truck.InitializePowertrain(veh.ChPowertrainModelType_SIMPLE)
    truck.InitializeTires()

    # Simulation loop
    step_size = 0.005
    while vis.Run():
        time = system.GetChTime()
        
        # Update driver inputs
        driver_inputs = driver.GetInputs()
        driver.Synchronize(time)
        truck.Synchronize(time, driver_inputs, veh.ChTerrain())
        
        # Advance simulation
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step_size)

if __name__ == '__main__':
    main()