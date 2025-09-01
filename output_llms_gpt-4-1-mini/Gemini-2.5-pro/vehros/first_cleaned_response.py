import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.ros as chros 
import math
import os
import sys


try:
    import rospy
except ImportError:
    print("rospy (ROS Python client) not found. Please ensure ROS is installed and sourced.")
    sys.exit(1)




chrono.SetChronoDataPath(os.environ.get('CHRONO_DATA_DIR', '../../../chrono_data/'))
veh.SetDataPath(os.path.join(chrono.GetChronoDataPath(), 'vehicle/'))





step_size = 0.005  

t_end = 1000 


publish_step_size = 1.0 / 50  


initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


chassis_vis_type = veh.VisualizationType_MESH
suspension_vis_type = veh.VisualizationType_PRIMITIVES
steering_vis_type = veh.VisualizationType_PRIMITIVES
wheel_vis_type = veh.VisualizationType_MESH
tire_vis_type = veh.VisualizationType_MESH


contact_method = chrono.ChContactMethod_NSC 


collision_system_type = chrono.ChCollisionSystem.Type_BULLET


engine_model = veh.EngineModelType_SHAFTS 



transmission_model = veh.TransmissionModelType_AUTOMATIC_SHAFTS 



tire_model = veh.TireModelType_TMEASY 




terrain_size_x = 200.0  
terrain_size_y = 200.0  
terrain_friction = 0.8
terrain_restitution = 0.01
terrain_young_modulus = 2e7 
terrain_poisson_ratio = 0.3


ros_node_name = "chrono_hmmwv_simulation"


clock_topic_name = "/clock"
odom_topic_name = "/chrono_vehicle/odom"
driver_inputs_topic_name = "/chrono_vehicle/driver_inputs" 




def main():
    print("Chrono version:", chrono.CHRONO_VERSION)
    print("Chrono Vehicle version:", veh.CHRONO_VEHICLE_VERSION)
    try:
        print("Chrono ROS version:", chros.CHRONO_ROS_VERSION)
    except AttributeError:
        print("Chrono ROS version attribute not found, check your PyChrono installation.")


    
    
    
    sys = chrono.ChSystemNSC() if contact_method == chrono.ChContactMethod_NSC else chrono.ChSystemSMC()
    sys.SetCollisionSystemType(collision_system_type)
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

    
    if contact_method == chrono.ChContactMethod_NSC:
        solver = chrono.ChSolverPSOR()
        solver.SetMaxIterations(150)
        solver.SetTolerance(1e-10)
        solver.SetOmega(0.8)
        solver.SetSharpnessLambda(1.0)
        sys.SetSolver(solver)
        sys.SetMaxPenetrationRecoverySpeed(4.0)

    
    
    
    hmmwv = veh.HMMWV_Reduced(sys) 

    
    hmmwv.SetContactMethod(contact_method)
    hmmwv.SetChassisFixed(False)
    hmmwv.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
    hmmwv.SetEngineType(engine_model)
    hmmwv.SetTransmissionType(transmission_model)
    hmmwv.SetTireType(tire_model)
    hmmwv.SetTirePressure(220e3) 

    
    hmmwv.Initialize()

    
    hmmwv.SetChassisVisualizationType(chassis_vis_type)
    hmmwv.SetSuspensionVisualizationType(suspension_vis_type)
    hmmwv.SetSteeringVisualizationType(steering_vis_type)
    hmmwv.SetWheelVisualizationType(wheel_vis_type)
    hmmwv.SetTireVisualizationType(tire_vis_type)

    vehicle = hmmwv.GetVehicle() 

    
    terrain = veh.RigidTerrain(sys)
    patch_mat = chrono.ChContactMaterialNSC() if contact_method == chrono.ChContactMethod_NSC else chrono.ChContactMaterialSMC()
    patch_mat.SetFriction(terrain_friction)
    patch_mat.SetRestitution(terrain_restitution)
    if contact_method == chrono.ChContactMethod_SMC:
        patch_mat.SetYoungModulus(terrain_young_modulus)
        patch_mat.SetPoissonRatio(terrain_poisson_ratio)

    
    
    patch = terrain.AddPatch(patch_mat,
                             chrono.ChVector3d(0, 0, 0),    
                             chrono.ChVector3d(0, 0, 1),    
                             terrain_size_x, terrain_size_y) 
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200) 
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5)) 
    terrain.Initialize()


    
    
    driver = veh.ChDriver(vehicle)

    
    
    print(f"Initializing ROS Manager for node: {ros_node_name}")
    ros_manager = chros.ChROSManager()
    
    if not rospy.is_shutdown():
        try:
            
            ros_manager.Initialize() 
            print("ROS Manager Initialized.")
        except RuntimeError as e:
            print(f"Error initializing ChROSManager: {e}")
            print("This might happen if a ROS node with the same name is already running or roscore is not available.")
            print("Attempting to connect to an existing node (or ensure roscore is running and retry).")
            
            
            
            if not rospy.core.is_initialized():
                 rospy.init_node(ros_node_name, anonymous=False) 
                 print(f"rospy.init_node('{ros_node_name}') called.")

    else:
        print("rospy is shutdown. Exiting.")
        return

    
    
    
    clock_handler = chros.ChROSClockHandler()
    ros_manager.RegisterHandler(clock_handler)
    print(f"Registered ChROSClockHandler (publishes to {clock_topic_name}).")

    
    
    
    
    driver_inputs_handler = chros.ChROSDriverInputsHandler(driver, driver_inputs_topic_name)
    ros_manager.RegisterHandler(driver_inputs_handler)
    print(f"Registered ChROSDriverInputsHandler (subscribes to {driver_inputs_topic_name}).")
    print("  -> Send geometry_msgs/Twist to this topic:")
    print("     linear.x controls throttle/braking (positive for throttle, negative for braking)")
    print("     angular.z controls steering (positive for left, negative for right)")

    
    
    
    odom_handler = chros.ChROSOdometryHandler(publish_step_size, vehicle, odom_topic_name, "odom", hmmwv.GetChassis().GetName())
    ros_manager.RegisterHandler(odom_handler)
    print(f"Registered ChROSOdometryHandler (publishes to {odom_topic_name}).")
    print(f"  -> Odometry frame_id: odom, child_frame_id: {hmmwv.GetChassis().GetName()}")

    
    
    use_irrlicht = True 
    if use_irrlicht:
        try:
            from pychrono import irrlicht as chronoirr
            vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
            vis.SetWindowTitle('HMMWV ROS Simulation')
            vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5) 
            vis.Initialize()
            vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
            vis.AddSkyBox()
            vis.AddTypicalLights()
            vis.AttachVehicle(vehicle) 
        except ImportError:
            print("Irrlicht bindings not available. Running without visualization.")
            use_irrlicht = False
        except Exception as e:
            print(f"Error initializing Irrlicht: {e}. Running without visualization.")
            use_irrlicht = False
    else:
        print("Irrlicht visualization disabled.")


    
    
    time = 0.0
    step_number = 0
    publish_time = 0.0

    print("\nStarting simulation loop...")
    print(f"Step size: {step_size:.4f}s")
    print(f"Publish rate: {1.0/publish_step_size:.1f} Hz")

    try:
        if use_irrlicht:
            while vis.Run() and time < t_end and not rospy.is_shutdown():
                
                time = sys.GetChTime()

                
                driver_inputs = driver.GetInputs()

                
                terrain.Synchronize(time)
                hmmwv.Synchronize(time, driver_inputs, terrain)
                if use_irrlicht:
                    vis.Synchronize(time, driver_inputs)

                
                terrain.Advance(step_size)
                hmmwv.Advance(step_size)
                if use_irrlicht:
                    vis.Advance(step_size)
                sys.DoStepDynamics(step_size)

                
                if time >= publish_time:
                    if not ros_manager.Update(time, step_size): 
                        print("ROS Manager update failed or ROS shutdown requested. Exiting loop.")
                        break
                    publish_time += publish_step_size

                
                if use_irrlicht:
                    vis.BeginScene()
                    vis.Render()
                    vis.EndScene()

                step_number += 1
        else: 
            while time < t_end and not rospy.is_shutdown():
                time = sys.GetChTime()
                driver_inputs = driver.GetInputs()

                terrain.Synchronize(time)
                hmmwv.Synchronize(time, driver_inputs, terrain)

                terrain.Advance(step_size)
                hmmwv.Advance(step_size)
                sys.DoStepDynamics(step_size)

                if time >= publish_time:
                    if not ros_manager.Update(time, step_size):
                        print("ROS Manager update failed or ROS shutdown requested. Exiting loop.")
                        break
                    publish_time += publish_step_size
                
                if step_number % 200 == 0: 
                    print(f"Sim time: {time:.2f} s")

                step_number += 1

    except Exception as e:
        print(f"An error occurred during the simulation loop: {e}")
    finally:
        print("Simulation loop ended.")
        
        if ros_manager:
            print("Shutting down ROS Manager...")
            ros_manager.Shutdown() 
            print("ROS Manager shut down.")
        
        
        if use_irrlicht and vis and vis.GetDevice() and vis.GetDevice().run():
            vis.GetDevice().closeDevice()

    print("PyChrono HMMWV ROS simulation finished.")

if __name__ == '__main__':
    
    print("Please ensure 'roscore' is running in a separate terminal.")
    print("You can control the vehicle by publishing geometry_msgs/Twist messages to:")
    print(f"  {driver_inputs_topic_name}")
    print("Example using rostopic pub:")
    print(f"  rostopic pub --once {driver_inputs_topic_name} geometry_msgs/Twist \"{{linear: {{x: 0.5, y: 0.0, z: 0.0}}, angular: {{x: 0.0, y: 0.0, z: 0.0}}}}\" (Throttle forward)")
    print(f"  rostopic pub --once {driver_inputs_topic_name} geometry_msgs/Twist \"{{linear: {{x: 0.0, y: 0.0, z: 0.0}}, angular: {{x: 0.0, y: 0.0, z: 0.3}}}}\" (Steer left)")
    input("Press Enter to start the simulation after ensuring roscore is running...")
    main()