from pychrono.core import *
from pychrono.vehicle import *
from pychrono.vehicle_dynamic_problem import *
from pychrono import visualization
from pychrono.irrlicht_utils import *


chrono_init()


man_truck = Vehicle()
man_truck.set_vehicle_type(Vehicle::Truck)
man_truck.set_vehicle_mass(20000.0)
man_truck.set_vehicle_length(5.5)
man_truck.set_vehicle_width(2.0)
man_truck.set_vehicle_height(1.5)
man_truck.set_vehicle_wheelbase(3.0)
man_truck.set_vehicle_tyres_count(4)
man_truck.set_vehicle_tyres_diameter(0.9)
man_truck.set_vehicle_tyres_pressure(2.5)
man_truck.set_vehicle_tyres_coefficient_of_friction(0.8)


tire_model = TireModel()
tire_model.set_type(TireModel::TMEASY)
man_truck.add_tire_model(tire_model)


man_truck.set_default_parameters()


vehicle_dynamics_problem = VehicleDynamicProblem()
vehicle_dynamics_problem.set_vehicle(man_truck)
vehicle_dynamics_problem.set_gravity(chrono_gravity())
vehicle_dynamics_problem.set_time_step(0.01)
vehicle_dynamics_problem.set_collision_settings(CollisionSettings())


terrain = create_rigid_terrain()
terrain.set_name("Rigid Terrain")
terrain.set_material_properties(0.5, 0.5, 0.5)


visualization_manager = visualization.VVisualizationManager()
irrlicht_application = visualization.IrrlichtApplication(width=800, height=600, fullscreen=False)


camera = camera_manager.add_camera(irrlicht_application, "Chase Camera")
camera.set_position(camera_position)
camera.set_lookat(camera_lookat)
camera.set_fov(45.0)
camera.set_field_of_view(110.0)


skybox = visualization.add_skybox(irrlicht_application, "Skybox")
directional_light = visualization.add_directional_light(irrlicht_application, "Directional Light")
directional_light.set_direction(visualization.Vector3D(0.0, -1.0, -0.5))


terrain.set_texture(visualization.Texture("terrain_texture.jpg"))
terrain.set_logo(visualization.Logo("truck_logo.png"))


irrlicht_application.add_scene_graph_node(terrain)


solver = vehicle_dynamics_problem.get_solver()


steering_control = vehicle_dynamics_problem.get_steering_control()
throttle_control = vehicle_dynamics_problem.get_throttle_control()
brake_control = vehicle_dynamics_problem.get_brake_control()


while irrlicht_application.run():
    
    vehicle_dynamics_problem.update()
    
    
    irrlicht_application.draw_frame()
    
    
    if irrlicht_application.get_input_controller().is_key_pressed(irrlicht_application.buttons.KEY_W):
        steering_control.set_steering_angle(0.1)
    elif irrlicht_application.get_input_controller().is_key_pressed(irrlicht_application.buttons.KEY_S):
        steering_control.set_steering_angle(-0.1)
    if irrlicht_application.get_input_controller().is_key_pressed(irrlicht_application.buttons.KEY_A):
        throttle_control.set_throttle(0.1)
    elif irrlicht_application.get_input_controller().is_key_pressed(irrlicht_application.buttons.KEY_D):
        throttle_control.set_throttle(-0.1)
    if irrlicht_application.get_input_controller().is_key_pressed(irrlicht_application.buttons.KEY_SPACE):
        brake_control.set_brake(0.5)
    
    
    solver.update_controls(steering_control, throttle_control, brake_control)
    
    
    vehicle_dynamics_problem.update_solver(solver)
    
    
    camera.set_position(man_truck.get_position() + visualization.Vector3D(0, 0, 5))


irrlicht_application.start_application()