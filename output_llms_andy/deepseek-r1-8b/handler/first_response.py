import pychrono as pyc
import chrono as c
import pychrono.ros as ros

# Initialize PyChrono environment
c.system.setGravity((0, -9.81, 0))  # Enable gravitational acceleration

# Create a fixed floor
floor = c.Box(mass=1000,  # Mass
             material=c.Material.STEEL,  # Rigid material
             size=(10, 0.01, 10),  # Size (x, y, z)
             position=(0, 0, 0))  # Position

# Create a movable box
box = c.Box(mass=1,  # Mass
           material=c.Material.ALUMINUM,  # Movable material
           size=(0.5, 0.5, 0.5),  # Size
           position=(0.5, 0.5, 0.5))  # Position

# Create a custom ROS handler
class ROSHandler(pyc.ros.RosHandler):
    def __init__(self):
        super().__init__()
        
    def setup(self):
        self.pub = self.create_publisher("object_position", 1)  # Publish integer messages
    
    def update(self, t):
        # Get the position of the movable box
        pos = self.body.get_position()
        # Convert position to integers (meters)
        position = [int(pos[0]*1000), int(pos[1]*1000), int(pos[2]*1000)]
        self.pub.publish(position)

# Configure ROS manager
ros_manager = c.ros.RosManager()
ros_manager.setClockHandler()  # Enable clock handling
ros_manager.setBodyHandler(box)  # Register the movable box as a body
# ros_manager.setTransformHandler()  # Optional: Enable transform handling if needed

# Register the custom ROS handler
ros_manager.registerHandler(ROSHandler(), priority=1)  # Priority 1 means it runs first

# Run simulation loop
t = 0.0
while True:
    # Advance simulation by a fixed timestep
    c.system.advance(0.01)
    t += 0.01
    
    # Update ROS communication
    ros_manager.update()
    
    # Print status
    print(f"Simulation time: {t:.3f}")
    print("Movable box position:", box.get_position())
    
    # Stop simulation after 10 seconds (can be adjusted)
    if t >= 10:
        c.system.stopSimulation()
        break

# Output final message
print("\nSimulation completed!")