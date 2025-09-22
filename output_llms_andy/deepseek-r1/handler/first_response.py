import pychrono as chrono
import pychrono.ros as chros
import numpy as np

def main():
    # ===== 1. Initialize Chrono system =====
    system = chrono.ChSystemNSC()
    system.SetGravity(chrono.ChVectorD(0, -9.81, 0))  # Standard gravity
    
    # ===== 2. Create materials =====
    material = chrono.ChMaterialSurfaceNSC()
    material.SetFriction(0.5)
    material.SetRestitution(0.2)

    # ===== 3. Create floor =====
    floor = chrono.ChBody()
    floor.SetPos(chrono.ChVectorD(0, -0.5, 0))
    floor.SetBodyFixed(True)  # Make floor immovable
    floor_shape = chrono.ChBoxShape()
    floor_shape.GetBoxGeometry().Size = chrono.ChVectorD(5, 0.5, 5)  # 10x1x10 meter platform
    floor.AddVisualShape(floor_shape)
    floor.GetCollisionModel().AddBox(material, 5, 0.5, 5)
    floor.SetCollide(True)
    system.Add(floor)

    # ===== 4. Create movable box =====
    box = chrono.ChBody()
    box.SetPos(chrono.ChVectorD(0, 2, 0))
    box.SetMass(1.0)
    box_shape = chrono.ChBoxShape()
    box_shape.GetBoxGeometry().Size = chrono.ChVectorD(0.2, 0.2, 0.2)  # 40cm cube
    box.AddVisualShape(box_shape)
    box.GetCollisionModel().AddBox(material, 0.2, 0.2, 0.2)
    box.SetCollide(True)
    system.Add(box)

    # ===== 5. ROS Integration Setup =====
    ros_manager = chros.ChROSPyManager()
    ros_manager.RegisterHandler(chros.ChROSClockHandler())        # Time synchronization
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(1, box))   # Track box position
    ros_manager.RegisterHandler(chros.ChROSTFHandler(1, False))   # Coordinate frames

    # Custom message handler (publishes simulation step count)
    class StepCounterHandler(chros.ChROSHandler):
        def __init__(self):
            super().__init__()
            self.topic_name = "/simulation/step_count"
            self.publisher = None
            self.step_count = 0
            
        def Initialize(self):
            self.publisher = ros_manager.GetNode().create_publisher(chros.Int32, self.topic_name, 1)
            return True
            
        def Tick(self, time):
            msg = chros.Int32()
            msg.data = self.step_count
            self.publisher.publish(msg)
            self.step_count += 1

    ros_manager.RegisterHandler(StepCounterHandler())

    # Initialize ROS
    if not ros_manager.Initialize():
        print("ROS initialization failed!")
        return

    # ===== 6. Simulation Loop =====
    time_step = 0.01
    realtime_step = True  # Sync with real-time

    while ros_manager.NodeOkay():
        # Advance simulation
        system.DoStepDynamics(time_step)
        
        # Update ROS components
        ros_manager.Update(system, system.GetChTime())

        # Maintain real-time pacing
        if realtime_step:
            ros_manager.SpinSome()
            ros_manager.Rate().sleep()

if __name__ == "__main__":
    main()