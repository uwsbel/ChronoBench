import rospy
from chrono_ros_msgs.srv import ChronoInitialize, ChronoInitializeRequest
from chrono_ros_msgs.msg import ChronoSystemState
from chrono_ros_msgs.srv import ChronoInitializeResponse
from chrono_ros_msgs.msg import ChronoObject
from chrono_ros_msgs.msg import ChronoObjectState
from chrono_ros_msgs.msg import ChronoSystemState
from chrono_ros_msgs.msg import ChronoTimeStep
from chrono_ros_msgs.msg import ChronoInertia
from chrono_ros_msgs.msg import ChronoObjectStateVector
from chrono_ros_msgs.msg import ChronoObject
from chrono_ros_msgs.msg import ChronoTimeStep
from chrono_ros_msgs.srv import ChronoInitialize, ChronoInitializeResponse
from chrono_ros_msgs.msg import ChronoSystemState
from chrono_ros_msgs.srv import ChronoStep, ChronoStepResponse
from chrono_ros_msgs.srv import ChronoStepResponse
from chrono_ros_msgs.srv import ChronoGravity, ChronoGravityResponse
from chrono_ros_msgs.srv import ChronoUpdateMesh, ChronoUpdateMeshResponse
from chrono_ros_msgs.srv import ChronoUpdateContacts, ChronoUpdateContactsResponse
from chrono_ros_msgs.srv import ChronoUpdateForces, ChronoUpdateForcesResponse
from chrono_ros_msgs.srv import ChronoUpdateRigidBodies, ChronoUpdateRigidBodiesResponse
from chrono_ros_msgs.srv import ChronoUpdateConstraints, ChronoUpdateConstraintsResponse
from chrono_ros_msgs.srv import ChronoUpdateMasses, ChronoUpdateMassesResponse
from chrono_ros_msgs.srv import ChronoUpdateContacts, ChronoUpdateContactsResponse
from chrono_ros_msgs.srv import ChronoInitialize, ChronoInitializeResponse
from chrono_ros_msgs.srv import ChronoStep, ChronoStepResponse
from chrono_ros_msgs.srv import ChronoGravity, ChronoGravityResponse
from chrono_ros_msgs.srv import ChronoUpdateMesh, ChronoUpdateMeshResponse
from chrono_ros_msgs.srv import ChronoUpdateContacts, ChronoUpdateContactsResponse
from chrono_ros_msgs.srv import ChronoUpdateForces, ChronoUpdateForcesResponse
from chrono_ros_msgs.srv import ChronoUpdateRigidBodies, ChronoUpdateRigidBodiesResponse
from chrono_ros_msgs.srv import ChronoUpdateConstraints, ChronoUpdateConstraintsResponse
from chrono_ros_msgs.srv import ChronoUpdateMasses, ChronoUpdateMassesResponse
from chrono_ros_msgs.srv import ChronoUpdateContacts, ChronoUpdateContactsResponse
from chrono_ros_msgs.srv import ChronoInitialize, ChronoInitializeResponse
from chrono_ros_msgs.srv import ChronoStep, ChronoStepResponse
from chrono_ros_msgs.srv import ChronoGravity, ChronoGravityResponse
from chrono_ros_msgs.srv import ChronoUpdateMesh, ChronoUpdateMeshResponse
from chrono_ros_msgs.srv import ChronoUpdateContacts, ChronoUpdateContactsResponse
from chrono_ros_msgs.srv import ChronoUpdateForces, ChronoUpdateForcesResponse
from chrono_ros_msgs.srv import ChronoUpdateRigidBodies, ChronoUpdateRigidBodiesResponse
from chrono_ros_msgs.srv import ChronoUpdateConstraints, ChronoUpdateConstraintsResponse
from chrono_ros_msgs.srv import ChronoUpdateMasses, ChronoUpdateMassesResponse
from chrono_ros_msgs.srv import ChronoUpdateContacts, ChronoUpdateContactsResponse
import chrono
import numpy as np

import chrono
from chrono import *
from chrono.rdf import *
from chrono.fea import *
from chrono.rdf import *


rospy.init_node('chrono_ros_simulation')


initialize_srv = rospy.Service('chrono_initialize', ChronoInitialize, handle_chrono_initialize)


step_srv = rospy.Service('chrono_step', ChronoStep, handle_chrono_step)


system_state_pub = rospy.Publisher('chrono_system_state', ChronoSystemState, queue_size=10)


class CustomROSHandler:
    def __init__(self):
        self.system_state_pub = system_state_pub

    def publish_system_state(self, system_state):
        self.system_state_pub.publish(system_state)


def handle_chrono_initialize(req):
    
    system = chrono.ChSystemNSC()

    
    gravity = chrono.ChVectorD(0, 0, -9.81)  
    system.SetGravity(gravity)

    
    floor = chrono.ChBodyEarth()
    floor.SetPos(chrono.ChVectorD(0, 0, 0))
    floor.SetBodyFixed(True)
    system.Add(floor)

    
    box = chrono.ChBodyEarth()
    box.SetPos(chrono.ChVectorD(0, 1, 0))
    box.SetSize(chrono.ChVectorD(1, 1, 1))
    box.SetMass(10)
    system.Add(box)

    
    system.Initialize()

    
    custom_ros_handler = CustomROSHandler()
    rospy.loginfo("Custom ROS handler registered")

    
    return ChronoInitializeResponse(success=True)


def handle_chrono_step(req):
    
    system.DoStep(0.001, 0)

    
    system_state = system.GetState()

    
    system_state_msg = ChronoSystemState()
    system_state_msg.time = system_state.GetTime()
    system_state_msg.positions = [body.GetPos() for body in system.GetBodies()]
    system_state_msg.velocities = [body.GetVel() for body in system.GetBodies()]
    system_state_msg.forces = [body.GetForce() for body in system.GetBodies()]

    
    custom_ros_handler.publish_system_state(system_state_msg)

    
    return ChronoStepResponse(success=True)


def handle_chrono_gravity(req):
    
    response = ChronoGravityResponse(success=True)
    system.SetGravity(req.gravity)
    return response

def handle_chrono_update_mesh(req):
    
    response = ChronoUpdateMeshResponse(success=True)
    system.UpdateMesh(req.mesh)
    return response

def handle_chrono_update_contacts(req):
    
    response = ChronoUpdateContactsResponse(success=True)
    system.UpdateContacts(req.contacts)
    return response

def handle_chrono_update_forces(req):
    
    response = ChronoUpdateForcesResponse(success=True)
    system.UpdateForces(req.forces)
    return response

def handle_chrono_update_rigid_bodies(req):
    
    response = ChronoUpdateRigidBodiesResponse(success=True)
    system.UpdateRigidBodies(req.rigid_bodies)
    return response

def handle_chrono_update_constraints(req):
    
    response = ChronoUpdateConstraintsResponse(success=True)
    system.UpdateConstraints(req.constraints)
    return response

def handle_chrono_update_masses(req):
    
    response = ChronoUpdateMassesResponse(success=True)
    system.UpdateMasses(req.masses)
    return response


rospy.Service('chrono_gravity', ChronoGravity, handle_chrono_gravity)
rospy.Service('chrono_update_mesh', ChronoUpdateMesh, handle_chrono_update_mesh)
rospy.Service('chrono_update_contacts', ChronoUpdateContacts, handle_chrono_update_contacts)
rospy.Service('chrono_update_forces', ChronoUpdateForces, handle_chrono_update_forces)
rospy.Service('chrono_update_rigid_bodies', ChronoUpdateRigidBodies, handle_chrono_update_rigid_bodies)
rospy.Service('chrono_update_constraints', ChronoUpdateConstraints, handle_chrono_update_constraints)
rospy.Service('chrono_update_masses', ChronoUpdateMasses, handle_chrono_update_masses)


def run_simulation():
    rospy.loginfo("Starting simulation loop")
    rate = rospy.Rate(100)  
    while not rospy.is_shutdown():
        
        system.DoStep(0.001, 0)

        
        system_state = system.GetState()

        
        system_state_msg = ChronoSystemState()
        system_state_msg.time = system_state.GetTime()
        system_state_msg.positions = [body.GetPos() for body in system.GetBodies()]
        system_state_msg.velocities = [body.GetVel() for body in system.GetBodies()]
        system_state_msg.forces = [body.GetForce() for body in system.GetBodies()]

        
        custom_ros_handler.publish_system_state(system_state_msg)

        
        rate.sleep()

if __name__ == '__main__':
    try:
        run_simulation()
    except rospy.ROSInterrupt:
        pass