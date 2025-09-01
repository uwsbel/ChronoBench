import chronos
import numpy as np
import matplotlib.pyplot as plt
import time


chrono = chronos.Chrono()



beam_length = 1.0  
beam_width = 0.1  
beam_thickness = 0.01 
support_location = (0, 0, 0)  
support_radius = 0.05 





def motor_a(time, force):
    
    return force * 0.1  

def motor_b(time, force):
    
    return force * 0.05 



constraints = {
    'beam_a': {'type': 'fixed', 'x': support_location[0], 'y': support_location[1], 'z': 0},
    'beam_b': {'type': 'fixed', 'x': support_location[0], 'y': support_location[1], 'z': 0}
}



chrono.set_system('beam_buckling')
chrono.set_parameters({
    'beam_length': beam_length,
    'beam_width': beam_width,
    'beam_thickness': beam_thickness,
    'support_location': support_location,
    'support_radius': support_radius,
    'motor_a_force': 100, 
    'motor_b_force': 50, 
    'motor_a_time_step': 0.01, 
    'motor_b_time_step': 0.01
})


chrono.run()


def calculate_bending_moment(force, beam_length, beam_width, beam_thickness):
    
    
    
    return force * beam_width


def calculate_stress(force, beam_length, beam_width, beam_thickness):
    
    
    
    return force * beam_thickness



def simulate_beam_element(force, beam_length, beam_width, beam_thickness, material_properties):
    
    
    stress = calculate_stress(force, beam_length, beam_width, beam_thickness)
    strain = calculate_strain(force, beam_length, beam_width, beam_thickness)
    return stress, strain


material_properties = {
    'material': 'Steel',
    'young_modulus': 200000,
    'tensile_modulus': 400000
}

def motor_a_function(time, force):
    return force * 0.1  

def motor_b_function(time, force):
    return force * 0.05  

import irrlicht
from irrlicht.graphics import *

class BeamBucklingSimulation(object):
    def __init__(self):
        self.chrono = chrono.Chrono()
        self.time_step = 0.01
        self.beam_length = 1.0
        self.beam_width = 0.1
        self.beam_thickness = 0.01
        self.support_location = (0, 0, 0)
        self.support_radius = 0.05
        self.motor_a_force = 100
        self.motor_b_force = 50
        self.motor_a_time_step = 0.01
        self.motor_b_time_step = 0.01

        self.chrono.set_system('beam_buckling')
        self.chrono.set_parameters({
            'beam_length': self.beam_length,
            'beam_width': self.beam_width,
            'beam_thickness': self.beam_thickness,
            'support_location': self.support_location,
            'support_radius': self.support_radius,
            'motor_a_force': self.motor_a_force,
            'motor_b_force': self.motor_b_force,
            'motor_a_time_step': self.motor_a_time_step,
            'motor_b_time_step': self.motor_b_time_step
        })

    def run(self):
        self.chrono.run()

if __name__ == '__main__':
    simulation = BeamBucklingSimulation()
    simulation.run()