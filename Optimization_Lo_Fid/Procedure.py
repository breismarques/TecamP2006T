# Procedure2.py

# Python Imports
import numpy as np
import pylab as plt
from subprocess import call
import time
import math
import os

# SUAVE Imports
import SUAVE
from SUAVE.Core import Data, Units
from SUAVE.Components.Energy.Networks.Battery_Propeller import Battery_Propeller
from SUAVE.Methods.Propulsion import propeller_design
from SUAVE.Input_Output.Results import  print_parasite_drag,  \
     print_compress_drag, \
     print_engine_data,   \
     print_mission_breakdown, \
     print_weight_breakdown
from SUAVE.Methods.Power.Battery.Sizing import initialize_from_energy_and_power, initialize_from_mass
from SUAVE.Input_Output.OpenVSP import vsp_write
from SUAVE.Methods.Aerodynamics.Fidelity_Zero.Lift.compute_max_lift_coeff import compute_max_lift_coeff
from SUAVE.Optimization.write_optimization_outputs import write_optimization_outputs
from SUAVE.Methods.Propulsion.electric_motor_sizing import size_from_kv
#from SUAVE.Input_Output.OpenVSP import vspaero

from SUAVE.Analyses.Process import Process

# Files Imports

import Analyses
import Missions
import Vehicles

# ----------------------------------------------------------------------        
#   Setup
# ----------------------------------------------------------------------   

def setup():
    
    # ------------------------------------------------------------------
    #   Analysis Procedure
    # ------------------------------------------------------------------ 
    
    # size the base config
    procedure = Process()
    procedure.simple_sizing = simple_sizing
    
    # Size the battery and charge it before the mission
    procedure.weights_battery = weights_battery
    # finalizes the data dependencies
    procedure.finalize = finalize
    
    # performance studies
    procedure.missions                   = Process()
    procedure.missions.design_mission    = design_mission

    # post process the results
    procedure.post_process = post_process
        
    return procedure

# ----------------------------------------------------------------------        
#   Target Range Function
# ----------------------------------------------------------------------    

def find_target_range(nexus,mission):
    
    segments = mission.segments
    cruise_altitude = mission.segments['climb_2'].altitude_end
    climb_1  = segments['climb_1']
    climb_2  = segments['climb_2']
  
    descent_1 = segments['descent_1']
    descent_2 = segments['descent_2']

    x_climb_1   = climb_1.altitude_end/np.tan(np.arcsin(climb_1.climb_rate/climb_1.air_speed))
    x_climb_2   = (climb_2.altitude_end-climb_1.altitude_end)/np.tan(np.arcsin(climb_2.climb_rate/climb_2.air_speed))

    x_descent_1 = (climb_2.altitude_end-descent_1.altitude_end)/np.tan(np.arcsin(descent_1.descent_rate/descent_1.air_speed))
    x_descent_2 = (descent_1.altitude_end-descent_2.altitude_end)/np.tan(np.arcsin(descent_2.descent_rate/descent_2.air_speed))
    
    cruise_range = mission.design_range-(x_climb_1+x_climb_2+x_descent_1+x_descent_2)
    
    #print segments['cruise'].distance
    
    nexus.missions.mission.total_range=segments['cruise'].distance+x_climb_1+x_climb_2+x_descent_1+x_descent_2
    
  
    #segments['cruise'].distance = cruise_range
    
    return nexus

# ----------------------------------------------------------------------        
#   Design Mission
# ----------------------------------------------------------------------   
 
def design_mission(nexus):
    
    mission = nexus.missions.mission
    mission.design_range = 1500.*Units.nautical_miles
    find_target_range(nexus,mission)
    results = nexus.results
    results.base = mission.evaluate()
    
    return nexus

# ----------------------------------------------------------------------        
#   Sizing
# ----------------------------------------------------------------------    

def simple_sizing(nexus):
    configs=nexus.vehicle_configurations
    base=configs.base
    
    
    #find conditions
    air_speed   = nexus.missions.mission.segments['cruise'].air_speed 
    altitude    = nexus.missions.mission.segments['climb_2'].altitude_end
    atmosphere  = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    
    freestream  = atmosphere.compute_values(altitude)
    freestream0 = atmosphere.compute_values(6000.*Units.ft)  #cabin altitude
    
    diff_pressure         = np.max(freestream0.pressure-freestream.pressure,0)
    fuselage              = base.fuselages['fuselage']
    fuselage.differential_pressure = diff_pressure 
    
    #now size engine
    mach_number        = air_speed/freestream.speed_of_sound
    
    #now add to freestream data object
    freestream.velocity    = air_speed
    freestream.mach_number = mach_number
    freestream.gravity     = 9.81
    
    conditions             = SUAVE.Analyses.Mission.Segments.Conditions.Aerodynamics()   #assign conditions in form for propulsor sizing
    conditions.freestream  = freestream
    
    #for config in configs:
    #    config.wings.horizontal_stabilizer.areas.reference = (26.0/92.0)*config.wings.main_wing.areas.reference
            
    #    for wing in config.wings:
            
    #        wing = SUAVE.Methods.Geometry.Two_Dimensional.Planform.wing_planform(wing)
    #        wing.areas.exposed  = 0.8 * wing.areas.wetted
    #        wing.areas.affected = 0.6 * wing.areas.reference
            
    #    fuselage              = config.fuselages['fuselage']
    #    fuselage.differential_pressure = diff_pressure 
        
        #turbofan_sizing(config.propulsors['turbofan'], mach_number = mach_number, altitude = altitude)
        #compute_turbofan_geometry(config.propulsors['turbofan'], conditions)
        
    # Change the dynamic pressure based on the, add a factor of safety   
    #base.envelope.maximum_dynamic_pressure = nexus.missions.mission.segments.cruise.dynamic_pressure*1.2
    
    #print nexus.missions.mission.segments.cruise.dynamic_pressure*1.2
    
    # Scale the horizontal and vertical tails based on the main wing area
    #For more or less "normal" looking designs the stabilizer and elevators together should be between 15 to 20% of the wing area. 
    #The fin and rudder together should be around 12 to 15%
    base.wings.horizontal_stabilizer.areas.reference = 0.17 * base.reference_area
    base.wings.vertical_stabilizer.areas.reference   = 0.13 * base.reference_area


    # ------------------------------------------------------------------
    #   Landing Configuration
    # ------------------------------------------------------------------
    #landing = nexus.vehicle_configurations.landing
    #landing_conditions = Data()
    #landing_conditions.freestream = Data()

    # landing weight
    #landing.mass_properties.landing = 1.0 * config.mass_properties.takeoff
    
    # Landing CL_max
    #altitude   = nexus.missions.mission.segments[-1].altitude_end
    #atmosphere = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    #freestream_landing = atmosphere.compute_values(0.)
    #landing_conditions.freestream.velocity           = nexus.missions.mission.segments['descent_2'].air_speed
    #landing_conditions.freestream.density            = freestream_landing.density
    #landing_conditions.freestream.dynamic_viscosity  = freestream_landing.dynamic_viscosity
    #CL_max_landing,CDi = compute_max_lift_coeff(landing,landing_conditions)
    #landing.maximum_lift_coefficient = CL_max_landing
    
    #Takeoff CL_max
    #takeoff = nexus.vehicle_configurations.takeoff
    #takeoff_conditions = Data()
    #takeoff_conditions.freestream = Data()    
    #altitude = nexus.missions.mission.airport.altitude
    #freestream_takeoff = atmosphere.compute_values(altitude)
   
    #takeoff_conditions.freestream.velocity           = nexus.missions.mission.segments.climb_1.air_speed
    #takeoff_conditions.freestream.density            = freestream_takeoff.density
    #takeoff_conditions.freestream.dynamic_viscosity  = freestream_takeoff.dynamic_viscosity 
    #max_CL_takeoff, CDi = compute_max_lift_coeff(takeoff,takeoff_conditions) 
    #takeoff.maximum_lift_coefficient = max_CL_takeoff
    
    #Base config CL_max
    #base = nexus.vehicle_configurations.base
    #base_conditions = Data()
    #base_conditions.freestream = takeoff_conditions.freestream   
    #max_CL_base, CDi = compute_max_lift_coeff(base,base_conditions) 
    #base.maximum_lift_coefficient = max_CL_base
    
    # Pull out the vehicle
    vec = nexus.vehicle_configurations.base
    
    # Resize the motor
    motor_forward = vec.propulsors.propulsor.motor_forward
    kv    = motor_forward.speed_constant
    motor_forward = size_from_kv(motor_forward, kv)

    motor_lift = vec.propulsors.propulsor.motor_lift
    kv    = motor_lift.speed_constant
    motor_lift = size_from_kv(motor_lift, kv)    
    
    # diff the new data
    vec.store_diff()
    
    
    return nexus

# ----------------------------------------------------------------------        
#   Weights
# ----------------------------------------------------------------------    

def weights_battery(nexus):
    
    # Evaluate weights for all of the configurations
    config = nexus.analyses.base
    config.weights.evaluate() 
    
    vec     = nexus.vehicle_configurations.base
    payload = vec.propulsors.propulsor.payload.mass_properties.mass  
    #msolar  = vec.propulsors.propulsor.solar_panel.mass_properties.mass
    MTOW    = vec.mass_properties.max_takeoff
    empty   = vec.weight_breakdown.empty
    mmotor  = vec.propulsors.propulsor.motor_forward.mass_properties.mass+vec.propulsors.propulsor.motor_lift.mass_properties.mass
    
    #Calculate battery mass
    bat     = vec.propulsors.propulsor.battery
    initialize_from_mass(bat,bat.mass_properties.mass)
    
    # diff the new data
    vec.store_diff()
        
    # Set Battery Charge
    maxcharge = nexus.vehicle_configurations.base.propulsors.propulsor.battery.max_energy
    charge    = maxcharge
    
    nexus.missions.mission.segments['climb_1'].battery_energy = charge 
    
       
    return nexus

# ----------------------------------------------------------------------
#   Finalizing Function
# ----------------------------------------------------------------------    

def finalize(nexus):
    
    nexus.analyses.finalize()   
    
    return nexus

# ----------------------------------------------------------------------
#   Post Process results to give back to the optimizer
# ----------------------------------------------------------------------   

def post_process(nexus):
    
    # Unpack data
    vehicle                           = nexus.vehicle_configurations.base
    results                           = nexus.results
    print "SPAN"
    print nexus.vehicle_configurations.cruise.wings.main_wing.spans.projected
    print "Root Chord"
    print vehicle.wings.main_wing.chords.root
    print "Voltage"
    print vehicle.propulsors.propulsor.voltage
    print vehicle.propulsors.propulsor.motor_forward.speed_constant
    print vehicle.propulsors.propulsor.motor_lift.speed_constant
    print vehicle.propulsors.propulsor.battery.specific_energy
    print vehicle.propulsors.propulsor.battery.specific_power
    print vehicle.propulsors.propulsor.battery.max_voltage
    print vehicle.propulsors.propulsor.battery.resistance
    print vehicle.propulsors.propulsor.payload.power_draw
    print vehicle.propulsors.propulsor.avionics.power_draw
    
    #aux = nexus.results
    
    #with open('outfile.txt','wb') as f:
    #    for line in aux:
    #        np.savetxt(f, line, fmt="%s")
        
    #res = nexus.results.segment.cruise.conditions
    summary                           = nexus.summary
    missions                          = nexus.missions  
    nexus.total_number_of_iterations +=1
    
    #throttle in design mission
    max_throttle = 0
    for segment in results.base.segments.values():
        max_segment_throttle = np.max(segment.conditions.propulsion.throttle[:,0])
        if max_segment_throttle > max_throttle:
            max_throttle = max_segment_throttle
            
    summary.max_throttle = max_throttle
    
    summary.total_range = missions.mission.total_range
    
    # CL max constraint, it is the same throughout the mission
    #CL = res.aerodynamics.lift_coefficient[0]
    
    # Pack up
    #summary.CL                = 1.2 - CL
    
    initial_time=vehicle.fuselages.fuselage.time
    
    time1 = time.time()
            
    print 'The total time to run the all the SIMULATION: '+ str(time1-initial_time) + '  Seconds'
    
    #when you run want to output results to a file
    filename = 'results.txt'
    
    conditions=nexus.results.base.conditions
    
    ## See if constraints are ok
    
    energy=[]
    
    energy.extend(conditions.climb_1.propulsion.battery_energy[:,0])
    energy.extend(conditions.climb_2.propulsion.battery_energy[:,0])
    energy.extend(conditions.cruise.propulsion.battery_energy[:,0])
    energy.extend(conditions.descent_1.propulsion.battery_energy[:,0])
    energy.extend(conditions.descent_2.propulsion.battery_energy[:,0])
    
    for x in energy:
        if x < 0.0:
            value=-1.0
            #missions.mission.total_range=540.9079921321364
            break
        else:
            value=1.0
    
    summary.battery_energy_all_segments=value
    
    
    throttle=[]
    
    throttle.extend(conditions.climb_1.propulsion.throttle[:,0])
    throttle.extend(conditions.climb_2.propulsion.throttle[:,0])
    throttle.extend(conditions.cruise.propulsion.throttle[:,0])
    throttle.extend(conditions.descent_1.propulsion.throttle[:,0])
    throttle.extend(conditions.descent_2.propulsion.throttle[:,0])
    
    for x in throttle:
        if x < -0.1:
            value=-1.0
            #missions.mission.total_range=540.9079921321364
            break
        elif x > 1.1:          
            value=2.0
            #missions.mission.total_range=540.9079921321364
            break
        else:
            value=1.0
    
    summary.throttle_all_segments=value
    
    lift_throttle=[]
    
    lift_throttle.extend(conditions.climb_1.propulsion.lift_throttle[:,0])
    lift_throttle.extend(conditions.climb_2.propulsion.lift_throttle[:,0])
    lift_throttle.extend(conditions.cruise.propulsion.lift_throttle[:,0])
    lift_throttle.extend(conditions.descent_1.propulsion.lift_throttle[:,0])
    lift_throttle.extend(conditions.descent_2.propulsion.lift_throttle[:,0])
    
    for x in lift_throttle:
        if x < -0.1:
            value=-1.0
            #missions.mission.total_range=540.9079921321364
            break
        elif x > 1.1:          
            value=2.0
            #missions.mission.total_range=540.9079921321364
            break
        else:
            value=1.0
    
    summary.lift_throttle_all_segments=value
    
    rpm_lift=[]
    
    rpm_lift.extend(conditions.climb_1.propulsion.rpm_lift[:,0])
    rpm_lift.extend(conditions.climb_2.propulsion.rpm_lift[:,0])
    rpm_lift.extend(conditions.cruise.propulsion.rpm_lift[:,0])
    rpm_lift.extend(conditions.descent_1.propulsion.rpm_lift[:,0])
    rpm_lift.extend(conditions.descent_2.propulsion.rpm_lift[:,0])
    
    #print rpm_lift
    
    for x in rpm_lift:
        if x < -1:
            value=-1.0
            #missions.mission.total_range=540.9079921321364
            break
        else:
            value=1.0
    
    summary.rpm_lift_all_segments=value
    
    rpm_forward=[]
    
    rpm_forward.extend(conditions.climb_1.propulsion.rpm_forward[:,0])
    rpm_forward.extend(conditions.climb_2.propulsion.rpm_forward[:,0])
    rpm_forward.extend(conditions.cruise.propulsion.rpm_forward[:,0])
    rpm_forward.extend(conditions.descent_1.propulsion.rpm_forward[:,0])
    rpm_forward.extend(conditions.descent_2.propulsion.rpm_forward[:,0])
    
    #print rpm_forward
    
    for x in rpm_forward:
        if x < -0.1:
            value=-1.0
            #missions.mission.total_range=540.9079921321364
            break
        else:
            value=1.0
    
    summary.rpm_forward_all_segments=value
    
    lift_coefficient=[]
    
    lift_coefficient.extend(conditions.climb_1.aerodynamics.lift_coefficient[:,0])
    lift_coefficient.extend(conditions.climb_2.aerodynamics.lift_coefficient[:,0])
    lift_coefficient.extend(conditions.cruise.aerodynamics.lift_coefficient[:,0])
    lift_coefficient.extend(conditions.descent_1.aerodynamics.lift_coefficient[:,0])
    lift_coefficient.extend(conditions.descent_2.aerodynamics.lift_coefficient[:,0])
    
    for x in lift_coefficient:
        if x < -0.1:
            value=-1.0
            #missions.mission.total_range=540.9079921321364
            break
        else:
            value=1.0
    
    summary.lift_coefficient_all_segments=value
    
    charging_power=[]
    
    charging_power.extend(conditions.climb_1.propulsion.battery_draw[:,0])
    charging_power.extend(conditions.climb_2.propulsion.battery_draw[:,0])
    charging_power.extend(conditions.cruise.propulsion.battery_draw[:,0])
    charging_power.extend(conditions.descent_1.propulsion.battery_draw[:,0])
    charging_power.extend(conditions.descent_2.propulsion.battery_draw[:,0])
    
    for x in charging_power:
        if x < -0.1:
            value=-1.0
            #missions.mission.total_range=540.9079921321364
            break
        else:
            value=1.0
    
    summary.battery_charging_power_all_segments=value
    
    
    AoA=[]
    
    AoA.extend(conditions.climb_1.aerodynamics.angle_of_attack[:,0])
    AoA.extend(conditions.climb_2.aerodynamics.angle_of_attack[:,0])
    AoA.extend(conditions.cruise.aerodynamics.angle_of_attack[:,0])
    AoA.extend(conditions.descent_1.aerodynamics.angle_of_attack[:,0])
    AoA.extend(conditions.descent_2.aerodynamics.angle_of_attack[:,0])
    
    AoAdeg=np.zeros([len(AoA),1])
    i=0
    while i<len(AoA):
        AoAdeg[i]=math.degrees(AoA[i])
        i=i+1
        
    #print AoAdeg
    
    mach=[]
    
    mach.extend(conditions.climb_1.freestream.mach_number[:,0])
    mach.extend(conditions.climb_2.freestream.mach_number[:,0])
    mach.extend(conditions.cruise.freestream.mach_number[:,0])
    mach.extend(conditions.descent_1.freestream.mach_number[:,0])
    mach.extend(conditions.descent_2.freestream.mach_number[:,0])
    
    #print mach
    
    for x in mach:
        if x > 1.01:
            value=2.0
            #missions.mission.total_range=540.9079921321364
            break
        else:
            value=1.0
    
    summary.mach_number_all_segments=value
    
    cp_lift=[]
    
    cp_lift.extend(conditions.climb_1.propulsion.propeller_power_coefficient_lift[:,0])
    cp_lift.extend(conditions.climb_2.propulsion.propeller_power_coefficient_lift[:,0])
    cp_lift.extend(conditions.cruise.propulsion.propeller_power_coefficient_lift[:,0])
    cp_lift.extend(conditions.descent_1.propulsion.propeller_power_coefficient_lift[:,0])
    cp_lift.extend(conditions.descent_2.propulsion.propeller_power_coefficient_lift[:,0])
    
    #print cp_lift
    
    cp_forward=[]
    
    cp_forward.extend(conditions.climb_1.propulsion.propeller_power_coefficient[:,0])
    cp_forward.extend(conditions.climb_2.propulsion.propeller_power_coefficient[:,0])
    cp_forward.extend(conditions.cruise.propulsion.propeller_power_coefficient[:,0])
    cp_forward.extend(conditions.descent_1.propulsion.propeller_power_coefficient[:,0])
    cp_forward.extend(conditions.descent_2.propulsion.propeller_power_coefficient[:,0])
    
    #print cp_forward
    
    ct_lift=[]
    
    ct_lift.extend(conditions.climb_1.propulsion.propeller_thrust_coefficient_lift[:,0])
    ct_lift.extend(conditions.climb_2.propulsion.propeller_thrust_coefficient_lift[:,0])
    ct_lift.extend(conditions.cruise.propulsion.propeller_thrust_coefficient_lift[:,0])
    ct_lift.extend(conditions.descent_1.propulsion.propeller_thrust_coefficient_lift[:,0])
    ct_lift.extend(conditions.descent_2.propulsion.propeller_thrust_coefficient_lift[:,0])
    
    #print ct_lift
    
    
    ct_forward=[]
    
    ct_forward.extend(conditions.climb_1.propulsion.propeller_thrust_coefficient_forward[:,0])
    ct_forward.extend(conditions.climb_2.propulsion.propeller_thrust_coefficient_forward[:,0])
    ct_forward.extend(conditions.cruise.propulsion.propeller_thrust_coefficient_forward[:,0])
    ct_forward.extend(conditions.descent_1.propulsion.propeller_thrust_coefficient_forward[:,0])
    ct_forward.extend(conditions.descent_2.propulsion.propeller_thrust_coefficient_forward[:,0])
    
    #print ct_forward
    
    #cwd = os.getcwd()
            
    #myfile=cwd+'/'+"base_data.txt"

    ## If file exists, delete it ##
    #if os.path.isfile(myfile):
    #    os.remove(myfile)
    #else:    ## Show an error ##
    #    print("Error: %s file not found" % myfile)
    
    #myfile=cwd+'/'+"cruise_data.txt"

    ## If file exists, delete it ##
    #if os.path.isfile(myfile):
    #    os.remove(myfile)
    #else:    ## Show an error ##
    #    print("Error: %s file not found" % myfile)
   
    write_optimization_outputs(nexus, filename)
    
    return nexus


