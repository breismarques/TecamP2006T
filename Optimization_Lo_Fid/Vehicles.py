# Vehicles.py

# Python Imports
import numpy as np
import pylab as plt
from subprocess import call
import time

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
from SUAVE.Input_Output.OpenVSP import vsp_write_x57
from SUAVE.Methods.Propulsion.electric_motor_sizing import size_from_kv
#from SUAVE.Input_Output.OpenVSP import vspaero




def setup():
    
    base_vehicle = vehicle_setup()
    configs = configs_setup(base_vehicle)
    
    return configs


# ----------------------------------------------------------------------
#   Define the Vehicle
# ----------------------------------------------------------------------

def vehicle_setup():
    
    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------    
    
    vehicle = SUAVE.Vehicle()
    vehicle.tag = 'Tecnam_P2006TElectric'    
    
    # ------------------------------------------------------------------
    #   Vehicle-level Properties
    # ------------------------------------------------------------------    

    # mass properties
    vehicle.mass_properties.max_takeoff               = 1400* Units.kilogram 
    vehicle.mass_properties.takeoff                   = 1400 * Units.kilogram   
    vehicle.mass_properties.operating_empty           = 1000 * Units.kilogram 
    vehicle.mass_properties.max_zero_fuel             = 1400 * Units.kilogram 
    vehicle.mass_properties.cargo                     = 80  * Units.kilogram   
    
    # envelope properties
    vehicle.envelope.ultimate_load = 5.7
    vehicle.envelope.limit_load    = 3.8

    # basic parameters
    vehicle.reference_area         = 64.4 * Units['meters**2']  
    vehicle.passengers             = 4
    vehicle.systems.control        = "fully powered" 
    vehicle.systems.accessories    = "medium range"

    # ------------------------------------------------------------------        
    #  Landing Gear
    # ------------------------------------------------------------------        
    # used for noise calculations
    landing_gear = SUAVE.Components.Landing_Gear.Landing_Gear()
    landing_gear.tag = "main_landing_gear"
    
    landing_gear.main_tire_diameter = 0.423 * Units.m
    landing_gear.nose_tire_diameter = 0.3625 * Units.m
    landing_gear.main_strut_length  = 0.4833 * Units.m
    landing_gear.nose_strut_length  = 0.3625 * Units.m
    landing_gear.main_units  = 2    #number of main landing gear units
    landing_gear.nose_units  = 1    #number of nose landing gear
    landing_gear.main_wheels = 1    #number of wheels on the main landing gear
    landing_gear.nose_wheels = 1    #number of wheels on the nose landing gear      
    vehicle.landing_gear = landing_gear
    
    # ------------------------------------------------------------------
    #  Fuselage
    # ------------------------------------------------------------------
    
    fuselage = SUAVE.Components.Fuselages.Fuselage()    
    fuselage.tag = 'fuselage'
    aux = time.time()
    fuselage.time = aux
    
    fuselage.number_coach_seats    = vehicle.passengers
    fuselage.seats_abreast         = 2
    fuselage.seat_pitch            = 0.995     * Units.meter
    fuselage.fineness.nose         = 1.27
    fuselage.fineness.tail         = 1  #3.31
    fuselage.lengths.nose          = 1.16  * Units.meter
    fuselage.lengths.tail          = 4.637 * Units.meter
    fuselage.lengths.cabin         = 2.653 * Units.meter
    fuselage.lengths.total         = 8.45 * Units.meter
    fuselage.lengths.fore_space    =  0.0   * Units.meter
    fuselage.lengths.aft_space     =  0.0   * Units.meter
    fuselage.width                 = 1.1  * Units.meter   #1.22
    fuselage.heights.maximum       = 1.41  * Units.meter
    fuselage.effective_diameter    =  2 * Units.meter
    fuselage.areas.side_projected  = 7.46  * Units['meters**2'] 
    fuselage.areas.wetted          = 25.0  * Units['meters**2'] 
    fuselage.areas.front_projected = 1.54 * Units['meters**2'] 
    fuselage.differential_pressure = 0.0 * Units.pascal # Maximum differential pressure
    
    fuselage.heights.at_quarter_length          = 1.077 * Units.meter
    fuselage.heights.at_three_quarters_length   =  0.5 * Units.meter   #0.621 * Units.meter
    fuselage.heights.at_wing_root_quarter_chord = 1.41  * Units.meter
    
    ## OpenVSP Design
    
    fuselage.OpenVSP_values = Data() # VSP uses degrees directly
    
    #MidFuselage1 Section
    
    fuselage.OpenVSP_values.midfus1 = Data()
    fuselage.OpenVSP_values.midfus1.z_pos=0.03
    
    #MidFuselage2 Section
    
    fuselage.OpenVSP_values.midfus2 = Data()
    fuselage.OpenVSP_values.midfus2.z_pos=0.06
    
    #MidFuselage3 Section
    
    fuselage.OpenVSP_values.midfus3 = Data()
    fuselage.OpenVSP_values.midfus3.z_pos=0.04
    
    #Tail Section
    
    fuselage.OpenVSP_values.tail = Data() 
    fuselage.OpenVSP_values.tail.bottom = Data()
    fuselage.OpenVSP_values.tail.z_pos = 0.039
    fuselage.OpenVSP_values.tail.bottom.angle = -20.0
    fuselage.OpenVSP_values.tail.bottom.strength = 1

   
    
    # add to vehicle
    vehicle.append_component(fuselage)


    # ------------------------------------------------------------------        
    #   Main Wing
    # ------------------------------------------------------------------        
    
    wing = SUAVE.Components.Wings.Main_Wing()
    wing.tag = 'main_wing'
    
    wing.thickness_to_chord      = 0.15
    wing.taper                   = 0.9
    wing.spans.projected         = 9.4 * Units.meter
    wing.chords.root             = 2. * Units.meter
    wing.chords.tip              = wing.chords.root*wing.taper
    wing.chords.mean_aerodynamic = 0.6 * Units.meter
    wing.areas.reference         = (wing.chords.root+wing.chords.tip)*wing.spans.projected/2  
    wing.twists.root             = 0. * Units.degrees
    wing.twists.tip              = 0. * Units.degrees
    wing.dihedral= 1. * Units.degrees
    wing.origin                  = [2.986,0,1.077] # meters
    wing.sweeps.leading_edge     = 1.9 * Units.deg
    wing.aspect_ratio            = (wing.spans.projected*wing.spans.projected)/wing.areas.reference
    wing.span_efficiency         = 0.99*(1-0.0407*(fuselage.width/wing.spans.projected)-1.792*((fuselage.width/wing.spans.projected)**2))
    wing.vertical                = False
    wing.symmetric               = True
    wing.high_lift               = True
    wing.dynamic_pressure_ratio  = 1.0

    
     
    ## Wing Segments
    
    # Root Segment
    
    #segment = SUAVE.Components.Wings.Segment()
    
    #segment.tag                   = 'root'
    #segment.percent_span_location = 0.0
    #segment.twist                 = 0. * Units.deg
    #segment.root_chord_percent    = 1.
    #segment.dihedral_outboard     = 1. * Units.degrees
    #segment.sweeps.quarter_chord  = 0. * Units.degrees
    #segment.thickness_to_chord    = 0.15
    
    #airfoil = SUAVE.Components.Wings.Airfoils.Airfoil()
    #airfoil.coordinate_file       = '/Users/Bruno/Documents/Delft/Courses/2016-2017/Thesis/Code/Airfoils/naca642415.dat'
    
    #segment.append_airfoil(airfoil)
    #wing.Segments.append(segment)
    
    
    # Tip Segment
    
    #segment = SUAVE.Components.Wings.Segment()
    
    #segment.tag                   = 'tip'
    #segment.percent_span_location = 1.0
    #segment.twist                 = 0. * Units.deg
    #segment.root_chord_percent    = 1.
    #segment.dihedral_outboard     = 1. * Units.degrees
    #segment.sweeps.quarter_chord  = 0. * Units.degrees
    #segment.thickness_to_chord    = 0.15
    
    #airfoil = SUAVE.Components.Wings.Airfoils.Airfoil()
    #airfoil.coordinate_file       = '/Users/Bruno/Documents/Delft/Courses/2016-2017/Thesis/Code/Airfoils/naca642415.dat'
    
    #segment.append_airfoil(airfoil)  
    #wing.Segments.append(segment)
    
    # ------------------------------------------------------------------
    #   Flaps
    # ------------------------------------------------------------------
    
    wing.flaps.chord      =  0.20   
    wing.flaps.span_start =  0.1053
    wing.flaps.span_end   =  0.6842
    wing.flaps.type       = 'single_slotted'

    # add to vehicle
    vehicle.append_component(wing)

    # ------------------------------------------------------------------        
    #  Horizontal Stabilizer
    # ------------------------------------------------------------------        
    
    wing = SUAVE.Components.Wings.Wing()
    wing.tag = 'horizontal_stabilizer'
    
    wing.aspect_ratio            = 4.193     
    wing.sweeps.quarter_chord    = 0.0 * Units.deg
    wing.thickness_to_chord      = 0.12
    wing.taper                   = 1.0
    wing.span_efficiency         = 0.733
    wing.spans.projected         = 3.3 * Units.meter
    wing.chords.root             = 0.787 * Units.meter
    wing.chords.tip              = 0.787 * Units.meter
    wing.chords.mean_aerodynamic = (wing.chords.root*(2.0/3.0)*((1.0+wing.taper+wing.taper**2.0)/(1.0+wing.taper))) * Units.meter
    wing.areas.reference         = 2.5971 * Units['meters**2']  
    wing.areas.exposed           = 4.0 * Units['meters**2']  
    wing.areas.wetted            = 4.0 * Units['meters**2']  
    wing.twists.root             = 0.0 * Units.degrees
    wing.twists.tip              = 0.0 * Units.degrees  
    wing.origin                  = [7.789,0.0,0.3314] # meters
    wing.vertical                = False 
    wing.symmetric               = True
    wing.dynamic_pressure_ratio  = 0.9  
    
    # add to vehicle
    vehicle.append_component(wing)
    
    # ------------------------------------------------------------------
    #   Vertical Stabilizer
    # ------------------------------------------------------------------
    
    wing = SUAVE.Components.Wings.Wing()
    wing.tag = 'vertical_stabilizer'    

    wing.aspect_ratio            = 1.407
    wing.sweeps.quarter_chord    = 38.75 * Units.deg
    wing.thickness_to_chord      = 0.12
    wing.taper                   = 1.0
    wing.span_efficiency         = -0.107
    wing.spans.projected         = 1.574 * Units.meter
    wing.chords.root             = 1.2 * Units.meter
    wing.chords.tip              = 0.497 * Units.meter
    wing.chords.mean_aerodynamic = (wing.chords.root*(2.0/3.0)*((1.0+wing.taper+wing.taper**2.0)/(1.0+wing.taper))) * Units.meter
    wing.areas.reference         = 1.761 * Units['meters**2']  
    wing.twists.root             = 0.0 * Units.degrees
    wing.twists.tip              = 0.0 * Units.degrees  
    wing.origin                  = [7.25,0,0.497] # meters
    wing.vertical                = True 
    wing.symmetric               = False
    wing.t_tail                  = False
    wing.dynamic_pressure_ratio  = 1.0
        
    # add to vehicle
    vehicle.append_component(wing)

    # ------------------------------------------------------------------
    #  Propellers Powered By Batteries
    # ------------------------------------------------------------------    
    
    # build network
    net = SUAVE.Components.Energy.Networks.Lift_Forward_Propulsor()
    
    net.nacelle_diameter_lift     = 0.08 * Units.meters
    net.nacelle_diameter_forward  = 0.1732 * Units.meters
    net.engine_length_lift        = 0.47244 * Units.meters
    net.engine_length_forward     = 1.2 * Units.meters
    net.number_of_engines_lift    = 12
    net.number_of_engines_forward = 2
    net.thrust_angle_lift         = 0.0 * Units.degrees
    net.thrust_angle_forward      = 0.0 * Units.degrees
    net.voltage           = 461. * Units['volt']  #461.
    net.areas_forward             = Data()
    net.areas_forward.wetted      = 1.1*np.pi*net.nacelle_diameter_forward*net.engine_length_forward
    net.areas_lift             = Data()
    net.areas_lift.wetted      = 1.1*np.pi*net.nacelle_diameter_forward*net.engine_length_lift
    net.number_of_engines         = 1
    net.nacelle_diameter          = net.nacelle_diameter_forward 
    net.areas                    = Data()
    net.areas.wetted              =  net.areas_lift.wetted
    net.engine_length            = 1.
    
    # Component 1 - Tip ESC 
    esc = SUAVE.Components.Energy.Distributors.Electronic_Speed_Controller()
    esc.efficiency = 0.95 # Gundlach for brushless motors
    net.esc_forward        = esc
    
    # Component 1 - High Lift ESC 
    esc = SUAVE.Components.Energy.Distributors.Electronic_Speed_Controller()
    esc.efficiency = 0.95 # Gundlach for brushless motors
    net.esc_lift        = esc
    
    # Component 2 - Tip Propeller
    
    # Design the Propeller
    #prop_attributes = Data()
    
    
    #prop_attributes                     = propeller_design(prop_attributes)
    
    prop_forward = SUAVE.Components.Energy.Converters.Propeller_Lo_Fid()
    prop_forward.number_blades       = 3.0
    prop_forward.propulsive_efficiency = 0.85
    prop_forward.freestream_velocity = 50.0 * Units['m/s'] # freestream m/s
    prop_forward.angular_velocity    = 27500. * Units['rpm'] # For 20x10 prop
    prop_forward.tip_radius          = 0.5 * Units.meter
    prop_forward.hub_radius          = 0.085 * Units.meter
    prop_forward.design_Cl           = 0.8
    prop_forward.design_altitude     = 14.0 * Units.km
    prop_forward.design_thrust       = 0.0 * Units.newton
    prop_forward.design_power        = 60000. * Units.watts 
    
    prop_forward  = propeller_design(prop_forward)
    net.propeller_forward = prop_forward
    
    # Component 2 - High Lift Propeller
    
    # Design the Propeller
    #prop_attributes = Data()  
    
    prop_lift = SUAVE.Components.Energy.Converters.Propeller_Lo_Fid()
    prop_lift.number_blades       = 5.0
    prop_lift.propulsive_efficiency = 0.85
    prop_lift.freestream_velocity = 1. * Units['m/s'] # freestream m/s
    prop_lift.angular_velocity    = 2750 * Units['rpm'] # For 20x10 prop
    prop_lift.tip_radius          = 0.26 * Units.meter  #0.2880360
    prop_lift.hub_radius          = 0.07772400 * Units.meter
    prop_lift.design_Cl           = 0.8
    prop_lift.design_altitude     = 0.0 * Units.meter
    prop_lift.design_thrust       = 0.0 * Units.newton
    prop_lift.design_power        = 10500. * Units.watts 
    prop_lift                     = propeller_design(prop_lift)

    net.propeller_lift = prop_lift
    
    
    # Component 3 - Tip Motor
    
    motor = SUAVE.Components.Energy.Converters.Motor_Lo_Fid()
    #motor.resistance           = 1.
    #motor.no_load_current      = 7.  * Units.ampere
    #motor.speed_constant       = 11.9999 * Units['rpm/volt'] # RPM/volt converted to (rad/s)/volt    
    #motor.propeller_radius     = prop_forward.tip_radius
    #motor.propeller_Cp         = prop_forward.Cp[0]
    #motor.gear_ratio           = 12. # Gear ratio
    #motor.gearbox_efficiency   = .98 # Gear box efficiency
    #motor.expected_current     = 160. # Expected current
    #motor.mass_properties.mass = 9.0  * Units.kg
    #net.motor_forward          = motor
    
    kv                         = 180. * Units['rpm/volt'] # RPM/volt is standard
    motor                      = size_from_kv(motor, kv)    
    motor.gear_ratio           = 1. # Gear ratio, no gearbox
    motor.gearbox_efficiency   = .98 # Gear box efficiency, no gearbox
    motor.motor_efficiency     = 0.825;
    net.motor_forward                  = motor
    
    # Component 3 - High Lift Motor
    
    motor = SUAVE.Components.Energy.Converters.Motor_Lo_Fid()
    #motor.resistance           = 0.008
    #motor.no_load_current      = 4.5  * Units.ampere
    #motor.speed_constant       = 5800. * Units['rpm/volt'] # RPM/volt converted to (rad/s)/volt
    #motor.propeller_radius     = prop_lift.tip_radius
    #motor.propeller_Cp         = prop_lift.Cp[0]
    #motor.gear_ratio           = 12. # Gear ratio
    #motor.gearbox_efficiency   = .98 # Gear box efficiency
    #motor.expected_current     = 25. # Expected current
    #motor.mass_properties.mass = 6.0  * Units.kg
    #net.motor_lift             = motor
    
    kv                         = 90. * Units['rpm/volt'] # RPM/volt is standard
    motor                      = size_from_kv(motor, kv)    
    motor.gear_ratio           = 1. # Gear ratio, no gearbox
    motor.gearbox_efficiency   = .98 # Gear box efficiency, no gearbox
    motor.motor_efficiency     = 0.825;
    net.motor_lift                  = motor
    
    # Component 4 - the Payload
    payload = SUAVE.Components.Energy.Peripherals.Payload()
    payload.power_draw           = 50. * Units.watts 
    payload.mass_properties.mass = 5.0 * Units.kg
    net.payload                  = payload
    
    # Component 5 - the Avionics
    avionics = SUAVE.Components.Energy.Peripherals.Avionics()
    avionics.power_draw = 50. * Units.watts
    net.avionics        = avionics      

    # Component 6 - the Battery
    bat = SUAVE.Components.Energy.Storages.Batteries.Constant_Mass.Lithium_Ion()
    bat.mass_properties.mass = 386.0 * Units.kg
    bat.specific_energy      = 121.8 * Units.Wh/Units.kg  #192.84
    bat.specific_power       = 0.312 * Units.kW/Units.kg  #0.837
    bat.resistance           = 0.32
    bat.max_voltage          = 538. * Units['volt']   #10000.
    initialize_from_mass(bat,bat.mass_properties.mass)
    net.battery              = bat
   
    
    # ------------------------------------------------------------------
    #   Vehicle Definition Complete
    # ------------------------------------------------------------------
    
    # add the energy network to the vehicle
    vehicle.append_component(net)
    
    #print vehicle

    return vehicle

# ----------------------------------------------------------------------
#   Define the Configurations
# ---------------------------------------------------------------------

def configs_setup(vehicle):
    
    # ------------------------------------------------------------------
    #   Initialize Configurations
    # ------------------------------------------------------------------
    configs = SUAVE.Components.Configs.Config.Container()

    base_config = SUAVE.Components.Configs.Config(vehicle)
    base_config.tag = 'base'
    configs.append(base_config)

    # ------------------------------------------------------------------
    #   Cruise Configuration
    # ------------------------------------------------------------------
    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'cruise'
    config.propulsors.propulsor.number_of_engines_lift = 0
    configs.append(config)

    return configs

def simple_sizing(configs):

    base = configs.base
    base.pull_base()

    # zero fuel weight
    base.mass_properties.max_zero_fuel = 0.9 * base.mass_properties.max_takeoff 

    # wing areas
    for wing in base.wings:
        wing.areas.wetted   = 2.0 * wing.areas.reference
        wing.areas.exposed  = 0.8 * wing.areas.wetted
        wing.areas.affected = 0.6 * wing.areas.wetted

    # diff the new data
    base.store_diff()

    # ------------------------------------------------------------------
    #   Landing Configuration
    # ------------------------------------------------------------------
    landing = configs.landing

    # make sure base data is current
    landing.pull_base()

    # landing weight
    landing.mass_properties.landing = 0.85 * base.mass_properties.takeoff

    # diff the new data
    landing.store_diff()

    return