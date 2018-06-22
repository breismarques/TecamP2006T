# Missions.py

# Python Imports
import numpy as np
import pylab as plt
from subprocess import call

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
#from SUAVE.Input_Output.OpenVSP import vspaero

# ----------------------------------------------------------------------
#   Define the Mission
# ----------------------------------------------------------------------

def mission_setup(analyses,vehicle):

    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission = SUAVE.Analyses.Mission.Sequential_Segments()
    mission.tag = 'The Test Mission'

    mission.atmosphere  = SUAVE.Attributes.Atmospheres.Earth.US_Standard_1976()
    mission.planet      = SUAVE.Attributes.Planets.Earth()
    
    
    # ------------------------------------------------------------------
    #   First Climb Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------
    
    # unpack Segments module
    #Segments = SUAVE.Analyses.Mission.Segments
    
    # all motors segment
    #all_motors_segment = Segments.Segment()   
    #ones_row     = all_motors_segment.state.ones_row
    
    
    #all_motors_segment.state.unknowns.lift_throttle                    = 0.8   * ones_row(1)
    #all_motors_segment.state.unknowns.propeller_power_coefficient_lift = 0.01 * ones_row(1)
    #all_motors_segment.state.unknowns.propeller_power_coefficient  = vehicle.base.propulsors.propulsor.propeller_forward.prop_attributes.Cp  * ones_row(1)
    #all_motors_segment.state.unknowns.battery_voltage_under_load   = vehicle.base.propulsors.propulsor.battery.max_voltage * ones_row(1)
    
    #all_motors_segment.process.iterate.unknowns.network  = vehicle.base.propulsors.propulsor.unpack_unknowns
    #all_motors_segment.process.iterate.residuals.network = vehicle.base.propulsors.propulsor.residuals   
    #all_motors_segment.process.iterate.initials.initialize_battery = SUAVE.Methods.Missions.Segments.Common.Energy.initialize_battery
    #all_motors_segment.state.residuals.network           = 0. * ones_row(4)

    #segment = Segments.Climb.Constant_Speed_Constant_Rate(all_motors_segment)
    #segment.tag = "climb_1"

    #segment.analyses.extend( analyses.takeoff )

    
    #segment.state.numerics.number_control_points = 16
    #segment.altitude_start = 0.0   * Units.meter
    #segment.altitude_end   = 100   * Units.meter
    #segment.air_speed      = 64 * Units.knots
    #segment.climb_rate     = 0.5  * Units['m/s']
    #segment.battery_energy = vehicle.base.propulsors.propulsor.battery.max_energy*0.2

    # add to misison
    #mission.append_segment(segment)
    
    
    # ------------------------------------------------------------------    
    #   Cruise Segment: constant speed, constant altitude
    # ------------------------------------------------------------------  
    
    # unpack Segments module
    Segments = SUAVE.Analyses.Mission.Segments
    
    
    # no lifting motors segment
    no_lift_segment = Segments.Segment()   
    ones_row     = no_lift_segment.state.ones_row
    
    
    no_lift_segment.state.unknowns.lift_throttle                    = 0.8   * ones_row(1)
    no_lift_segment.state.unknowns.propeller_power_coefficient_lift = 0.01 * ones_row(1)
    no_lift_segment.state.unknowns.propeller_power_coefficient  = vehicle.base.propulsors.propulsor.propeller_forward.Cp[0]  * ones_row(1)
    no_lift_segment.state.unknowns.battery_voltage_under_load   = vehicle.base.propulsors.propulsor.battery.max_voltage * ones_row(1)
    no_lift_segment.state.unknowns.__delitem__('propeller_power_coefficient_lift')
    no_lift_segment.state.unknowns.__delitem__('lift_throttle')
    no_lift_segment.state.unknowns.throttle=0.8   * ones_row(1)
    
    no_lift_segment.process.iterate.unknowns.network  = vehicle.base.propulsors.propulsor.unpack_unknowns_no_lift
    no_lift_segment.process.iterate.residuals.network = vehicle.base.propulsors.propulsor.residuals_no_lift    
    no_lift_segment.process.iterate.initials.initialize_battery = SUAVE.Methods.Missions.Segments.Common.Energy.initialize_battery
    no_lift_segment.state.residuals.network           = 0. * ones_row(2)
    
    segment = SUAVE.Analyses.Mission.Segments.Cruise.Constant_Speed_Constant_Altitude(no_lift_segment)
    segment.tag = "cruise"
    
    # connect vehicle configuration
    segment.analyses.extend(analyses.cruise)
    
    # segment attributes     
    segment.state.numerics.number_control_points = 16
    segment.altitude       = 1000. * Units.meter
    segment.air_speed  = 70. * Units.knots
    segment.distance       = 25. * Units.nautical_miles
    segment.battery_energy = vehicle.base.propulsors.propulsor.battery.max_energy
    
    mission.append_segment(segment)    
    

    # ------------------------------------------------------------------    
    #   Mission definition complete    
    # ------------------------------------------------------------------
    
    return mission

def missions_setup(base_mission):

    # the mission container
    missions = SUAVE.Analyses.Mission.Mission.Container()

    # ------------------------------------------------------------------
    #   Base Mission
    # ------------------------------------------------------------------

    missions.base = base_mission

    return missions