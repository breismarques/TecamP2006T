# Plot_Mission.py
# 
# Created:  May 2015, E. Botero
# Modified: 

# ----------------------------------------------------------------------        
#   Imports
# ----------------------------------------------------------------------    

import SUAVE
from SUAVE.Core import Units

import pylab as plt

# ----------------------------------------------------------------------
#   Plot Mission
# ----------------------------------------------------------------------

def plot_mission(nexus):
    results   = nexus.results.base


    # ------------------------------------------------------------------    
    #   Throttle
    # ------------------------------------------------------------------
    plt.figure("Throttle History")
    axes = plt.gca()
    i=0
    for segment in results.segments.values():
        time = segment.conditions.frames.inertial.time[:,0] / Units.min
        eta  = segment.conditions.propulsion.throttle[:,0]
        eta_lift = segment.conditions.propulsion.lift_throttle[:,0]
        
        axes.plot(time, eta, 'ko-', label='Cruise Throttle' if i == 0 else "")
        axes.plot(time, eta_lift, 'bo-', label='HL Throttle' if i == 0 else "")
        i=i+1
    axes.set_xlabel('Time (mins)')
    axes.set_ylabel('Throttle')
    axes.get_yaxis().get_major_formatter().set_scientific(False)
    axes.get_yaxis().get_major_formatter().set_useOffset(False)
    plt.legend(loc='best')
    plt.ylim((0,1))
    axes.grid(True)         

    # ------------------------------------------------------------------    
    #   Altitude
    # ------------------------------------------------------------------
    plt.figure("Altitude")
    axes = plt.gca()    
    for segment in results.segments.values():     
        time     = segment.conditions.frames.inertial.time[:,0] / Units.min
        altitude = segment.conditions.freestream.altitude[:,0] / Units.km
        axes.plot(time, altitude, 'bo-')
    axes.set_xlabel('Time (mins)')
    axes.set_ylabel('Altitude (km)')
    axes.grid(True)    
    
    # ------------------------------------------------------------------    
    #   Aerodynamics
    # ------------------------------------------------------------------
    fig = plt.figure("Aerodynamic Forces")
    for segment in results.segments.values():
        
        time   = segment.conditions.frames.inertial.time[:,0] / Units.min
        Lift   = -segment.conditions.frames.wind.lift_force_vector[:,2]
        Drag   = -segment.conditions.frames.wind.drag_force_vector[:,0]
        Thrust = segment.conditions.frames.body.thrust_force_vector[:,0]

        axes = fig.add_subplot(3,1,1)
        axes.plot( time , Lift , 'bo-' )
        axes.set_xlabel('Time (min)')
        axes.set_ylabel('Lift (N)')
        axes.get_yaxis().get_major_formatter().set_scientific(False)
        axes.get_yaxis().get_major_formatter().set_useOffset(False)          
        axes.grid(True)
        
        axes = fig.add_subplot(3,1,2)
        axes.plot( time , Drag , 'bo-' )
        axes.set_xlabel('Time (min)')
        axes.set_ylabel('Drag (N)')
        axes.get_yaxis().get_major_formatter().set_scientific(False)
        axes.get_yaxis().get_major_formatter().set_useOffset(False)          
        axes.grid(True)
        
        axes = fig.add_subplot(3,1,3)
        axes.plot( time , Thrust , 'bo-' )
        axes.set_xlabel('Time (min)')
        axes.set_ylabel('Thrust (N)')
        axes.get_yaxis().get_major_formatter().set_scientific(False)
        axes.get_yaxis().get_major_formatter().set_useOffset(False)  
        plt.ylim((-10,2500))
        axes.grid(True)
        
    # ------------------------------------------------------------------    
    #   Aerodynamics 2
    # ------------------------------------------------------------------
    fig = plt.figure("Aerodynamic Coefficients")
    for segment in results.segments.values():
        
        time   = segment.conditions.frames.inertial.time[:,0] / Units.min
        CLift  = segment.conditions.aerodynamics.lift_coefficient[:,0]
        CDrag  = segment.conditions.aerodynamics.drag_coefficient[:,0]
        Drag   = -segment.conditions.frames.wind.drag_force_vector[:,0]
        Thrust = segment.conditions.frames.body.thrust_force_vector[:,0]

        axes = fig.add_subplot(3,1,1)
        axes.plot( time , CLift , 'bo-' )
        axes.set_xlabel('Time (min)')
        axes.set_ylabel('CL')
        axes.get_yaxis().get_major_formatter().set_scientific(False)
        axes.get_yaxis().get_major_formatter().set_useOffset(False)          
        axes.grid(True)
        
        axes = fig.add_subplot(3,1,2)
        axes.plot( time , CDrag , 'bo-' )
        axes.set_xlabel('Time (min)')
        axes.set_ylabel('CD')
        axes.get_yaxis().get_major_formatter().set_scientific(False)
        axes.get_yaxis().get_major_formatter().set_useOffset(False)          
        axes.grid(True)
        
        axes = fig.add_subplot(3,1,3)
        axes.plot( time , Drag   , 'bo-' )
        axes.plot( time , Thrust , 'ro-' )
        axes.set_xlabel('Time (min)')
        axes.set_ylabel('Drag and Thrust (N)')
        axes.get_yaxis().get_major_formatter().set_scientific(False)
        axes.get_yaxis().get_major_formatter().set_useOffset(False)  
        axes.grid(True)
        
    
    # ------------------------------------------------------------------    
    #   Aerodynamics 2
    # ------------------------------------------------------------------
    fig = plt.figure("Drag Components")
    axes = plt.gca()    
    for i, segment in enumerate(results.segments.values()):
        
        time   = segment.conditions.frames.inertial.time[:,0] / Units.min
        drag_breakdown = segment.conditions.aerodynamics.drag_breakdown
        cdp = drag_breakdown.parasite.total[:,0]
        cdi = drag_breakdown.induced.total[:,0]
        cdc = drag_breakdown.compressible.total[:,0]
        cdm = drag_breakdown.miscellaneous.total[:,0]
        cd  = drag_breakdown.total[:,0]
        
        axes.plot( time , cdp , 'ko-', label='CD_parasite' )
        axes.plot( time , cdi , 'bo-', label='CD_induced' )
        axes.plot( time , cdc , 'go-', label='CD_compressible' )
        axes.plot( time , cdm , 'yo-', label='CD_miscellaneous' )
        axes.plot( time , cd  , 'ro-', label='CD_total'   )
        
        if i == 0:
            axes.legend(loc='upper center')
        
    axes.set_xlabel('Time (min)')
    axes.set_ylabel('CD')
    axes.grid(True)
    
    # ------------------------------------------------------------------    
    #   Current Draw
    # ------------------------------------------------------------------
    plt.figure("Current Draw")
    axes = plt.gca()
    i=0
    for segment in results.segments.values():     
        time     = segment.conditions.frames.inertial.time[:,0] / Units.min
        energy_lift = segment.conditions.propulsion.current_lift[:,0]
        energy_forward = segment.conditions.propulsion.current_forward[:,0]
        axes.plot(time, energy_lift, 'ko-', label='HL Propellers' if i == 0 else "")
        axes.plot(time, energy_forward, 'bo-', label='Cruise Propellers' if i == 0 else "")
        i=i+1
        
    axes.set_xlabel('Time (mins)')
    axes.set_ylabel('Current Draw (Amps)')
    axes.get_yaxis().get_major_formatter().set_scientific(False)
    axes.get_yaxis().get_major_formatter().set_useOffset(False)   
    plt.ylim((0,500))
    plt.legend(loc='best')
    axes.grid(True)  
    
    # ------------------------------------------------------------------    
    #   Motor RPM
    # ------------------------------------------------------------------
    plt.figure("Motor RPM")
    axes = plt.gca()
    i=0
    for segment in results.segments.values():   
        time     = segment.conditions.frames.inertial.time[:,0] / Units.min
        energy_lift = segment.conditions.propulsion.rpm_lift[:,0]
        energy_forward = segment.conditions.propulsion.rpm_forward[:,0]
        axes.plot(time, energy_lift, 'ko-', label='HL Propellers' if i == 0 else "")
        axes.plot(time, energy_forward, 'bo-', label='Cruise Propellers' if i == 0 else "")
        i=i+1
    
    axes.set_xlabel('Time (mins)')
    axes.set_ylabel('Motor RPM')
    axes.get_yaxis().get_major_formatter().set_scientific(False)
    axes.get_yaxis().get_major_formatter().set_useOffset(False) 
    plt.legend(loc='best')
    axes.grid(True)

    # ------------------------------------------------------------------
    #   Flight Conditions
    # ------------------------------------------------------------------
    fig = plt.figure("Flight Conditions")
    for segment in results.segments.values():

        time     = segment.conditions.frames.inertial.time[:,0] / Units.min
        altitude = segment.conditions.freestream.altitude[:,0] / Units.km
        mach     = segment.conditions.freestream.mach_number[:,0]
        aoa      = segment.conditions.aerodynamics.angle_of_attack[:,0] / Units.deg

        axes = fig.add_subplot(3,1,1)
        axes.plot( time , aoa , 'bo-' )
        axes.set_ylabel('Angle of Attack (deg)')
        axes.get_yaxis().get_major_formatter().set_scientific(False)
        axes.get_yaxis().get_major_formatter().set_useOffset(False)          
        axes.grid(True)

        axes = fig.add_subplot(3,1,2)
        axes.plot( time , altitude , 'bo-' )
        axes.set_ylabel('Altitude (km)')
        axes.get_yaxis().get_major_formatter().set_scientific(False)
        axes.get_yaxis().get_major_formatter().set_useOffset(False)          
        axes.grid(True)

        axes = fig.add_subplot(3,1,3)
        axes.plot( time , mach, 'bo-' )
        axes.set_xlabel('Time (min)')
        axes.set_ylabel('Mach Number (-)')
        axes.get_yaxis().get_major_formatter().set_scientific(False)
        axes.get_yaxis().get_major_formatter().set_useOffset(False)          
        axes.grid(True)   
        
    
    # ------------------------------------------------------------------    
    #  Charging Power and Battery Energy
    # ------------------------------------------------------------------
    
    fig = plt.figure("Electric Outputs")
    for segment in results.segments.values():
        
        time   = segment.conditions.frames.inertial.time[:,0] / Units.min 
        charge = segment.conditions.propulsion.battery_draw[:,0]
        energy = segment.conditions.propulsion.battery_energy[:,0] / Units.MJ
        
        axes = fig.add_subplot(3,1,2)
        axes.plot( time , charge , 'bo-' )
        axes.set_ylabel('Charging Power (W)')
        axes.grid(True)
        
        axes = fig.add_subplot(3,1,3)
        axes.plot( time , energy , 'bo-' )
        axes.set_xlabel('Time (min)')
        axes.set_ylabel('Battery Energy (MJ)')
        axes.grid(True)  
    
        
    return

if __name__ == '__main__': 
    main()    
    plt.show()