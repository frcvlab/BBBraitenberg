#
# Example: Behavior Based extension of Braitenros
#

import BBbraitenrosT3 as br

v1 = br.Braitenros(simFlag=False) # simFlag=True for Gazebo, omit for T3



    
 #too close    
def stop(v1): 
    if v1.lf_touch>0 or v1.rf_touch>0 or v1.lb_touch>0 or v1.rb_touch>0:
        v1.vright = 0
        v1.vleft=0
        return br.Released
    return not br.Released
    
def seefront(v1): 
    if v1.lf_detect>0 or v1.rf_detect>0:
        v1.vright = v1.lf_detect
        v1.vleft= v1.rf_detect
        return br.Released
    return not br.Released


v1.addBehavior( [ stop,seefront] ) 

v1.behave() # carry out the behavior


