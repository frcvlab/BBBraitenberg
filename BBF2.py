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

    
def seeback(v1): 
    if v1.lb_detect>0 or v1.rb_detect>0:
        v1.vright = -v1.lb_detect
        v1.vleft= -v1.rb_detect
        return br.Released
    return not br.Released

v1.addBehavior( [ stop,seeback ] )
v1.addBehavior( [ stop,seefront] ) 

v1.behave(showTouch=True) # carry out the behavior


