# BBBraitenberg
Python/ROS wrapper that supports Braitenberg like connections and also some Behavior-based programming constructs


BBbraitenros: Behavior-based Braitenros
An open extension of the braitenros python package to allow some behavior-based functionality
November 2023 (v4)

Fordham Robotics and Computer Vision Lab. D. Lyons. 
You can use and modify this software (at your own risk) but please attribute/cite it.

Prereq: ROS1 desktop installed. Must include Gazebo. Turtlebot3 packages installed. (https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)

1.0 Overview 

The braitenros python package was designed to allow a user to specify Braitenberg vehicles (Braitenberg 1986) and have them execute in ROS on a Turtlebot3 (T3) robot or its Gazebo simulation. The laser range finder on the T3 is used to simulate contact sensors on the left and right, front and back of the platform. The camera is used to simulate left and right front light sensors. Left and right motor input commands are used to generate the ROS linear and angular velocities. Positive and negative connections (based on some empirically determined constants) are supported. 
One design objective of braitenros was to allow students with little programming exposure to be able to specify Braitenberg vehicles. Figure 1 shows the python code necessary to define a Braitenberg vehicle with positive contralateral connections of the light sensors to the motors. Clearly it still requires some amount of Python syntax: import, assignment, dot notation, functions and function arguments. Assignment, function and function arguments are likely intuitive for a student with prior math experience, leaving just the import and dot notation to be assimilated. 

                import braitenrosT3 as br
                
                v1=br.Braitenros()
                
                v1.connect( v1.vleft_connect,    v1.rf_light_connect )
                v1.connect( v1.vright_connect,  v1.lf_light_connect )
                
                v1.behave()

(a)	 

![image](https://github.com/frcvlab/BBBraitenberg/assets/19497095/3aea5a95-b168-4fa4-8d6f-e10868f22b08)

**Figure 1: Braitenros code (a) and Braitenberg diagram (b) for Vehicle 1 (Braitenberg 1986).**

A wide range of interesting taxis and kinesis like behaviors can be built using only this framework, including target tracking, wall following, obstacle avoidance and combinations of these (a vehicle is not limited to one set of connections). 
However, without the ability to add more instructions, a user will run into limitations after a few experiments. For this reason, the braitenros package was extended to include some features of Behavior Based programming (Arkin 1998). The disadvantage of the extension is that it requires a greater comfort level with specifying instructions for the robot in a programming language. Nonetheless, the extension does not require knowledge of loops or conditionals, and function definitions and calls are stylized and minimized. 

While the extension does not require any great exposure to computer programming, it nonetheless opens the package for those who do have this background and they are free to make the behavior-based modules as complex as they wish.
The remainder of this document is structure as follows. Section 2 describes how to specify a behavior and how to begin carrying out behaviors on the vehicle. Section 3 describes how behaviors can aggregate their output to the robot: two modes are supported – summing of effects and priority ordering of effects. Behaviors can access global state information and Section 4 describes this. All behaviors can access all sensory inputs and Section 5 presents a list of all the sensory input available to behaviors.

2.0 How to specify a behavior

A behavior is specified as a python function. (Don’t worry if you don’t know python or what a function is in a programming language). The syntax is shown in Figure 2 below.
#a wander behavior
    
        def B1(v1):    # must include this syntax
        
            v1.vright = v1.random()   # use 'equal' rather than connect
            v1.vleft  = v1.random()    # random is predfined to return -0.5 to +0.5
            
            return br.Released            # return whether the behavior is 'released' or not
**Figure 2: Example of a behavior definition.**

Any line that starts with ‘#’ is ignored by braitenros and is just a comment for documentation purposes. The same goes for anything that comes after a ‘#’ in the middle of a line.

Every behavior definition starts with the python keyword ‘def’ followed by the name of the behavior (which can be anything the user wants). This keyword must start in the first column. Following the name of the behavior is the argument list for function: this must always be a single parameter, and that parameter will hold the Braitenberg vehicle name (e.g., v1 in Figure 1 is the Braitenberg vehicle variable). For convenience we will always use v1 as the name of the vehicle in these examples. Following the argument list (which is in parenthesis) is a colon. This is just the standard python syntax for a function definition. If knowing that does not help, then forget it and just remember that this syntax must be followed exactly. The only part you should feel free to change is the name of the behavior.
After this first line of the behavior definition, all the following lines in the behavior definition must all be indented 4 spaces. In specifying a braitenros vehicle, we had to say how the sensors were connected to the motors (using the v1.connect command). In a behavior we just use an equal sign “=” to do this. In the example in Figure 2 we set v1.vright equal to some input – that input could be any of the sensory inputs for the vehicle (all listed in Section 5), and this is the equivalent of the v1.connect we used before. The input we use here is v1.random() which will generate a random number between -0.5 and +0.5. We do the same for v1.vleft.
The final line in every behavior is the return line. The return line will say whether the behavior thinks that it has been ‘released’ (in the sense of the Innate Releasing Mechanism of Ethology). Figure 2 shows a return in which the behavior is released. This is not so important for this behavior, but it will be very important in later sections.

Figure 3 shows everything that must be written into a file (called for example BB1.py – the file names need to end in .py) for this behavior to be carried out. The first step is to import the package, as we did for braitenros. The second step is to create the vehicle, v1=br.Braitenros(). Note that there is an optional argument used here in Figure 3: If we specify the argument simFlag=True between the parenthesis, this tells braitenros that we are going to run a Gazebo simulation and not use the actual T3 robot.
          Import BbbraitenrosT3 as br
          
          v1 = br.Braitenros(simFlag=True) # simFlag=True for Gazebo, omit for T3
          
          # a wander behavior
          def B1(v1): # must include this syntax
              v1.vright = v1.random()   # use ‘equal’ rather than connect
              v1.vleft  = v1.random()    # random is defined to return -0.5 to +0.5
              return br.Released           # return whether the behavior is ‘released’ or not
              
          v1.addBehavior(B1) # add in the behavior you defined to the vehicle
          
          v1.behave() # carry out the behavior
**Figure 3: The full example file for the B1 behavior**

After the behavior is defined, the behavior must be added to the list of behaviors to be carried out when the vehicle begins to behave. This is done with the v1.addBehavior() command. Notice that this line starts at column 0 again – it is not indented. Only the lines that you want to be part of B1 are indented. The final line is the same as for the regular braitenros package: v1.behave(). That line starts everything working and, after a short delay, you will see your Gazebo simulation, or your T3 robot, begin to move.

3.0 Aggregating effects

3.1 Summing Effects

Multiple behaviors can be defined and added to a vehicle using the syntax from Section 2. If, for example, two behaviors are defined, and they each issue values for v1.vleft and v1.vright (the motor velocities) then the result is that the values are summed together and sent to the motors. In fact, this is exactly the same meaning that multiple connections have in the regular braitenros package. Lots of interesting physical behaviors can be constructed in this way.

3.2 Priority Effects

It is also possible to add behaviors in such a way that some behaviors have higher priority and will override the effects of lower priority behaviors. Figure 4 shows an example of a second behavior B2 and a priority aggregation of the effects of the behaviors B1 (from Figure 3) and B2. To understand how this is done, first we need to explore B2 a little.

The first line after the ‘def’ of B2 (indented 4 spaces) is a conditional statement used to determine of this behavior satisfies its IRM condition: that the sum of the light intensity picked up by the right and left light sensors (v1.rf_light and v1.lf_light) are greater than an intensity of 1 (which is a small intensity).  If it is, then a familiar contralateral connection of light to motors is specified, with a weight factor of 0.05. The weight factor controls how much the light affects the motor (bigger is more effect). Note that these lines are indented another 4 spaces (that is 8 in total), indicating they are all part of what must be done if the condition is true. The final line returns that the behavior is released. If the condition does not hold, then it is returned that the behavior is not released (return not br.Released). You can tell this is not part of the conditional because it is indented 4 columns, not 8. The indentation tells us what lines should ‘go together.’

        def B2(v1):
            if (v1.rf_light+v1.lf_light)>1: # IRM: check if there is much light in view
                v1.vright=0.05*v1.lf_light  # contralateral 'connection' with weight
                v1.vleft =0.05*v1.rf_light
                return br.Released # say the behavior is released if you see light
            return not br.Released # and not otherwise
                
        v1.addBehavior([B2,B1]) # add in the behaviors you defined to the vehicle
                                # in priority order between square parenthesis
                                # B2 is highest priority and B1 is only carried out if 
                                # B2 is 'not released'
**Figure 4: Example of Priority Aggregation of Behaviors**

When we add the two behaviors, we want to add them so that this new behavior has higher priority, and if it is released, then it should prevent the lower B1 priority behavior for having any effect. We indicate this by giving an ordered list of behaviors between square parenthesis to the v1.addBehavior() command. In Figure 4, v1.addBehavior([B2,B1]) will ensure that B2 is higher priority and will cancel the effect of B1 if it is released. If it is not released, then B1 will carry out as it did in the previous section.

4.0 Sensory Inputs

4.1 Left and Right Bump Contact sensors

There are multiple contact sensors, all calculated from the laser range data. The simplest are two Boolean valued bump sensors for the front left and front right regions. How close a vehicle v needs to be to a surface before these are triggered is controlled by the value set in v.tooClose; the initial value is 0.5m. The sensor values are always available in v.bumpLeft and v.bumpRight and have the values True (when too close) and False (otherwise).

4.2 Left and Right, Front and Back Contact sensors

For a vehicle v, the touch sensors v.lf_touch, v.lb_touch, v.rb_touch and v.rf_touch are set to the target distance or cleared to 0 based on whether any laser range reading in a quadrant is closer than v.tooClose as shown in Figure 9.  


![image](https://github.com/frcvlab/BBBraitenberg/assets/19497095/fed852c7-14fe-4cf3-80bb-003a7f569391)

**Figure 9: Mapping touch sensors to the laser range data. The dotted circle around the T3 robot represents the 360 laser range readings (on per degree).**

There is a second set of proximity detection sensors which are set when a target is somewhere around the robot but not too close (as determined by the value of v.tooClose). There are v.lf_detect, v.lb_detect, v.rb_detect and v.rf_detect are set to 1 or cleared to 0 based on whether any laser range reading in a quadrant is in view but not yet too close.

4.3 Left and Right light sensors

For a vehicle v, the left and right sensor values respectively can be accessed at any time in the variables v.lf_light and v.rf_light. However, if you want to specify a connection to the sensor, you should use the variables v.lf_light_connect and vrf_light_connect. The sensory input connection always appears second in the v.connect function: e.g., v.connect( v.vright_connect,  v.lf_light_connect ).

4.4 Light stimulus

The default stimulus for the light sensors is any white region of the image. A sheet of white paper is ideal. However, the target stimulus color can be reset at any time with the function setColorTarget. For a vehicle v, v.setColorTarget(min,max) will set the color target to be any color region whose color value is between the color min and the color max. Both min and max are specified as a tuple of three numbers specifying the blue, green and red color components, each between 0 and 255.  The default white page color target has min=(250,250,250) and max=(255,255,255). A very green stimulus target might be min=(0,250,0) and max=(0,255,255).

4.5 Laser range data

The laser range measurements are also directly available to the user. For vehicle v, the laser range measurements are available as v.laserReadings. This is a list of 360 values, each the distance from the laser range sensor to the closest surface to the robot in that direction. The 0 angle reading is directly in front of the robot and they proceed counter clockwise. 

4.6 Pose of the vehicle

The current x and y location of the vehicle, along with the angle that its center axis makes with the X axis, is also available as a sensory input. This is actually calculated data rather than sensory input and as time goes by, this value gets more and more inaccurate. Nonetheless, it can be very useful for small and short experiments, so it is listed here as an input. The value v.Pose[0] is the x coordinate of the position, and the value v.Pose[1] is the y coordinate of the position. The value v.Pose[2] is the angle, in radians, of the center axis of the robot.

4.7 Additional Misc. Commands

The default configuration of braitenros is to look for the T3 ROS topics. You can switch to the Gazebo simulation by including the parameter simFlag=True when you create the vehicle, e.g., v=br.Braitenros(simFlag=True).

You can view some limited state and camera information if you include the showCamera=True parameter when you call behave for vehicle v, e.g., v.behave(showCamera=True).
To see the state of the touch and detect sensors, use v.behave(showTouch=True).
A plot of all the positions covered since the v.behave() was called can be requested by including v.plotPosition() after the v.behave() line. Note that v.behave() will never terminate on its own; you need to interrupt the vehicle by typing ^C at least once. Note also, that plotPosition will not terminate until you “x out” the plot window. 
You can request a plot of all the motor commands issued so far by including the line v.plotMotors() after the v.behave().

5.0 Global state memory

5.1 Remembering state

Sometimes it is convenient for a behavior to ‘remember’ information about the past. For example, a vehicle might look for bright light and act in a specific way after the light is seen – even if the light is removed. Let’s say the vehicle is ‘scared’ by the light, so we will refer to the piece of global state information that we need to remember by the name ‘scared’. Figure 5 gives an example of how a behavior can use this feature.
The behavior scared in Figure 5 uses a conditional to test when the total light intensity measured by the light sensors is greater than 10 (not a very great intensity as it happens). It commits the success of this conditional to memory by stating v1.remember("scared",True). Now this, or any other behavior, can see the value of “scared”. The behavior runaway in Figure 5 accesses “scared” and if it is True (v1.recall("scared")), it releases a behavior that moves the motors strongly in the forward direction. This release could be used to override whatever other behavior is controlling the vehicle and move the vehicle a large distance.
def scared(v1):

    if (v1.rf_light+v1.lf_light)>10: # too much light
        v1.remember("scared",True)
              else:
                  v1.remember("scared",False)
              return not br.Released
              
          def runaway(v1):
              if v1.recall("scared"):
                  v1.vright = 3*abs(v1.random())   # bigger intensity => kinesis
                  v1.vleft  = 3*abs(v1.random())       
                  return br.Released # return whether the behavior is 'released' or not
              return not br.Released
**Figure 5: Example of setting, resetting and reading stored global state**

5.2 Habituated state

The v1.remember and v1.recall commands can be used to manipulate global state information in a very general way. However, there are some patterns in the way global state information is used. One very useful example is habituated state: state that builds up over time to being triggered and then once triggered, gradually dies down again. The “bored” habituated state example in Figure 6 can be used to prevent a vehicle from getting stuck doing any one thing (e.g., circling a light with phototaxis).
def bored(v1):

              v1.usingHabituated("bored",50,5)  # a 'memory state; get bored after 50 
                                                                             # and then not bored again after 5 more
              if v1.triggered("bored"): # IRM: release if "bored"
                  v1.vright = 0.5 # just move the robot quickly in a straight line
                  v1.vleft   = 0.5
                  return br.Released   # release the behavior when bored  
              return not br.Released  # and not otherwise      
              
          v1.addBehavior([bored,phototaxis,wander]) # add in the behaviors 
                                                                                # in priority order between square parenthesis
**Figure 6: Example of Habituated State used in a behavior**
 
The v1.usingHabituatedState() command allows us to define a global state “bored” that will repeatedly be triggered after 50 time units and then will ‘untrigger’ again after another 5. The behavior bored uses a conditional to evaluate an IRM condition based on whether the state variable “bored” is triggered or not. If it is (v1.triggered("bored")), then it releases a behavior that is a large move straight ahead.
The v1.addBehavior() command adds bored as the highest priority behavior – so if it is released it will prevent the phototaxis and wander behaviors from having any effect.

6.0 Multiple Vehicles

The BBbraitenros package will support multiple vehicles moving simultaneously. For simulation, the correct Gazebo launch file must have been used, starting the number of robots that are to be used. For physical robots, the correct number of robots needs to have been started with rosmaster running on the computer that BBbraitenros will run on. 

The br.Braitenros command must be called to create each vehicle. For example, in Figure 7 below two vehicles are created. The same behavior is added to each vehicle. Separate behaviors, or any mix of behavior, can be added to each vehicle in general. 

To start the collection of vehicles, the br.multipleBehave command must be used. The argument for this command is the list of vehicles that you want to start simultaneously. The vehicles will activate the behaviors and run until a “^C” command is used to interrupt and stop all vehicles. 
The br.multiplePlotPosition command can be used to show the positions of every vehicle on the same graph. 

          Import BbbraitenrosT3 as br
          
          v1 = br.Braitenros(simFlag=True) # simFlag=True for Gazebo, omit for T3
          v2 = br.Braitenros(simFlag=True) # simFlag=True for Gazebo, omit for T3
          
          # a wander behavior
          def B1(v1): # must include this syntax
              v1.vright = v1.random()   # use ‘equal’ rather than connect
              v1.vleft  = v1.random()    # random is defined to return -0.5 to +0.5
              return br.Released           # return whether the behavior is ‘released’ or not
              
          v1.addBehavior(B1) # add in the behavior you defined to the vehicle
          v2.addBehavior(B1)
          
          br.multipleBehave( [v1,v2] ) # carry out the behavior on both vehicles
          
          br.multiplePlotPosition( [v1,v2] ) # plot both positions on 1 graph
**Figure 7: Multiple vehicles with the B1 behavior**

When there are multiple vehicles, it is sometimes useful to know from which vehicle a particular laser originated. For example, if vehicle1 is in front of vehicle 2, then the laser contacts from vehicle1 originate from vehicle2. When a vehicle is created, the argument name can be used to assign a name to that vehicle. The package will attempt to calculate then which name is associated with each touch sensor. The variable lf_touched, lb_touched, etc. will be set to this calculated name. Figure 8 shows an example.

          import BBbraitenrosT3 as br
          
          v1 = br.Braitenros(simFlag=True) 
          v2 = br.Braitenros(simFlag=True,name="blue") # will be the leader
          v3 = br.Braitenros(simFlag=True)
          
          def stop(v): 
              if (v.lf_touch>0 and v.lf_touched=="blue") or 
                  (v.rf_touch>0 and v.rf_touched=="blue"):
                  v.vright = 0
                  v.vleft  = 0
                  return br.Released
              return not br.Released
              
          def seefront(v): 
              if (v.lf_detect>0 ) or (v.rf_detect>0 ):
                  v.vright = 0.25*v.lf_detect
                  v.vleft= 0.25*v.rf_detect
                  return br.Released
              return not br.Released
          
          def backoff(v):
              if 0<v.lf_touch<0.4 or 0<v.rf_touch<0.4:
                  v.vright,v.vleft=-0.15,-0.15
                  return br.Released
              return not br.Released
              
          def circle(v):
              v.vright= 0.2
              v.vleft = 0.2
              return br.Released
          
          v3.tooClose=0.5
          v3.not2Close=4.0
          v3.addBehavior( [ backoff, stop,seefront] ) 
          
          v1.tooClose=0.5
          v1.not2Close=4.0
          v1.addBehavior( [ backoff, stop,seefront] ) 
          
          v2.addBehavior( circle ) # the leader 
          
          br.multipleBehave( [v1,v2,v3] )
          br.multiplePlotPosition( [v1,v2,v3] )
**Figure 8: Multiple vehicles and vehicle naming**

References

(Braitenberg 1986) Braitenberg, V., Vehicles: Experiments in Synthetic Psychology, MIT Press Feb 1986 ISBN: 9780262521123

(Arkin 1998) Behavior-based Robotics,  MIT Press

(Mataric 1998) Mataric, M., Behavior-based Robotics as a tool for synthesis of artificial behavior and analysis of natural behavior. Trends in Cognitive Science V2
