# cleaningRobot


   
* The code below allows you to control the cleaning robot. It’s possible to manually control the robot, 
activate the fan, virtual wall or run one of implemented algorithms - random, spiral, with a trajectory 
in the shape of the letter "S" and two wall following algorithms that use fuzzy logic.
   
* The standard eFLL library (Embedded Fuzzy Logic Library) allows the construction of fuzzy logic systems
in which the rules may contain only two linguistic variables. The functionality of the library has been
extended to enable the construction of a rule containing three linguistic variables combined with the logical AND operator.
   
* For correctly working fuzzy logic system, you need to replace the eFLL library from project.
 
* Designed and tested on ATmega2560
