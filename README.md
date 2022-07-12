# Compass_Driven_stepper
this includes the arduino firmware to point a stepper driven platform to a given direction

** How the stepper is controlled **

1. Rotate it clockwise until the limit is triggered
2. then rotate anticlock wise until the limit is triggered again and measure how many stepss needed
3. get the angle per steps by dividing 360/endToEndStepCount
