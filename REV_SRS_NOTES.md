# Rev Smart Robot Servo programming notes

## Angle limits
- to set center point:
	1. you can't
- to set sides:
	1. move to side, at least 20 degrees away from center point
	2. press "left" or "right", depending on the way it is rotated from the center point
	3. press program to set changes

## Switching Modes
- to switch to Servo / Continuous rotation servo:
	1. switch to C / S
	2. press program

## Resetting Defaults
- to reset defaults:	
	1. hold program for 5 seconds. lights will flash to indicate reset

## Testing Rotation
- press test to have the servo automatically rotate
- press test again to manually choose position:
	- in C mode:
		1. left rotates left until stopped.
		2. right rotates right until stopped.
		3. program moves it back to center.
	- in S mode:
		1. left rotates left until the limit (135 degrees by default)
		2. right rotates right until the limit (135 degrees by default)
		3. program moves it back to center.