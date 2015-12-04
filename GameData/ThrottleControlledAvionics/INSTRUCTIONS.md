#TCA Manual

###For simple use:

1. Turn TCA on
2. Turn SAS on
3. Activate the engines and throttle them up!

###Engine Modes:

In editor or in flight (through the part menu) you may set any engine to work in one of the three modes: 

1. **Main Engine** (_default_). TCA tries to maximize the thrust of these engines. In a perfectly balanced ship all Main Engines should have 100% thrust in the absence of control input. These engines are also used to control vertical speed.
2. **Maneuver Engine**. TCA tries to minimize the thrust of these engines. In a perfectly balanced ship these engines produce thrust only in response to control input
3. **Manual Control**. TCA does not change the thrust of these engines, but includes them in calculations.
4. **Balanced Thrust**.

###Autotuning Parameters:

* If this option is enabled, TCA calculates Steering Gains and PI coefficients as functions of maximum possible angular acceleration along each of the principal axes of a craft. Note, that calculations are based on the predefined response curves tuned for the stock SAS.
* If you have already tuned these parameters for the ship, save its configuration before enabling, as autotuning will overwrite previous parameters.

###Steering Gains:

* Master Gain modifies the magnitude of all steering input after other modifications. If the craft reacts slowly, increase it a little. If the craft vibrates after a maneuver, decrease it.
* Pitch, Yaw and Roll modify only corresponding inputs.
* Linking Pitch and Yaw useful if the ship's engines are symmetrical along main axis. In that case Pitch&Yaw slider may be used in stead of Master Gain.

###Engines PI-controller tuning:

* P (proportional) parameter controls the response speed of the engines. If the craft reacts slowly, increase it a little. If the craft vibrates after a maneuver, decrease it.
* I (integral) parameter controls the smoothing. It prevents rapid changes in thrust. If engines's trust jitters, increase it.

###Vertical Speed Control, hovering and horizontal flight:

* If you're using TCA to control VTOL or during vertical landing of a rocket, you may enable the Vertical Speed Control System.
* The desired vertical speed may be set with the scroll bar. Then the total thrust of all controllable engines is modified in an attempt to reach the specified vertical speed.
* To completely disable the VSC, just set the desired vertical speed to its maximum value.
* VSC is also very useful to maintain stable horizontal flight. Consider a VTOL that has lifted off, reached some altitude and started to pitch to get some forward momentum. If the thrust of its engines will remain constant, it will start to loose altitude as it moves forward. But with the VSC the thrust will be adjusted, and the VTOL will move more or less in a plane.

###Set vertical speed with throttle controls:

* When this option is enabled, the throttle is locked at 100% and throttle controls are used to set desired vertical speed instead. If VSC system was switched off it is automatically enabled and the desired speed is set to 0.

###Kill Horizontal Velocity:

* Enables an autopilot that tries to maneuver the craft so that the horizontal component of its velocity became zero. It includes flip-over prevention system, so whatever the speed, the autopilot decrease it carefully and steady.

###Notes:

* For safety reasons the Vertical and Horizontal speed controls are disabled in orbit, but not on suborbital trajectories, so be carefull.
* If your ship wobbles and oscillates with TCA and SAS enabled, rebuild it with more struts, or decrease appropriate Steering Gains.
* Solid boosters have constant thrust and thus cannot be controlled by TCA. But they are still accounted for, if present.
