#Throttle Controlled Avionics

Originally created by:
[qfeys](http://forum.kerbalspaceprogram.com/members/45099-qfeys),
[Zenka](http://forum.kerbalspaceprogram.com/members/108134-Zenka)

Current development and maintenance:
[allista](http://forum.kerbalspaceprogram.com/members/102693-allista)

***

##Introduction

TCA is a plugin that enhances the attitude control of a ship by dynamically changing the thrust of its engines. Even with most unbalanced designs TCA maintains stable flight and quick response to user or autopilot input. And optional vertical speed limiter enables precise control over ship's altitude.

##Features

* **Per-vessel configuration** profiles and **custom presets**.
* Support for **multiple** control modules (**cockpits**, pods, probe-cores, etc.)
* Sophisticated **parameter tuning**, both automatic and manual.
* Built-in **altitude and hovering control**.
* Built-in **autopilot to kill horizontal speed**.
* Both the **Toolbar and** the stock **AppLauncher** are supported.
* **In-game** configurable **key binding**.
* **Career mode** integration.

##Instructions

###For simple use:

1. Turn TCA on (default 'y'),
2. Turn SAS on (default 't'),
3. Launch!

###Autotuning Parameters:

If this option is enabled, TCA calculates Steering Gains and PI coefficients as functions of maximum possible angular acceleration along each of the principal axes of a craft. Note, that calculations are based on the predefined response curves tuned for the stock SAS.

If you have already tuned these parameters for the ship, save its configuration before enabling, as autotuning will overwrite previous parameters.

###Steering Gains:

Master Gain modifies the magnitude of all steering input after other modifications. If the craft reacts slowly, increase it a little. If the craft vibrates after a maneuver, decrease it.

Pitch, Yaw and Roll modify only corresponding inputs. Linking Pitch and Yaw useful if the ship's engines are symmetrical along main axis. In that case Pitch&Yaw slider may be used in stead of Master Gain.

###Engines PI-controller tuning:

* P (proportional) parameter controls the response speed of the engines. If the craft reacts slowly, increase it a little. If the craft vibrates after a maneuver, decrease it.
* I (integral) parameter controls the smoothing. It prevents rapid changes in thrust. If engines's trust jitters, increase it.

###Vertical Speed Control, hovering and horizontal flight:

If you're using TCA to control VTOL or during vertical landing of a rocket, you may enable the Vertical Speed Control System. 

The desired vertical speed may be set with the scroll bar in the (configurable) interval from -10m/s to 10m/s (not including). Then the total thrust of all controllable engines is modified in an attempt to reach the specified vertical speed. This speed, however, is never achieved, but approached asymptotically, so you need to set it a little higher (0.1-0.5m/s) than desired.

To completely disable the VSC, just set it the desired speed to its maximum value (default 10m/s).

VSC is also very useful to maintain stable horizontal flight. Consider a VTOL that has lifted off, reached some altitude and started to pitch to get some forward momentum. If the thrust of its engines will remain constant, it will start to loose altitude as it moves forward. But with the VSC the thrust will be adjusted, and the VTOL will move more or less in a plane.

###Set vertical speed with throttle controls:
When this option is enabled, the throttle is locked at 100% and throttle controls are used to set desired vertical speed instead. If VSC system was switched off it is automatically enabled and the desired speed is set to 0.

###Kill Horizontal Velocity:
Enables an autopilot that tries to maneuver the craft so that the horizontal component of its velocity became zero. It includes flip-over prevention system, so whatever the speed, the autopilot decrease it carefully and steady.

###Notes:

* For safety reasons the Vertical and Horizontal speed controls are disabled in orbit, but not on suborbital trajectories, so be carefull.
* If your ship wobbles and oscillates with TCA and SAS enabled, rebuild it with more struts, or decrease appropriate Steering Gains.
* Thrust of jets and turbofan engines changes very slowly. This makes using them as attitude controllers impractical. Don't use them with TCA. 
* Solid boosters have constant thrust and thus cannot be controlled by TCA.

##Future Plans:

* **Priority:**
* Ideas:
    * Autotuning for MechJeb2
* Legacy:
    * Automatically match speed with target
    * Translations with non RCS-enabled engines

##Acknowledgments:

First of all, many thanks to **qfeys** and **Zenka** for creating the original concept and implementation. Without them TCA would not exist.

And here are the mods whose code and ideas were used in one way or another:

* [Hangar](http://forum.kerbalspaceprogram.com/threads/88933)
* [MechJeb2](http://forum.kerbalspaceprogram.com/threads/12384)
