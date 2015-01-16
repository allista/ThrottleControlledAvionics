#Throttle Controlled Avionics

Current developers: 
[qfeys](http://forum.kerbalspaceprogram.com/members/45099-qfeys),
[Zenka](http://forum.kerbalspaceprogram.com/members/108134-Zenka),
[allista](http://forum.kerbalspaceprogram.com/members/102693-allista).

If interested, contact us on the kerbal forums either personally or in the [TCA thread](http://forum.kerbalspaceprogram.com/threads/67270)

***

##Introduction

TCA is a plugin that enhances the attitude control of a ship by dynamically changing the thrust of its engines. Even with most unbalanced designs TCA maintains stable flight and quick response to user or autopilot input. And optional vertical speed limiter enables precise control over ship's altitude.

##Features

* **Per-vessel configuration** profiles and **custom presets**.
* Support for **multiple** control modules (**cockpits**, pods, probe-cores, etc.)
* Sophisticated **parameter tuning**, both automatic and manual.
* Built-in **altitude and hovering control**.
* Both the **Toolbar and** the stock **AppLauncher** are supported.

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

###Vertical Speed Limit, hovering and horizontal flight:

If you're using TCA to control VTOL or during vertical landing of a rocket, you may enable the Vertical Speed Limiter. 

The limit may be set with the scroll bar in the (configurable) interval from -10m/s to 10m/s (not including). When the Limit is set, the total thrust of all controllable engines is modified in an attempt to reach the specified vertical speed. The speed limit itself is never achieved, however, but approached asymptotically, so you need to set it a little higher (0.1-0.5m/s) than desired.

To completely disable the Speed Limit, just set it to maximum value (default 10m/s).

Another use of the Vertical Speed Limit is a stable horizontal flight. Consider a VTOL that has lifted off, reached some altitude and started to pitch to get some forward momentum. If the thrust of its engines will remain constant, it will start to loose altitude as it moves forward. But with the automatic speed limiting the thrust will be adjusted, and the VTOL will move more or less in a plane.

###Notes:

* If your ship wobbles and oscillates with TCA and SAS enabled, rebuild it with more struts, or decrease appropriate Steering Gains.
* Thrust of jets and turbofan engines changes very slowly. This makes using them as attitude controllers impractical. Don't use them with TCA. 
* Solid boosters have constant thrust and thus cannot be controlled by TCA.

##Future Plans:

* Automatically match speed with target.
* Translations with non RCS-enabled engines.
* Explanatory video.