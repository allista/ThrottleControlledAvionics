#TCA Reference Manual (WIP)

##Overview

TCA is a _software_ that can be installed onto any command module, cockpit or probe core. To do it in _career_ mode you should buy it in **Specialized Control**; in sandbox it is installed by default. Whether TCA is installed on a particular part is indicated in part's info pane in Editor.

When TCA is bought in R&D, all qualified parts in flight are upgraded using the OTA update. However, TCA interface and functionality only becomes available if a vessel has engines and/or RCS thrusters.

###What TCA can and cannot do

At the technical level, all that TCA does is it changes thrust limiters of engines and RCS thrusters in real time. Using this "simple" technique TCA controls both total thrust and torque applied to the vessel, which, in its turn, allows to do all the other wonderful things.

This of course means that TCA can do nothing with vessels that don't have neither engines nor RCS; but even a rocket with a *single* engine and enough control authority (provided by RCS or reaction wheels) can benefit from TCA.

So, what TCA can do?

###RCS balancing

If a vessel has RCS thrusters (and RCS is enabled), TCA automatically tries to change their thrust limiters, so that the torque generated conformed to the control input. In particular, when only translational controls are used, TCA tries to eliminate or at least minimize the undesired torque. **Note**, that with some physically unbalanced ship designs this may effectively disable RCS thrust almost completely.

###Engines balancing

Engines' balancing is performed in the same general manner as RCS balancing, but with much more control over the process: an engine always has an assigned TCA Role (or *mode*) which affects how its thrust limiter is handled; and there're several parameters in the *Advanced* section that control engines' response to control input.

###Attitude Control (T-SAS)

TCA has its own SAS with all the standard attitude cues. It may seem redundant if not for the two things: first, in most cases it performs better than stock SAS; second, and most important, *it controls the attitude of the total thrust vector*, not the attitude of a control module. Hence **T**-SAS.

###Vertical Speed Control (VSC)

TCA can automatically control the vertical speed of a ship that has enough engines pointed downward. This is obviously intended for VTOLs, but may also be used for a soft touchdown of any standard rocket.

###Altitude Control

Even more automatically, TCA itself may use VSC to maintain the desired altitude. And if the *Follow Terrain* mode is enabled, Altitude Control instead tries to maintain vessel's relative altitude above the ground.

*Important to note*, that *Follow Terrain* mode implicitly enables the built-in *Radar* that is used by TCA to avoid collisions with the terrain. Try to fly at 20m across the KSC in this mode...

###Horizontal Speed Control (HSC)

TCA can also control horizontal speed of a vessel *by changing the direction of the total thrust vector* (i.e. rotating the whole vessel using T-SAS), by using the standard translational controls and lateral engines in *Manual* mode.

###Autopilot programs

All of the above combined provides basis for a set of a much more complex autopilot programs, including waypoint navigation, automatic landing and more.

##Engines' Modes and Groups

In editor or in flight, through the part menu or engines' *Profiles* (discussed later) you may set any engine to work in one of the following modes: 

* **Main Engine** (_default_): when balancing, TCA tries to maximize the thrust of these engines. In a perfectly balanced ship all Main Engines should have 100% thrust in the absence of control input. *These engines are used to control vertical speed AND attitude.*
* **Balanced Thrust**: a group of engines in this mode is always balanced so that it does not generate any torque. It is mostly useful for jets-based VTOLs. *These engines are used to control vertical speed.*
* **Maneuver Engine**: when balancing, TCA tries to minimize the thrust of these engines. In a perfectly balanced ship these engines produce thrust only in response to control input. *These engines are used to control attitude and for translation.*
* **Manual Control**: balancing does not change the thrust of these engines, but includes them in calculations. HSC, however, may use them to change horizontal speed of the vessel.

Each engine has also a *group ID*. By default it is set to zero, meaning the engine is not included into any group. But *all the engines sharing the same non-zero group ID will also share the mode and*, if they are Manual engines, *the value of thrust limiters*
. This is a mere convenience to setup engines quicker during construction and to have shorter Profiles.

##Engines' Profiles

##Interface Basics

TCA graphical interface consists of the three separate windows:

* In-Flight main window that controls all the aspects of TCA functionality
* In-Editor configuration window that allows editing of engines profiles and pre-launch TCA state
* Macro Editor window that is available both in flight and in editor

First two windows are summoned by TCA toolbar button. Both Blizzy's Toolbar and the stock AppLauncher are supported; by default the Toolbar is used when it is available, but this may be changed in TCA.glob file (see *Appendix*).

###Tooltips

Most of TCA control elements -- buttons, input fields, sliders -- have tooltips that are displayed on mouse hover. **Pay attention to them**, as they explain verbosely the meaning of short names of these controls.

###Button switches

Many of the TCA functions are controlled through *switches*. These are buttons that are <color=lime>green</color> when the function is enabled, <color=yellow>yellow</color> when it is disabled, and gray when it is unavailable.

###TCA activation and status

To enable/disable TCA, use the **Enabled** switch at the top-left corner of the main window, **or** the hot-key (Y by default). The hot-key may be changed in the *Advanced* section (see *Appendix*).

In flight the icon of the TCA toolbar button changes according to TCA state: "Disabled", "Enabled", "No Electric Charge". A more descriptive status is displayed in the top-right corner of the main window.

##Attitude Control (T-SAS)

##Vertical Speed and Altitude Control

###Hovering and horizontal flight

If you're using TCA to control VTOL or during vertical landing of a rocket, you may enable the Vertical Speed Control.

The desired vertical speed may be set with the slider. Then the total thrust of all controllable engines is modified in an attempt to reach the specified vertical speed. To completely disable the VSC, just set the desired vertical speed to its maximum value.

VSC is also very useful to maintain stable horizontal flight. Consider a VTOL that has lifted off, reached some altitude and started to pitch to get some forward momentum. If the thrust of its engines will remain constant, it will start to loose altitude as it moves forward. But with the VSC the thrust will be adjusted, and the VTOL will move more or less in a plane.

###Follow Terrain mode

###AutoThrottle

When this option is enabled, the throttle is locked at 100% and throttle controls are used to set desired vertical speed instead. If VSC system was switched off it is automatically enabled and the desired speed is set to 0.

##Horizontal Speed Control (HSC)

###Stop

Enables an autopilot that tries to maneuver the craft so that the horizontal component of its velocity became zero. It includes flip-over prevention system, so whatever the speed, the autopilot decrease it carefully and steady.

###Level Flight

###Cruise Control

##On-Planet Autopilots

###Anchor

###Land

###Waypoint Navigation

###GoTo/Follow Target

##In-Orbit Autopilots

###Execute Maneuver

###Match Velocity with Target

###Brake near Target

##Utility Modules

###Radar and Collision Prevention System (CPS)

###VTOL Assist

###Flight Stabilizer

##Squadron Mode

##Macros

###Macro Editor

###In-Flight Macro Controls

##General Notes

For safety reasons the VSC, HSC and On-Planet autopilots are disabled in orbit, but not on suborbital trajectories, so be carefull.

If your ship wobbles and oscillates with TCA and SAS enabled, rebuild it with more struts, or decrease appropriate Steering Gains.

Solid boosters have constant thrust and thus cannot be controlled by TCA. But they are still accounted for, if present.

##Appendix

###Advanced section

Advanced, or rather rarely needed settings are hidden in a separate section of the main window that can be accessed through *advanced* switch located at the left corner of the titlebar.

####Reload TCA Settings

This button reloads **TCA.glob** file from disk.

####TCA hotkey

This button allows to change the key that toggles TCA from keyboard.

####Sensitivity of throttle controls

This slider changes the speed with which Main Throttle keyboard controls change desired vertical speed or altitude.

####Autotuning Engines' Controller:

If this option is enabled, TCA calculates Steering Gains and PI coefficients as functions of maximum possible angular acceleration along each of the principal axes of a ship. **Note**, that calculations are based on the predefined response curves tuned for the stock SAS.

If you have already tuned these parameters for the ship, _save its configuration before enabling autotuning_, as it will overwrite manually changed parameters.

####Steering Gains:

* **Master Gain** modifies the magnitude of all steering input after other modifications. If the craft reacts slowly, increase it a little. If the craft vibrates after a maneuver, decrease it.
* **Pitch, Yaw and Roll** modify only corresponding inputs.
* **Linking** Pitch and Yaw useful if the ship's engines are symmetrical along the main axis. In that case Pitch&Yaw slider may be used in stead of Master Gain.

####Engines' PI-controller tuning:

* **P** (*proportional*) parameter controls the response speed of the engines. If the craft reacts slowly, increase it a little. If the craft vibrates after a maneuver, decrease it.
* **I** (*integral*) parameter controls the smoothing. It prevents rapid changes in thrust. If engines's trust jitters, increase it.

####Named Configurations

###TCA.glob :: the global TCA settings

Most of the user-related settings are available through the TCA GUI. There are, however, tons of internal parameters that are located in the _GameData/ThrottleControlledAvionics/Plugins/PluginData/ThrottleControlledAvionics/**TCA.glob**_ file. Generally, it is not recommended to tamper with this file, but several of the settings there may be of interest to some users.

It is a plane text file which may be edited using any text editor. After saving, if the game is already running and a TCA-enabled vessel is in flight, you may apply the changes by going to the "Advanced" pane in the main window and pressing the "Reload TCA Settings" button.