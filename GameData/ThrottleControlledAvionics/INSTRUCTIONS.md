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

##Engine Modes and Groups

In editor or in flight, through the part menu or engines' *Profiles* (discussed later) you may set any engine to work in one of the following modes: 

* **Main Engine** (_default_): when balancing, TCA tries to maximize the thrust of these engines. In a perfectly balanced ship all Main Engines should have 100% thrust in the absence of control input. *These engines are used to control vertical speed AND attitude.*
* **Balanced Thrust**: a group of engines in this mode is always balanced so that it does not generate any torque. It is mostly useful for jets-based VTOLs. *These engines are used to control vertical speed.*
* **Maneuver Engine**: when balancing, TCA tries to minimize the thrust of these engines. In a perfectly balanced ship these engines produce thrust only in response to control input. *These engines are used to control attitude and for translation.*
* **Manual Control**: balancing does not change the thrust of these engines, but includes them in calculations. HSC, however, may use them to change horizontal speed of the vessel.

Each engine has also a *group ID*. By default it is set to zero, meaning the engine is not included into any group. But *all the engines sharing the same non-zero group ID will also share the mode and*, if they are Manual engines, *the value of thrust limiters*
. This is a mere convenience to setup engines quicker during construction and to have shorter Profiles.

##Engines Profiles

An *Engines Profile* is a sets of configurations of all the engines a ship has.
Engine configuration includes: 

* *On/Off* state: should the engine be active when its profile is?
* Engine's *mode*
* For a *Manual* engine: its *thrust limiter*

A *group* of engines has a single configuration.

An engine in a profile is identified by its name, position in a ship and thrust characteristics; so, in principle, a profile may be transferred to another ship of a similar design via *Named Configuration* (see *Appendix*).

Only one profile can be active at any moment, and there is always an active profile. A profile may be designated as *Default*. A default profile cannot be deleted and is automatically switched to when no other profile is available in a given situation.

In-flight a profile may activated manually; or automatically on one of the following conditions: *on activation of a given stage*; or *on going into/out-of orbit*. The latter is defined by the current trajectory: if the ship is in orbit or on an escape trajectory, or not.

Another option available for a profile is leveling of a ship on activation (the *Level *switch). If enabled, this option automatically enables the *Level* program of the *HSC*, causing the ship to point its thrust vector downwards. This is useful when you switch between, say, linear-rocket profile and a VTOL profile while deorbiting.

##Interface Basics

TCA graphical interface consists of the four separate windows:

* In-Flight main window that controls all the aspects of TCA functionality
* In-Editor configuration window that allows editing of engines profiles and pre-launch TCA state
* Macro Editor window that is available both in flight and in editor
* Manual wondow (in which you're probably reading this), that is *always* available.

The Main, In-Editor and Manual windows are summoned by TCA toolbar button. Both Blizzy's Toolbar and the stock AppLauncher are supported; by default the Toolbar is used when it is available, but this may be changed in TCA.glob file (see *Appendix*).

###Tooltips

Most of TCA control elements -- buttons, input fields, sliders -- have tooltips that are displayed on mouse hover. **Pay attention to them**, as they explain verbosely the meaning of short names of these controls.

###Button switches

Many of the TCA functions are controlled through *switches*. These are buttons that are <color=lime>green</color> when the function is enabled, <color=yellow>yellow</color> when it is disabled, and gray when it is unavailable.

###Mutually exclusive functions

Many of the TCA functions are mutually exclusive from the user's point of view. This means that often when **you** enable one button switch others that were active are automatically switched off.

Autopilot programs, on the other hand, use underlying basic functionality through a different interface; that's why when **you** enable, say, *Land* switch, the *Stop* switch also becomes enabled, but if **you** enable the *Stop* switch by hand, all other programs stop and corresponding switches become inactive.

###TCA activation and status

To enable/disable TCA, use the **Enabled** switch at the top-left corner of the main window, **or** the hot-key (Y by default). The hot-key may be changed in the *Advanced* section (see *Appendix*).

In flight the icon of the TCA toolbar button changes according to TCA state: "Disabled", "Enabled", "No Electric Charge". A more descriptive status is displayed in the top-right corner of the main window.

###In-Editor Configuration Window

In Editor, TCA Configuration window may be summoned by the toolbar button **if** a ship has at least one TCA-capable command module **and** at least one engine. Otherwise the Manual is summoned by that button. On the right corner of the titlebar of the Configuration window there is a [**?**] button that also summons the Manual.

On top of the Configuration window there's a big *Edit Macros* button that summons the Macro Editor (see **Macros** section).

Below it there's a set of switches that correspond to several TCA functions in-flight, which allows to define the initial TCA state for current ship construct.

Next there's a big field of the *Engines Profile Editor*, that allows you to create, delete and edit ship's Engines Profiles.

###In-Flight Main Window

The Main window consists of several rows of controls. Generally, each row represents one of the TCA subsystems, which are discussed in the corresponding sections. The first row, however, has just three elements:

1. *Enabled* switch that enables/disables the TCA itself.
2. *Squadron Mode* switch that toggles the squadron mode (see below).
    * If it is enabled, the *squad ID chooser* is shown next to this switch.
3. *TCA Status Indicator* that shows current TCA status.

And above the rows there's a titlebar which also has tow buttons: the **advanced** switch that toggles *Advanced* section (see *Appendix*) and the [**?**] button that summons the **Manual**.

##Attitude Control (T-SAS)

The controls of the T-SAS subsystem are located in the second row of the Main window:

* *State Indicator* that is white when T-SAS is inactive and <color=cyan>cyan</color> when it's active. It is followed by the *mode switches*:
    * **Kill** causes T-SAS to automatically kill any angular velocity the ship has. It is different from the stock SAS in that it does not try to hold any particular attitude; it just kills any rotation. But it does this much more quickly and efficiently, without oscillations.
    * **Hold** is analogous to the stock SAS.
    * **Maneuver** points the thrust vector along the maneuver vector, thus allowing to perform maneuvers with engines that are not aligned with the command module.
    * **PG** and **RG** -- prograde and retrograde respectively.
    * **R+** and **R-** -- along/against the radius-vector of the current curvature of the ship's trajectory.
    * **N+** and **N-** -- along/against the orbit normal vector.
    * **T+** and **T-** -- to/from the current target.
    * **rV+** and **rV-** -- along/against the velocity relative to the target.
* *Auto* indicator/switch that is green when some other subsystem uses T-SAS to point the thrust in some custom direction, and is grayed out otherwise.
* *Error* indicator that displays the angular error (in degrees) between the desired and actual attitudes. When T-SAS is inactive, this indicator shows "N/A".

##Vertical Speed and Altitude Control (VCS)

If you're using TCA to control VTOL or during vertical landing of a rocket, it is more convenient to control the vertical speed or altitude of a ship 'directly', rather than the Main Throttle. The VSC allows you (or the autopilot program) to do just that. Its controls are located in the third row of the Main window.

###Vertical Speed mode controls

This is the default mode of the VSC which has the following controls:

* *Vertical Speed* indicator that shows current vertical speed.
* **Set Point** slider that shows and allows to set the desired vertical speed. The limits of the set-point are defined in *TCA.glob::VSC* (see *Appendix*). *If the slider is set to its **maximum***, the VSC is <color=red>switched off</color> and the engines are controlled by Main Throttle only.

###Altitude mode controls

The mode itself is toggled with the *Hover* switch located in the row below. When it is enabled, it provides the following controls:

* *Altitude* indicator that shows *current altitude +/- current vertical speed*. In the *Follow Terrain* mode both values are relative to the surface below and may change very rapidly if you're flying above a ragged terrain.
* **Set Point** control that consists of several elements:
    1. *Desired Altitude* filed that shows and allows to change the value of the desired altitude directly typing it in from keyboard. If the value *is set* and is <color=lime>green</color>, the altitude is above the ground; if it's <color=red>red</color> it is below.
    2. **Set** button that applies the value entered in the *Desired Altitude* field.
    3. **-10m** and **+10m** buttons subtract or add ten meters to the currently set desired altitude.

###Follow Terrain mode

This option is controlled by a check-box to the right of the *Hover* switch.

When it is enabled, it changes the meaning of altitude and vertical speed within TCA: all systems start using *height from the ground* and *vertical speed of the **ground** relative to the ship* rather than altitude relative to the 'sea level' and vertical speed relative to the planet's center.

It also enables the *Radar* that scans the relief on course. The, to avoid collisions with hillside, mountains or buildings, TCA temporarily either changes the course, or slows the ship down and increases the altitude to fly above the obstacle. This behavior is also indicated by corresponding status messages: "Ascending" and "Obstacle On Course".

###AutoThrottle

This option is controlled by a check-box at the end of the VSC controls row of the Main window.

When it is enabled, the Main Throttle is locked at 100% and the keyboard throttle controls (default: *Shift*/*Ctrl*, *Z* and *X*) are used to set desired vertical speed or altitude instead. If VSC itself is off (i.e. the *Hover* is off and the desired vertical speed is set to maximum) and *AutoThrottle* is enabled, it automatically enables VSC and sets the desired vertical speed to 0.

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

Its contents is divided into sections dedicated to TCA subsystems. To reference those in this Manual I use the special notation: **TCA.glob::SectionName**.