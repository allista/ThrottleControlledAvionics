#TCA Manual

##Overview

TCA is a _software_ that can be installed onto any command module, cockpit or probe core. It is, however, modularized, with modules being distributed along the TechTree. So to have TCA in _career_ mode you should first purchase it in **R&D** building, while in sandbox it is installed by default. Whether TCA is installed on a particular part is indicated in part's info pane in Editor.

When the core TCA subsystem or any of the modules is bought, all qualified parts in flight are upgraded using the OTA update. However, TCA interface and functionality only become available if a vessel has engines and/or RCS thrusters.

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

* **Thrust & Maneuver** (_default_): when balancing, TCA tries to maximize the thrust of these engines. In a perfectly balanced ship all such engines should have 100% thrust in the absence of control input. *These engines are used to control vertical speed AND attitude.*
* **Thrust**: a group of engines in this mode is always balanced so that it does not generate any torque. It is mostly useful for jets-based VTOLs. *These engines are used to control vertical speed.*
* **Maneuver**: when balancing, TCA tries to minimize the thrust of these engines. In a perfectly balanced ship these engines produce thrust only in response to control input. *These engines are used to control attitude, and for translation.*
* **UnBalanced Thrust**: this mod is primarily intended for a single-engine configuration like rocket lander or a VTOL-capable plane with a single lifting engine. TCA never tries to balance engines in this mode, but still *uses them to control altitude and vertical speed.*
* **Manual Control**: these engines are never balanced or used to control altitude or vertical velocity. Instead, their thrust may be directly controlled with a slider under "Manual Engines" pane which is toggled with "Show/Hide Manual Engines" button; or from a corresponding engines profile. *HSC, however, uses them for horizontal thrust when needed.*

Each engine has also a *group ID*. By default it is set to zero, meaning the engine is not included into any group. But *all the engines sharing the same non-zero group ID will also share the mode and*, if they are Manual engines, *the value of thrust limiters*. This is a mere convenience to setup engines quicker during construction and to have shorter Profiles.

**Note** on changing properties of a single engine in a *group* from the part menu. A group acts as a whole, so when you change some property of a grouped engine these changes are propagated to other engines in that group. This includes enabling/disabling engines: *shutting down one engine will shut down the whole group*.

##Engines Profiles

An *Engines Profile* is a sets of configurations of all the engines a ship has.
Engine configuration includes: 

* *On/Off* state: should the engine be active when its profile is?
* Engine's *mode*
* For a *Manual* engine: its *thrust limiter*

A *group* of engines has a single configuration.

An engine in a profile is identified by its name, position in a ship and thrust characteristics; so, in principle, a profile may be transferred to another ship of a similar design via *Named Configuration* (see *Appendix*).

Only one profile can be active at any moment, and *there is always an active profile*. A profile may be designated as *Default*; a default profile cannot be deleted and is automatically switched to when no other profile is available in a given situation.

In-flight a profile may be activated manually or automatically on one of the following conditions: *on activation of a given stage* or *on going into/out-of orbit*. The latter is defined by the current trajectory: if the ship is in orbit or on an escape trajectory, or not.

Another option available for a profile is leveling of a ship on activation (the *Level *switch). If enabled, this option automatically enables the *Level* program of the *HSC*, causing the ship to point its thrust vector downwards. This is useful when you switch between, say, linear-rocket profile and a VTOL profile while deorbiting.

###Staging and Action Groups

Activation of engines by staging or action groups *always* takes priority over TCA setting, so the *active* profile is always updated on a stage or active group activation. But *remember the caveat of grouped engines*.

TCA itself may also be toggled via action group. To set it up, you need to find a Control Part (probe core, cockpit, etc.) that says "TCA Active: True" in its part menu (as a rule, this should be the first one you have added to a ship). Then follow the standard procedure action group assignment.

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

In flight the icon of the TCA toolbar button changes according to TCA state: "Disabled", "Enabled", "No Electric Charge". A more descriptive status is displayed in the top-right corner of the main window:

* *Obstacle On Course*: the ship is on a collision course with something
* *Ground Collision Possible*: the same as above, but with the negative vertical velocity
* *Loosing Altitude*: vertical velocity is negative and lower than the set-point
* *Low Control Authority*: available torque is not enough to achieve desired attitude in time
* *Engines Unoptimized*: engines are unable to provide required torque, so they are set to give zero torque
* *Ascending*: the ship is automatically gaining altitude to avoid a collision with an obstacle
* *VTOL Assist On*: VTOL Assist autopilot is active at the moment
* *Stabilizing Flight*: Stabilizing Flight autopilot is active at the moment
* *Altitude Control*: Altitude Control is active and is the main autopilot at the moment
* *Vertical Speed Control*: Vertical Speed Control is active and is the main autopilot at the moment
* *Systems Nominal*: all working well
* *No Active Engines*: all engines are disabled or in a flameout; all RCS thrusters are disabled or give no thrust
* *No Electric Charge*: no electric charge is left on the ship; TCA can't function

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
    * **PG** and **RG**: prograde and retrograde respectively.
    * **R+** and **R-**: along/against the radius-vector of the current curvature of the ship's trajectory.
    * **N+** and **N-**: along/against the orbit normal vector.
    * **T+** and **T-**: to/from the current target.
    * **rV+** and **rV-**: along/against the velocity relative to the target.
* *Auto* indicator/switch that is green when some other subsystem uses T-SAS to point the thrust in some custom direction, and is grayed out otherwise.
* *Error* indicator that displays the angular error (in degrees) between the desired and actual attitudes. When T-SAS is inactive, this indicator shows "N/A".

##Vertical Speed and Altitude Control

If you're using TCA to control VTOL or during vertical landing of a rocket, it is more convenient to control the vertical speed or altitude of a ship 'directly', rather than manually change the Main Throttle. The VSC and Hover modules allow you (or autopilot programs) to do just that. Their controls are located in the third row of the Main window:

###Vertical Speed Control (VSC)

When Altitude Control is disabled or not installed:

* *Vertical Speed* indicator that shows current vertical speed.
* **Set Point** slider that shows and allows to set the desired vertical speed. The limits of the set-point are defined in *TCA.glob::VSC* (see *Appendix*). *If the slider is set to its **maximum***, the VSC is <color=red>switched off</color> and the engines are controlled by Main Throttle only.

###Altitude Control (Hover)

When Altitude Control is installed and enabled by the "Hover" switch:

* *Altitude* indicator that shows *current altitude +/- current vertical speed*. In the *Follow Terrain* mode both values are relative to the surface below and may change very rapidly if you're flying above a ragged terrain.
* **Set Point** control that consists of several elements:
    1. *Desired Altitude* filed that shows and allows to change the value of the desired altitude directly typing it in from keyboard. If the value *is set* and is <color=lime>green</color>, the altitude is above the ground; if it's <color=red>red</color> it is below.
    2. **Set** button that applies the value entered in the *Desired Altitude* field.
    3. **-10m** and **+10m** buttons subtract or add ten meters to the currently set desired altitude.

###Follow Terrain mode

This option is controlled by a switch to the right of the *Hover* switch.

When it is enabled, it changes the meaning of altitude and vertical speed within TCA: all systems start using *height from the ground* and *vertical speed of the **ground** relative to the ship* rather than altitude relative to the 'sea level' and vertical speed relative to the planet's center.

It also enables the *Radar* that scans the relief on course. Then, to avoid collisions with hillside, mountains or buildings, TCA temporarily either changes the course or slows the ship down and increases the altitude to fly above the obstacle. This behavior is also indicated by corresponding status messages: "Ascending" and "Obstacle On Course".

###AutoThrottle

This option is controlled by a switch at the end of the VSC controls row of the Main window.

When it is enabled, the Main Throttle is locked at 100% and the keyboard throttle controls (default: *Shift*/*Ctrl*, *Z* and *X*) are used to set desired vertical speed or altitude instead. If VSC itself is off (i.e. the *Hover* is off and the desired vertical speed is set to maximum) and *AutoThrottle* is enabled, it automatically enables VSC and sets the desired vertical speed to 0.

##Horizontal Speed Control (HSC)

This is a very important TCA Module that provides services to almost all more complex modules. By itself it provides two simple autopilot programs:

* **Stop**: tries to maneuver the craft so that the horizontal component of its velocity became zero. It includes flip-over prevention system, so whatever the speed, the autopilot decrease it carefully and steady.

* **Level Flight**: directs total thrust vector down to the planet's center.

##Cruise Control

This module tries to keep current horizontal speed and direction. When active, the speed may be changed either via slider below or using *pitch* controls; direction may be changed with *yaw* controls.

##On-Planet Autopilots

This set of programs is available while a vessel is on planetary body (landed or in flight).

###Anchor

This module allows a flying ship to be "pinned" to a point on the surface underneath it. When the Anchor is activated, TCA saves the location below; then constantly maneuvers the ship so that its center of mass was posed directly above the saved point.

###Land

This is a complex autopilot program that uses many other TCA modules. As you would expect, it tries to land a ship; but not right away on whatever happens to be below it. Instead, it first scans the ground underneath, trying to find a flat surface big enough for the ship, then (if found) maneuvers towards it and carefully lands, avoiding lateral movements and flipping-over. It also handles brakes and landing gear and turns the engines off on the ground.

###Waypoint Navigation

This module allows TCA to automatically fly along a path of the user-defined waypoints or other landed ships. At any waypoint you have three options to choose from: fly by it (default), pause the game or land near it (using the Land autopilot).

You can add waypoints directly in flight or in the Map View using mouse: first, press the "Add Waypoint" button in TCA Main Window; then left-click anywhere on the ground to create a waypoint; left-click again to create another; to finish the process click the right mouse button. Note, that rotation with the right mouse button pressed works normally in this mode.

If you try to add a waypoint while another ship is targeted, it is saved as a waypoint. If it then moves, the waypoint will move with it. But if it is destroyed or recovered, the waypoint will remember its last seen position and become stationary.

Waypoints are displayed (on the ground and in Map View) when the "Show Waypoints" button is pressed. Their color indicates their order: from <color=red>red</color>, the nearest, through <color=yellow>yellow</color> and <color=green>green</color> to <color=cyan>cyan</color>, the furthest.

###GoTo/Follow Target

In GoTo mode TCA will fly the ship to the currently selected target and then activate the Stop program upon arrival. In Follow mode the ship will constantly follow the target vessel.

Simple. But what if you're trying to make several ships following the same target? In that case, to avoid collisions, TCA instances of the followers will negotiate and create a stable wedge formation with the target on its tip.

##In-Orbit Autopilots

###Warp switch

Enables automatic time-warping to the start of the burn, taking into account attitude error and burn duration.

###Execute Node

Does as it says; except it uses the T-SAS to control the attitude, so it's easily possible to perform orbital maneuvering with an unbalanced VTOL whose cockpit is rotated 90 degrees with respect to engines. Or to change orbit of a whole compound space station with engines pointing in different directions. I mean, controllably and predictably change orbit. Just assign engines' Roles properly.

###Match V

Constantly corrects ships orbital velocity to match that of the target object, using main thrusters as well as RCS (if available). Don't try it from far away, though, as in that case it will considerable modify your current orbit.

###Brake Near

First waits for the nearest approach point with the target, then matches orbital velocities. No continuous orbit correction is made afterwards.

###ToOrbit

This autopilot tries to achieve a circular orbit with user-defined radius and inclination. It uses a two-step approach, where it first gets into a suborbital trajectory with low apoapsis; at that apoapsis it accelerates to an orbit with the apoapsis equal to the final orbit radius; and finally it performs a circularization maneuver.

When you enable this autopilot, a small configuration window appears allowing you to define desired orbital parameters.

* **Radius** is set in kilometers above the "sea level"; if a planet has atmosphere it is forced to be 1km above it, otherwise it is force to be grater than 10km.
* **Inclination** is set in degrees and is physically limited to range from *|vessel latitude|* to *180 - |vessel latitude|*.
* **Ascending/Descending Node** switch defines the type of the orbit node located above the starting point. I.e. AN orbits start to the North of the equator, and DN start to the South.
* **Prograde/Retrograde Orbit** switch defines orbit direction: a vessel on a PG orbit rotates in the same direction as the central body, and on a RG orbit in the opposite. In other words RG orbits correspond to negative inclinations.

###Rendezvous

This is a very complex autopilot whose final goal is to bring your ship close to its target. You may start from ground, from suborbital trajectory or from orbit; the only limitation is that the target should be in orbit around the same central body.

To achieve its goal the Rendezvous autopilot performs a series of maneuvers divided into several stages:

* **To Orbit** stage works almost like ToOrbit autopilot, except if you start from the ground it will try to predict the duration of ascension and will wait for the appropriate launch window. It also calculates the inclination needed to minimize dV of the consequent maneuvers.
* **Start Orbit** stage corrects current orbit to achieve a faster resonance with the target, so you don't have to wait a week for the rendezvous maneuver window.
* **Rendezvous Maneuver** stage computes and performs the main maneuver that will eventually bring your ship close to the target.
* **Fine-tune Approach** stage is performed repeatedly after the main rendezvous maneuver until the closest approach distance is below the threshold (default *100m + ship's radius + target's radius*; configurable in *TCA.glob::REN*).
* **Match Orbits** stage schedules and performs a brake maneuver that synchronizes ship's orbit with the target's orbit at nearest approach.
* **Approach** stage brings the ship in close proximity to the target after orbits were synchronized.

###Land

Unlike the Land autopilot in On-Planet group, this autopilot is dedicated to landing from orbit. It is also a complex autopilot that in fact uses the On-Planet Land program as its final stage. The whole sequence of operation is as follows:

* **Deorbit Burn** stage calculates, schedules and performs a deorbiting maneuver.
* **Correct Trajectory** stage repeatedly corrects trajectory to bring the landing site close to the target.
* **Deceleration** stage performs a brake maneuver that kills most of the velocity and at the same time fine-tunes the landing site's position.
* **Approach** stage is just the *Go To* navigation program.
* **Land** stage is the *On-Planet Land* program.
* **Hard Land** *(optional)*: if something goes wrong, e.g. the ship doesn't have enough fuel to perform the Approach&Land scenario, or its TWR is too low, or it lacks the control authority in thick atmosphere. In that case this stage kicks in after Deceleration and tries it's best to land the ship instead of crushing it. It uses all means available, including parachutes, airbrakes, braking with ship's own drag (by placing its most wide side against the airstream), and final suicidal burn.

###Jump To *(aka Ballistic Jump or Suborbital Hop)*

This autopilot uses a calculated suborbital trajectory to get to a far enough target on the surface of the same planet. Think of ballistic missiles here. Its operations divided into several stages as well:

* **Liftoff** stage is used to ensure the initial acceleration to the suborbital trajectory will not meed a quick end in the nearest mountainside.
* **Acceleration** stage calculates and performs a maneuver to achieve the desired trajectory.
* **Correct Trajectory** stage repeatedly corrects trajectory to fine-tune the position of a landing site.
* **Land** stage encloses the Deceleration, Approach, Land/Hard Land stages of the Land autopilot described above.

##Utility Modules

###Radar

Scans the surface before and underneath the ship to detect obstacles before the ship's nose crash into them. By itself does not do anything, but is used by HSC and Altitude Control to avoid collisions.

###Collision Prevention System (CPS)

Scans nearby vessels, measuring distance and relative speed. The predicts possible collisions and calculates an avoidance maneuver which is fed to the HSC and VSC to actuate.

###VTOL Mode of manual control

This module changes the way manual controls work. With it enabled a VTOL craft starts to behave like a conventional helicopter: 

* **pitch-roll-yaw** controls define the angle of the total thrust vector to the vertical; this angle itself is limited to the non-suicidal values that depend on current TWR and thrust response time. Idle controls in that mode are equivalent to the *Level* program.
* **reference part** used in this mode may differ from the current control part (which is set by "Control from here" button in the part menu) and is picked by TCA automatically. This is allows to control the craft as a whole as if from a remote, not from "this particular probe core that Jeb's attached to the side with the duck-tape".

###VTOL Assist

This module helps with vertical landings and take-offs: it automatically retracts/deploys landing gear, enables-disables brakes and tries to prevent flipping over when dealing with inclined surfaces. It may be toggled in editor during ship construction or in the *Advanced* section.

###Flight Stabilizer

This is a safeguard module that acts when nothing else does: it catches the ship in uncontrollable rotation in absence of SAS, user input or other TCA autopiloting; when it does, it first enables the Level program, and when it's safe, the stock SAS. It may be toggled in editor during ship construction or in the *Advanced* section.

##Squadron Mode

This mode is enabled by a switch in the first row of the main window. When enabled, a squadron ID selector appears, allowing you to change the designation of the current ship.

All ships assigned to the same squadron (with TCA enabled) will respond simultaneously on most of the commands you execute through the TCA window or keyboard. The exception being the Macros. When a command is executed in a squadron the on-screen message is displayed.

An additional command-button appears in Squadron mode on the Navigation panel: the "Follow Me" button. When pressed, makes all other ships in the squadron to select the current ship as their target and execute the "Follow" program.

##Macros

Macros are simple user-defined programs consisting of built-in TCA actions, some stock actions and other macros that are already defined. Note, that when another macro is referenced, it is actually copied to the current one, so no recursion is possible.

Other elements necessary to make a program are also present: IF-THEN-ELSE statements, conditional expressions with boolean operators, FOR and WHILE loops.

All of this is wrapped into a mostly-mouse-driven graphical interface, so no programming language knowledge is required (for that you have kOS).

###Macro Editor

This is a separate window that may be summoned in-flight or in editor. It contains controls to create/edit TCA macros.

The first row of controls allows you to:

* **Load**: load a previously defined macro.
* **New**: create a new macro from scratch.
* **Apply**: if you editing the current macro in flight, this button will apply the changes in place.
* **Save to Vessel DB**: save the macro to ship's computer.
* **Save to Global DB**: save the macro globally; Global macros are accessible from other ships and from other savegames. They are stored in the *GameData/ThrottleControlledAvionics/TCA.macro* file, so you can share it with others.

Then follows the editable name of the macro and its body. The body of a macro consists of nested blocks of actions, the outermost block being the macro itself and each inner block being an action.

An action entry in a block looks as follows:

[<color=red>X</color>] Action Button [^][<][>]

* **[<color=red>X</color>]** button deletes the action.
* **Action Button** displays the name and parameters of the action. If pressed, it enables editing mode to change action parameters, add sub actions and so on.
* **[^]** button moves the action one position upward in a block.
* **[<]** button moves the action out of the current block *to its parent* block, *if possible*.
* **[>]** button moves the action *into the preceding* block, *if possible*.

Below is the "Add Action" button which opens the pane with the choice of actions to add, including:

* **Builtin**: elementary actions, including flow control and loops.
* **Current Vessel**: macros saved in the ship's computer.
* **Global Database** macros saved globally.

And if an action is conditional, in edit mode you can push the "<color=green>**+**</color>" button to open the "Condition List" pane.

###In-Flight Macro Controls

In flight the main TCA window has the following buttons to manage and execute macros:
    
* **Select Macro/Current Macro**: load a macro from local or global database.
* **Execute/Pause**: start the selected macro or pause the execution of an already running macro.
* **Stop**: stop execution and rewind the macro to the start.
* **Edit/New**: open the Macro Editor. If a macro was selected, it is loaded to be edited; otherwise a new empty macro is created.

##General Notes

1. For safety reasons the VSC, HSC and On-Planet autopilots are disabled in orbit, but not on suborbital trajectories, so be carefull.
2. If your ship wobbles and oscillates with TCA and SAS enabled, rebuild it with more struts, or decrease appropriate Steering Gains.
3. Solid boosters have constant thrust and thus cannot be controlled by TCA. But they are still accounted for, if present.

##Appendix

###Advanced section

Advanced, or rather rarely needed settings are hidden in a separate section of the main window that can be accessed through *advanced* switch located at the left corner of the titlebar.

####Reload TCA Settings

This button reloads **TCA.glob** file from disk.

####TCA hotkey

This button allows to change the key that toggles TCA from keyboard.

####TCA control switches

* **VTOL Mode** enables/disables VTOL Mode of manual controls
* **VTOL Assist** enables/disables VTOL Assist module
* **Flight Stabilizer** enables/disables Flight Stabilizer module
* **AutoGear** allows/forbids the automatic use of landing gear action group by TCA
* **AutoBrakes** allows/forbids the automatic use of brakes action group by TCA
* **AutoStage** allows/forbids the automatic staging by TCA
* **AutoChute** allows/forbids the automatic use parachutes by TCA

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

This section allows to save the whole TCA configuration of the current ship under an arbitrary name; such configuration then becomes available to any other vessel within this savegame and may loaded, *replacing that ship's configuration*.

###TCA.glob :: the global TCA settings

Most of the user-related settings are available through the TCA GUI. But under the hood there are tons of internal parameters that are located in the _GameData/ThrottleControlledAvionics/Plugins/PluginData/ThrottleControlledAvionics/**TCA.glob**_ file.

It is a plane text file which may be *viewed* in any text editor. Its content is divided into sections dedicated to various TCA modules. To reference those in this Manual I use the special notation: **TCA.glob::SectionName**.

It is *not recommended* to tamper with this file as it **should** be overwritten on each update. You can, however, override any of its settings in _GameData/ThrottleControlledAvionics/**TCA.user**_, which you should create yourself when you need it.

To override a global setting from TCA.glob just copy it to the TCA.user and change its value there. To override a setting of some module, copy its NODE name along with curly brackets, then copy the setting of interest on a new line _inside_ the brackets and change its value.

After saving, if the game is already running and a TCA-enabled vessel is in flight, you may apply the changes by going to the "Advanced" pane in the main window and pressing the "Reload TCA Settings" button.

###Requirements

Currently TCA requires only one other mod which is, of course, the latest **[ModuleManager](http://forum.kerbalspaceprogram.com/index.php?/topic/50533-105-module-manager-2618-january-17th-with-even-more-sha-and-less-bug/)**.