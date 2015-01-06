#Throttle Controlled Avionics

Current devellopers : qfeys, Zenka, allista

If interested, contact us on the kerbal forums either personally or in the [TCA thread](http://forum.kerbalspaceprogram.com/threads/67270)

***

##Introduction

TCA is a plugin that enhances the attitude control of a ship by dynamically changing the thrust of its engines:

* When ship's attitude controls are used (by a user or via SAS) the thrust of each engine is modified to produce the needed torque.

* On the other hand, when controls are idle, TCA tries to compensate any torque produced by the engines to maintain the stable attitude.

* And optional vertical speed limiter enables precise control over ship's altitude.

##Features

* Support for multiple control modules (like cockpits, pods and probe-cores).
* Automatic engine selection in every situation (TCA will not fire your main thrusters while you're trying to land on top of the VAB using side-mounted VTOL-engines).
* Combination of TCA attitude control with the SAS delivers unprecedented in-flight stability to most unbalanced ship designs.
* Built-in altitude and hovering control.
* Comprehensive algorithm tuning and per-vessel profiles.
* Both the Toolbar and the stock AppLauncher are supported.

##Drawbacks and Known Issues

* For obvious reasons TCA cannot handle engines with the slow changing thrust (like jets or turbofans).
* Nor can it handle solid fuel thrusters, as they don't support thrust tweaking.
* TCA works great with the stock SAS, but combining it with the MechJeb's Smart SAS results in ever-increasing oscillations, as MechJeb uses different approach to kill rotation than SAS. **But** we're working on it.
* TCA does not **yet** support InfernalRobotics and its vectoring designs.

##Future Plans:

* Zero vector: let TCA figure out how to break your movement compared to the selected object.
* Engine powered RCS. Allow for translation when using RCS controls using your (hopefully sideward) engines
* Bindable keys
* Explanatory video