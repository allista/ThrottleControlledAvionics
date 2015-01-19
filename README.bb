[CENTER][B][SIZE=5]Throttle Controlled Avionics - Continued[/SIZE][/B]
[SIZE=4][B]real-time engine balancing, attitude and altitude control[/B][/SIZE]

[B]Originally created[/B] by [URL="http://forum.kerbalspaceprogram.com/members/45099-qfeys"][B]qfeys[/B][/URL] and [URL="http://forum.kerbalspaceprogram.com/members/108134-Zenka"][B]Zenka[/B][/URL]
Current development and maintenance: [URL="http://forum.kerbalspaceprogram.com/members/102693-allista"]allista[/URL][/CENTER]

[HR][/HR]

TCA is a plugin that enhances the attitude control of a ship by dynamically changing the thrust of its engines. Even with most unbalanced designs TCA maintains stable flight and quick response to user or autopilot input. And optional vertical speed limiter enables precise control over ship's altitude.

[CENTER][video=youtube_share;HymGjhyIvIs]http://youtu.be/HymGjhyIvIs[/video][/CENTER]

[B][SIZE=4]Features[/SIZE][/B]

[list]
[*] [B]Per-vessel configuration[/B] profiles and [B]custom presets[/B].
[*] Support for [B]multiple[/B] control modules ([B]cockpits[/B], pods, probe-cores, etc.)
[*] Sophisticated [B]parameter tuning[/B], both automatic and manual.
[*] Built-in [B]altitude and hovering control[/B].
[*] Both the [B]Toolbar and[/B] the stock [B]AppLauncher[/B] are supported.[/list]
[B][SIZE=4]Downloads and Installation[/SIZE][/B]

If you're upgrading, [B]DELETE[/B] the old version before installing a new one
[B]BUT[/B] do not delete the TCA.conf file to preserve your settings.

[list]
[*] [URL="https://github.com/qfeys/ThrottleControlledAvionics/blob/master/ChanegLog.md"][B]ChangeLog[/B][/URL]
[*] [URL="https://kerbalstuff.com/mod/"]KerbalStuff[/URL]
[*] [URL="https://github.com/qfeys/ThrottleControlledAvionics/releases"]GitHub[/URL]
[*] [URL="https://github.com/qfeys/ThrottleControlledAvionics"]Source code[/URL][/list]
[B][SIZE=4]Instructions[/SIZE][/B]

[B]For simple use:[/B]
[list=1]
[*] Turn TCA on (default 'y'),
[*] Turn SAS on (default 't'),
[*] Launch![/list]
[B]Autotuning Parameters:[/B]
If this option is enabled, TCA calculates Steering Gains and PI coefficients as functions of maximum possible angular acceleration along each of the principal axes of a craft. Note, that calculations are based on the predefined response curves tuned for the stock SAS.
If you have already tuned these parameters for the ship, save its configuration before enabling, as autotuning will overwrite previous parameters.

[B]Steering Gains:[/B]
Master Gain modifies the magnitude of all steering input after other modifications. If the craft reacts slowly, increase it a little. If the craft vibrates after a maneuver, decrease it.
Pitch, Yaw and Roll modify only corresponding inputs. Linking Pitch and Yaw useful if the ship's engines are symmetrical along main axis. In that case Pitch&Yaw slider may be used in stead of Master Gain.

[B]Engines PI-controller tuning:[/B]
[list]
[*] P (proportional) parameter controls the response speed of the engines. If the craft reacts slowly, increase it a little. If the craft vibrates after a maneuver, decrease it.
[*] I (integral) parameter controls the smoothing. It prevents rapid changes in thrust. If engines's trust jitters, increase it.[/list]
[B]Vertical Speed Limit, hovering and horizontal flight:[/B]
If you're using TCA to control VTOL or during vertical landing of a rocket, you may enable the Vertical Speed Limiter. 
The limit may be set with the scroll bar in the (configurable) interval from -10m/s to 10m/s (not including). When the Limit is set, the total thrust of all controllable engines is modified in an attempt to reach the specified vertical speed. The speed limit itself is never achieved, however, but approached asymptotically, so you need to set it a little higher (0.1-0.5m/s) than desired.
To completely disable the Speed Limit, just set it to maximum value (default 10m/s).
Another use of the Vertical Speed Limit is a stable horizontal flight. Consider a VTOL that has lifted off, reached some altitude and started to pitch to get some forward momentum. If the thrust of its engines will remain constant, it will start to loose altitude as it moves forward. But with the automatic speed limiting the thrust will be adjusted, and the VTOL will move more or less in a plane.

[B]Notes:[/B]
[list]
[*] If your ship wobbles and oscillates with TCA and SAS enabled, rebuild it with more struts, or decrease appropriate Steering Gains.
[*] Thrust of jets and turbofan engines changes very slowly. This makes using them as attitude controllers impractical. Don't use them with TCA. 
[*] Solid boosters have constant thrust and thus cannot be controlled by TCA.[/list]
[B][SIZE=4]Future Plans:[/SIZE][/B]
[list]
[*] Automatically match speed with target.
[*] Translations with non RCS-enabled engines.[/list]
[B][SIZE=4]Acknowledgments:[/SIZE][/B]
First of all, many thanks to [B]qfeys[/B] and [B]Zenka[/B] for creating the original concept and implementation. Without them TCA would not exist.
And here are the mods whose code and ideas were used in one way or another:
[list]
[*] [URL="http://forum.kerbalspaceprogram.com/threads/88933"]Hangar[/URL]
[*] [URL="http://forum.kerbalspaceprogram.com/threads/12384"]MechJeb2[/URL][/list]