using System;
using UnityEngine;

namespace ThrottleControlledAvionics
{
	public class TCAConfig : ConfigNodeObject
	{
		new public const string NODE_NAME = "TCACONFIG";

		static readonly float MAX_STEERING = Mathf.Sqrt(3);

		[Persistent] public bool  Enabled;
		[Persistent] public float SteeringThreshold = 0.01f; //too small steering vector may cause oscilations
		[Persistent] public float VerticalCutoff    = 1f;    //max. positive vertical speed m/s (configurable)
		[Persistent] public float StabilityCurve    = 0.3f;  /* coefficient of non-linearity of efficiency response to partial steering 
												    		   (2 means response is quadratic, 1 means response is linear, 0 means no response) */
	}

	public class TCAGlobals : ConfigNodeObject
	{
		new public const string NODE_NAME = "TCAGLOBALS";

		[Persistent] public static float  MaxCutoff            = 10f;   //max. positive vertical speed m/s (configuration limit)
		[Persistent] public static float  MaxSteeringThreshold = 0.5f;  //max. positive vertical speed m/s (configuration limit)
		[Persistent] public static float  TorqueThreshold      = 0.1f;  //engines which produce less specific torque are considered to be main thrusters and excluded from TCA control
		[Persistent] public static float  IdleResponseSpeed    = 0.25f; //constant response speed when attitude controls are idle

		[Persistent] public static Rect   ControlsPos;
		[Persistent] public static Rect   HelpPos;

		[Persistent] public static string TCA_Key = "y";

		public static string Instructions = string.Empty;
		const string instructions = 
@"Welcome to TCA instructions manual.

For simple use:
	1) Put TCA on ('{0}'),
	2) Put SAS on ('t'),
	3) Launch!

Vertical Speed Limit, hovering and horizontal flight:
	* If you're using TCA to control VTOL or during vertical landing of a rocket, you may enable the Vertical Speed Limiter. 
	The limit may be set with the scroll bar in the interval from -{1:F1}m/s to {1:F1}m/s (not including). When the Limit is set, the total thrust of all controllable engines is modified in an attempt to reach the specified vertical speed. The speed limit itself is never achived, however, but approached asymptotically, so you need to set it a little higher (~0.1-0.5m/s) than desired.
	To completely disable the Speed Limit, just set it to maximum value ({1}m/s).
	* Another use of the Vertical Speed Limit is a stable horizontal flight. Consider a VTOL that has lifted off, reached some altitude and started to pitch to get some forward momentum. If the thrust of its engines will remain constant, it will start to loose altitude as it moves forward. But with the automatic speed limiting the thrust will be adjusted, and the VTOL will move more or less in a plane.

Notes:
	* Thrust of jets and turbofan engines changes very slowly. This makes using them as attitude controllers impractical. Don't use them with TCA. 
	* Solid boosters have constant thrust and thus cannot be controlled by TCA.";

		public override void Load(ConfigNode node)
		{
			base.Load(node);
			Instructions = string.Format(instructions, TCA_Key, MaxCutoff);
		}
	}

	class TCAConfiguration
	{
		ConfigNode root;

	}
}

