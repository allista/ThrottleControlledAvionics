//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

using System;
using UnityEngine;

namespace ThrottleControlledAvionics
{
	[CareerPart]
	[RequireModules(typeof(ManeuverAutopilot),
	                typeof(AttitudeControl),
	                typeof(ThrottleControl),
	                typeof(TranslationControl),
	                typeof(TimeWarpControl))]
	public class MatchVelocityAutopilot : TCAModule
	{
		public class Config : ModuleConfig
		{
			[Persistent] public float TranslationThreshold = 5f;   //m/s
		}
		static Config MVA { get { return TCAScenario.Globals.MVA; } }
		public MatchVelocityAutopilot(ModuleTCA tca) : base(tca) {}

		ThrottleControl THR;
		AttitudeControl ATC;

		Vessel Target;
		ManeuverExecutor Executor;
		float TTA = -1;

		public override void Init()
		{
			base.Init();
			Executor = new ManeuverExecutor(TCA);
			CFG.AP1.AddCallback(MatchVelCallback, Autopilot1.MatchVel, Autopilot1.MatchVelNear);
		}

		protected override void UpdateState()
		{ 
			base.UpdateState();
			IsActive &= VSL.InOrbit && VSL.orbit != null && Target != null && Target.GetOrbit() != null 
				&& VSL.Engines.MaxThrustM > 0 && CFG.AP1.Any(Autopilot1.MatchVel, Autopilot1.MatchVelNear);
			var tVSL = VSL.TargetVessel;
			ControlsActive = IsActive || tVSL != null && tVSL.situation == Vessel.Situations.ORBITING && tVSL.mainBody == VSL.mainBody;
			if(IsActive) return;
			reset();
		}

		public void MatchVelCallback(Multiplexer.Command cmd)
		{
			switch(cmd)
			{
			case Multiplexer.Command.Resume:
			case Multiplexer.Command.On:
				Working = false;
				THR.Throttle = 0;
				Target = VSL.TargetVessel;
				CFG.AT.On(Attitude.KillRotation);
				break;

			case Multiplexer.Command.Off:
				reset(); break;
			}
		}

		protected override void reset()
		{
			base.reset();
			if(Working) 
			{
				THR.Throttle = 0;
				CFG.AT.On(Attitude.KillRotation);
			}
			CFG.AP1.OffIfOn(Autopilot1.MatchVel);
			CFG.AP1.OffIfOn(Autopilot1.MatchVelNear);
			Working = false;
			Target = null;
		}

		public static double BrakingDistance(double V0, VesselWrapper VSL, out float ttb)
		{
			ttb = VSL.Engines.TTB((float)V0);
			var throttle = ThrottleControl.NextThrottle((float)V0, 1, VSL);
			return V0*ttb + 
				VSL.Engines.MaxThrustM/VSL.Engines.MaxMassFlow * 
				((ttb-VSL.Physics.M/VSL.Engines.MaxMassFlow/throttle) * 
				 Math.Log((VSL.Physics.M-VSL.Engines.MaxMassFlow*throttle*ttb)/VSL.Physics.M) - ttb);
		}

		public static double BrakingOffset(double V0, VesselWrapper VSL, out float ttb)
		{ return BrakingDistance(V0, VSL, out ttb)/V0; }

		public static double BrakingOffset(double V0, VesselWrapper VSL)
		{ float ttb; return BrakingDistance(V0, VSL, out ttb)/V0; }

		public static double BrakingNodeCorrection(double V0, VesselWrapper VSL)
		{ 
			float ttb;
			var offset = BrakingOffset(V0, VSL, out ttb);
			return  offset - ttb/2;
		}

		bool StartCondition(float dV)
		{
			if(Working) return true;
			if(!CFG.AP1[Autopilot1.MatchVelNear]) return true;
			//calculate time to nearest approach
			double ApprUT;
			var tOrb = Target.GetOrbit();
			TrajectoryCalculator.ClosestApproach(VSL.orbit, tOrb, VSL.Physics.UT, out ApprUT);
			TTA = (float)(ApprUT-VSL.Physics.UT);
			var ApprdV = (VSL.orbit.getOrbitalVelocityAtUT(ApprUT) - tOrb.getOrbitalVelocityAtUT(ApprUT)).xzy;
			dV = (float)ApprdV.magnitude;
			CFG.AT.OnIfNot(Attitude.Custom);
			ATC.SetThrustDirW(ApprdV);
			if(TTA > 0)
			{
				VSL.Info.Countdown = TTA-BrakingOffset(dV, VSL, out VSL.Info.TTB);
				if(CFG.WarpToNode && ATC.Aligned)
					VSL.Controls.WarpToTime = VSL.Physics.UT+VSL.Info.Countdown-ATC.AttitudeError;
				if(VSL.Info.Countdown > 0) return false;
			}
			VSL.Info.Countdown = 0;
			Working = true;
			return true;
		}

		protected override void Update()
		{
			if(!IsActive) return;
			var dV  = Target.GetObtVelocity()-VSL.vessel.obt_velocity;
			if(Executor.Execute(dV, GLB.THR.MinDeltaV, StartCondition)) return;
			if(CFG.AP1[Autopilot1.MatchVelNear]) reset();
		}

		public override void Draw()
		{
			if(ControlsActive)
			{
				if(Utils.ButtonSwitch("Match V", CFG.AP1[Autopilot1.MatchVel], 
				                      "Continuously match orbital velocity with the target", GUILayout.ExpandWidth(false)))
					CFG.AP1.XToggle(Autopilot1.MatchVel);
				if(Utils.ButtonSwitch("Brake Near", CFG.AP1[Autopilot1.MatchVelNear], 
				                      "Match orbital velocity with the target at closest approach", GUILayout.ExpandWidth(false)))
					CFG.AP1.XToggle(Autopilot1.MatchVelNear);
			}
			else
			{
				GUILayout.Label(new GUIContent("Match V", "Continuously match orbital velocity with the target"), 
				                Styles.inactive_button, GUILayout.ExpandWidth(false));
				GUILayout.Label(new GUIContent("Brake Near", "Match orbital velocity with the target at closest approach"), 
				                Styles.inactive_button, GUILayout.ExpandWidth(false));
			}
		}
	}
}

