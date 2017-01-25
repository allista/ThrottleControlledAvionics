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
using AT_Utils;

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
			[Persistent] public float MaxApproachDistance  = 10000f;   //m
		}
		static Config MVA { get { return Globals.Instance.MVA; } }
		public MatchVelocityAutopilot(ModuleTCA tca) : base(tca) {}

		ThrottleControl THR;

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
			IsActive &= 
				!VSL.LandedOrSplashed && VSL.orbit != null && 
				CFG.Target != null && CFG.Target.GetOrbit() != null && 
				CFG.AP1.Any(Autopilot1.MatchVel, Autopilot1.MatchVelNear);
			var tVSL = VSL.TargetVessel;
			ControlsActive &= IsActive || VSL.InOrbit && tVSL != null && !tVSL.LandedOrSplashed && tVSL.mainBody == VSL.Body;
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
				stage = Stage.Start;
				SetTarget(VSL.TargetAsWP);
				CFG.AT.On(Attitude.KillRotation);
				break;

			case Multiplexer.Command.Off:
				CFG.AT.On(Attitude.KillRotation);
				SetTarget();
				reset(); 
				break;
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
			stage = Stage.Start;
			Working = false;
		}

		public static float BrakingDistance(float V0, float thrust, float mflow, float throttle, VesselWrapper VSL, out float ttb)
		{
			ttb = VSL.Engines.TTB(V0, thrust, mflow, throttle);
			return V0*ttb + 
				thrust/mflow * 
				((ttb-VSL.Physics.M/mflow/throttle) * 
				 Mathf.Log((VSL.Physics.M-mflow*throttle*ttb)/VSL.Physics.M) - ttb);
		}

		public static float BrakingDistance(float V0, VesselWrapper VSL, out float ttb)
		{ 
			return BrakingDistance(V0, VSL.Engines.MaxThrustM, VSL.Engines.MaxMassFlow, 
			                       ThrottleControl.NextThrottle((float)V0, 1, VSL), 
			                       VSL, out ttb);
		}

		public static float BrakingOffset(float V0, VesselWrapper VSL, out float ttb)
		{ return BrakingDistance(V0, VSL, out ttb)/V0; }

		public static float BrakingOffset(float V0, VesselWrapper VSL)
		{ float ttb; return BrakingDistance(V0, VSL, out ttb)/V0; }

		public static float BrakingNodeCorrection(float V0, VesselWrapper VSL)
		{ 
			float ttb;
			var offset = BrakingOffset(V0, VSL, out ttb);
			return  offset - ttb/2;
		}

		bool StartCondition(float dV)
		{
			if(Working) return true;
			if(TTA > 0)
			{
				VSL.Info.Countdown = TTA-BrakingOffset(dV, VSL, out VSL.Info.TTB);
				if(VSL.Info.Countdown > 0)
				{
					if(VSL.Controls.CanWarp) 
						VSL.Controls.WarpToTime = VSL.Physics.UT+VSL.Info.Countdown-VSL.Controls.MinAlignmentTime;
				 	return false;
				}
			}
			VSL.Info.Countdown = 0;
			Working = true;
			return true;
		}

		public enum Stage { Start, Brake, Wait }
		[Persistent] public Stage stage;

		protected override void Update()
		{
			if(!IsActive) return;
			Vector3 dV;
			if(CFG.AP1[Autopilot1.MatchVel])
			{
				dV = CFG.Target.GetObtVelocity()-VSL.vessel.obt_velocity;
				VSL.Engines.RequestClusterActivationForManeuver(dV);
				Executor.Execute(dV, GLB.THR.MinDeltaV);
			}
			else
			{
				double ApprUT;
				var tOrb = CFG.Target.GetOrbit();
				var dist = TrajectoryCalculator.ClosestApproach(VSL.orbit, tOrb, VSL.Physics.UT, out ApprUT);
				TTA = (float)(ApprUT-VSL.Physics.UT);
				switch(stage)
				{
				case Stage.Start:
					if(dist > MVA.MaxApproachDistance)
					{
						Status(string.Format("<color=yellow>WARNING:</color> Nearest approach distance is <color=magenta><b>{0}</b></color>\n" +
						                     "<color=red><b>Push to proceed. At your own risk.</b></color>", 
						                     Utils.formatBigValue((float)dist, "m")));
						stage = Stage.Wait;
						goto case Stage.Wait;
					}
					stage = Stage.Brake;
					goto case Stage.Brake;
				case Stage.Wait:
					if(!string.IsNullOrEmpty(TCAGui.StatusMessage)) break;
					stage = Stage.Brake;
					goto case Stage.Brake;
				case Stage.Brake:
					dV = (tOrb.getOrbitalVelocityAtUT(ApprUT)-VSL.orbit.getOrbitalVelocityAtUT(ApprUT)).xzy;
					VSL.Engines.RequestClusterActivationForManeuver(dV);
					if(Executor.Execute(dV, GLB.THR.MinDeltaV, StartCondition)) break;
					reset();
					break;
				}
			}
		}

		public override void Draw()
		{
			if(ControlsActive)
			{
				if(Utils.ButtonSwitch("Match Velocity", CFG.AP1[Autopilot1.MatchVel], 
				                      "Continuously match orbital velocity with the target", GUILayout.ExpandWidth(true)))
					CFG.AP1.XToggle(Autopilot1.MatchVel);
				if(Utils.ButtonSwitch("Brake Near Target", CFG.AP1[Autopilot1.MatchVelNear], 
				                      "Match orbital velocity with the target at closest approach", GUILayout.ExpandWidth(true)))
					CFG.AP1.XToggle(Autopilot1.MatchVelNear);
			}
			else
			{
				GUILayout.Label(new GUIContent("Match Velocity", "Continuously match orbital velocity with the target"), 
				                Styles.inactive_button, GUILayout.ExpandWidth(true));
				GUILayout.Label(new GUIContent("Brake Near Target", "Match orbital velocity with the target at closest approach"), 
				                Styles.inactive_button, GUILayout.ExpandWidth(true));
			}
		}
	}
}

