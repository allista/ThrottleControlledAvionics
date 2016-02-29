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
			new public const string NODE_NAME = "MVA";

			[Persistent] public int OrbitSolverIterations  = 20;   //deg
			[Persistent] public int OrbitSolverMinTTA      = 120;  //sec
			public float OrbitSolverDPeriod;

			[Persistent] public float TimeBeforeApproach   = 5f;   //sec
			[Persistent] public float StartAttitudeError   = 0.5f; //deg
			[Persistent] public float TranslationThreshold = 5f;   //m/s

			public override void Init()
			{ OrbitSolverDPeriod = 2f/OrbitSolverIterations; }
		}
		static Config MVA { get { return TCAScenario.Globals.MVA; } }
		public MatchVelocityAutopilot(ModuleTCA tca) : base(tca) {}

		ThrottleControl THR;
		TimeWarpControl WRP;
		AttitudeControl ATC;
		TranslationControl TRA;
		ManeuverAutopilot MAN;

		bool MainThrust;
		float TTA = -1;

		public override void Init()
		{
			base.Init();
			CFG.AP.AddCallback(MatchVelCallback, Autopilot.MatchVel, Autopilot.MatchVelNear);
		}

		protected override void UpdateState()
		{ 
			IsActive = CFG.Enabled && VSL.InOrbit && VSL.orbit != null && 
				VSL.Engines.MaxThrustM > 0 && VSL.HasTarget && VSL.Target.GetOrbit() != null &&
				CFG.AP.Any(Autopilot.MatchVel, Autopilot.MatchVelNear);
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
				MainThrust = false;
				THR.Throttle = 0;
				CFG.AT.On(Attitude.KillRotation);
				break;

			case Multiplexer.Command.Off:
				reset(); break;
			}
		}

		void reset()
		{
			if(Working) 
			{
				THR.Throttle = 0;
				CFG.AT.On(Attitude.KillRotation);
			}
			CFG.AP.OffIfOn(Autopilot.MatchVel);
			CFG.AP.OffIfOn(Autopilot.MatchVelNear);
			MainThrust = false;
			Working = false;
		}

		protected override void Update()
		{
			if(!IsActive) return;
			var dV  = VSL.vessel.obt_velocity-VSL.Target.GetObtVelocity();
			var dVm = (float)dV.magnitude;
			//if we're waiting for the nearest approach
			THR.Throttle = 0;
			if(!Working && CFG.AP[Autopilot.MatchVelNear])
			{
				//calculate time to nearest approach
				var tOrb   = VSL.Target.GetOrbit();
				var ApprUT = VSL.Physics.UT;
				int iters  = 0;
				var min_dist = Orbit.SolveClosestApproach(VSL.orbit, tOrb, ref ApprUT, 
				                           VSL.orbit.period*MVA.OrbitSolverDPeriod, 0.0, 
				                           VSL.Physics.UT, VSL.Physics.UT + VSL.orbit.period, 0.01, 
				                           MVA.OrbitSolverIterations, ref iters);
				TTA = (float)(ApprUT-VSL.Physics.UT);
				//if near enough, use local-space calculation
				var pos = VSL.Target.GetTransform().position-VSL.Physics.wCoM;
				if(TimeWarp.CurrentRateIndex == 0 && 
				   (TTA < MVA.OrbitSolverMinTTA ||
				    pos.magnitude < min_dist*2))
				{
					TTA = Vector3.Dot(pos, dV) < 0? -1 : 
						Vector3.Project(pos, dV).magnitude/dVm;
					if(dVm > MVA.TranslationThreshold)
						CFG.AT.OnIfNot(Attitude.AntiRelVel);
				}
				else //recalculate dV at nearest approach
				{
					dV = (VSL.orbit.getOrbitalVelocityAtUT(ApprUT) - tOrb.getOrbitalVelocityAtUT(ApprUT)).xzy;
					dVm = (float)dV.magnitude;
					if(dVm > MVA.TranslationThreshold)
					{
						CFG.AT.OnIfNot(Attitude.Custom);
						ATC.AddCustomRotationW(dV, VSL.Engines.MaxThrust);
					}
				}
				if(TTA > 0)
				{
					VSL.Info.TTB = MAN.TTB(dVm, 1);
					VSL.Info.Countdown = TTA-VSL.Info.TTB-MVA.TimeBeforeApproach;
					//warp to the nearest approach point if requested
					if(CFG.WarpToNode && ATC.Aligned)
						WRP.WarpToTime = VSL.Physics.UT+VSL.Info.Countdown-ATC.AttitudeError;
					if(VSL.Info.Countdown > 0) return;
				}
				Working = true;
			}
			//if dV is too small, don't do anything
			if(dVm < GLB.THR.MinDeltaV) 
			{
				if(CFG.AP[Autopilot.MatchVelNear]) reset();
				return;
			}
			//use main engines if dV is big enough, or if there's no translation capabilities
			if(MainThrust || dVm > MVA.TranslationThreshold || !VSL.Controls.TranslationAvailable)
			{
				CFG.AT.OnIfNot(Attitude.AntiRelVel);
				if(MainThrust || ATC.AttitudeError < MVA.StartAttitudeError)
				{
					THR.DeltaV = dVm;
					MainThrust = ATC.AttitudeError < GLB.ATC.AttitudeErrorThreshold;
				}
			}
			//if translation is available, use it as necessary
			if(VSL.Controls.TranslationAvailable)
			{
				if(dVm <= MVA.TranslationThreshold)
				{
					if(!MainThrust)
						CFG.AT.OnIfNot(Attitude.KillRotation);
					TRA.AddDeltaV(VSL.LocalDir(dV));
				}
			}
		}

		public override void Draw()
		{
			if(Utils.ButtonSwitch("Match Velocity", CFG.AP[Autopilot.MatchVel], 
			                      "Match orbital velocity with the target", GUILayout.ExpandWidth(true)))
				CFG.AP.XToggle(Autopilot.MatchVel);
			if(Utils.ButtonSwitch("Brake Near Target", CFG.AP[Autopilot.MatchVelNear], 
			                          "Match orbital velocity with the target at nearest point", GUILayout.ExpandWidth(true)))
				CFG.AP.XToggle(Autopilot.MatchVelNear);
		}
	}
}

