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
		public MatchVelocityAutopilot(ModuleTCA tca) { TCA = tca; }

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
				VSL.MaxThrustM > 0 && VSL.HasTarget && VSL.Target.GetOrbit() != null &&
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
				TCA.THR.Throttle = 0;
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
				TCA.THR.Throttle = 0;
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
			TCA.THR.Throttle = 0;
			if(!Working && CFG.AP[Autopilot.MatchVelNear])
			{
				//calculate time to nearest approach
				var tOrb   = VSL.Target.GetOrbit();
				var ApprUT = VSL.UT;
				int iters  = 0;
				var min_dist = Orbit.SolveClosestApproach(VSL.orbit, tOrb, ref ApprUT, 
				                           VSL.orbit.period*MVA.OrbitSolverDPeriod, 0.0, 
				                           VSL.UT, VSL.UT + VSL.orbit.period, 0.01, 
				                           MVA.OrbitSolverIterations, ref iters);
				TTA = (float)(ApprUT-VSL.UT);
				//test
				//if near enough, use local-space calculation
				var pos = VSL.Target.GetTransform().position-VSL.wCoM;
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
						TCA.ATC.AddCustomRotationW(dV, VSL.MaxThrust);
					}
				}
				if(TTA > 0)
				{
					VSL.TTB = TCA.MAN.TTB(dVm, VSL.MaxThrustM, 1);
					VSL.Countdown = TTA-VSL.TTB-MVA.TimeBeforeApproach;
					//warp to the nearest approach point if requested
					if(CFG.WarpToNode && TCA.ATC.Aligned)
						TCA.WRP.WarpToTime = VSL.UT+VSL.Countdown-TCA.ATC.AttitudeError;
					if(VSL.Countdown > 0) return;
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
			if(MainThrust || dVm > MVA.TranslationThreshold || !VSL.TranslationAvailable)
			{
				CFG.AT.OnIfNot(Attitude.AntiRelVel);
				if(MainThrust || TCA.ATC.AttitudeError < MVA.StartAttitudeError)
				{
					TCA.THR.DeltaV = dVm;
					MainThrust = TCA.ATC.AttitudeError < GLB.ATC.AttitudeErrorThreshold;
				}
			}
			//if translation is available, use it as necessary
			if(VSL.TranslationAvailable)
			{
				if(dVm <= MVA.TranslationThreshold)
				{
					if(!MainThrust)
						CFG.AT.OnIfNot(Attitude.KillRotation);
					TCA.TRA.AddDeltaV(VSL.LocalDir(dV));
				}
			}
		}
	}
}

