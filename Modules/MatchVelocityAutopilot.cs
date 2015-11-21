//   MatchVelocityAutopilot.cs
//
//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by/4.0/ 
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

			[Persistent] public float StartAttitudeError   = 0.5f; //deg
			[Persistent] public float TranslationThreshold = 5f;   //m/s
		}
		static Config MVA { get { return TCAScenario.Globals.MVA; } }
		public MatchVelocityAutopilot(ModuleTCA tca) { TCA = tca; }

		public override void Init()
		{
			base.Init();
			CFG.AP.AddCallback(Autopilot.MatchVel, Enable);
		}

		protected override void UpdateState()
		{ 
			IsActive = CFG.Enabled && VSL.HasTarget && 
				CFG.AP[Autopilot.MatchVel] && VSL.InOrbit;
			if(IsActive) return;
			reset();
		}

		public override void Enable(bool enable = true)
		{
			if(enable) 
			{
				Working = false;
				TCA.THR.Throttle = 0;
				CFG.AT.On(Attitude.KillRotation);
			}
			else reset();
		}

		void reset()
		{
			if(Working) 
			{
				TCA.THR.Throttle = 0;
				CFG.AT.On(Attitude.KillRotation);
			}
			CFG.AP.OffIfOn(Autopilot.MatchVel);
			Working = false;
		}

		protected override void Update()
		{
			if(!IsActive) return;
			//calculate velocity delta
			var dV = VSL.vessel.obt_velocity-VSL.Target.GetObtVelocity();
			var dVm = (float)dV.magnitude;
			//if it's too small, don't do anything
			if(dVm < GLB.THR.MinDeltaV) return;
			//use main engines if dV is big enough, or if there's no translation capabilities
			TCA.THR.Throttle = 0;
			if(Working || dVm > MVA.TranslationThreshold || !VSL.TranslationAvailable)
			{
				CFG.AT.OnIfNot(Attitude.AntiRelVel);
				if(Working || TCA.ATC.AttitudeError < MVA.StartAttitudeError)
				{
					TCA.THR.DeltaV = dVm;
					Working = TCA.ATC.AttitudeError < GLB.ATC.AttitudeErrorThreshold;
				}
			}
			//if translation is available, use it as necessary
			if(VSL.TranslationAvailable)
			{
				if(dVm <= MVA.TranslationThreshold && !Working)
				{
					CFG.AT.OnIfNot(Attitude.KillRotation);
					TCA.TRA.AddDeltaV(VSL.LocalDir(dV));
				}
				else //calculate remaining time
				{
					var pos = VSL.Target.GetTransform().position-VSL.wCoM;
					if(Vector3.Dot(pos, dV) < 0 || 
					   Vector3.Project(pos, dV).magnitude/dVm < TCA.ATC.AttitudeError) 
						TCA.TRA.AddDeltaV(VSL.LocalDir(dV));
				}
			}
		}
	}
}

