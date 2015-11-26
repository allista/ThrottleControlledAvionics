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
	public class TranslationControl : AutopilotModule
	{
		public class Config : ModuleConfig
		{
			new public const string NODE_NAME = "MVA";

			[Persistent] public float MinDeltaV         = 0.01f; //m/s
			[Persistent] public PID_Controller TransPID = new PID_Controller(0.5f, 0.01f, 0.5f, 0, 1);
		}
		static Config TRA { get { return TCAScenario.Globals.TRA; } }
		public TranslationControl(ModuleTCA tca) { TCA = tca; }

		readonly PIDf_Controller pid = new PIDf_Controller();

		Vector3 Translation;
		public void AddTranslation(Vector3 trans) { Translation += trans; }

		Vector3 DeltaV;
		public void AddDeltaV(Vector3 dV) { DeltaV += dV; }

		public void Off() 
		{ DeltaV = Vector3.zero; Translation = Vector3.zero; pid.Reset(); }


		public override void Init()
		{
			base.Init();
			pid.setPID(TRA.TransPID);
			CFG.AP.AddCallback(Autopilot.MatchVel, Enable);
		}

		protected override void UpdateState()
		{ 
			IsActive = CFG.Enabled && VSL.TranslationAvailable;
			if(IsActive) return;
			pid.Reset();
		}

		protected override void OnAutopilotUpdate(FlightCtrlState s)
		{
			if(!IsActive) return;
			var dVm = DeltaV.magnitude;
			if(dVm >= TRA.MinDeltaV)
			{
				pid.Update(dVm);
				Translation = pid.Action*DeltaV.CubeNorm();
			}
			else pid.Reset();
			if(!Translation.IsZero())
			{ s.X = Translation.x; s.Z = Translation.y; s.Y = Translation.z; }
			Translation = Vector3.zero;
			DeltaV = Vector3.zero;
		}
	}
}

