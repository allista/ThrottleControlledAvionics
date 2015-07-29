//   AltitudeControl.cs
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
	public class AltitudeControl : TCAModule
	{
		public class Config : ModuleConfig
		{
			new public const string NODE_NAME = "ALT";

			[Persistent] public float MaxSpeed = 100f; //Maximum absolute vertical velocity
			[Persistent] public float ErrF  = 0.01f; //altitude error coefficient
			[Persistent] public float TWRp  = 2f;    //twr power factor
			[Persistent] public float TWRd  = 2f;    //twr denominator
			[Persistent] public PID_Controller RocketPID = new PIDf_Controller(0.1f, 0.5f, 0.03f, -9.9f, -9.9f);
			[Persistent] public PID_Controller JetsPID   = new PIDf_Controller(0.5f, 0, 0.5f, -9.9f, -9.9f);

			public override void Init()
			{
				RocketPID.Max = JetsPID.Max =  MaxSpeed;
				RocketPID.Min = JetsPID.Min = -MaxSpeed;
			}
		}
		static Config ALT { get { return TCAConfiguration.Globals.ALT; } }

		readonly PIDf_Controller2 rocket_pid = new PIDf_Controller2();
		readonly PIDf_Controller  jets_pid   = new PIDf_Controller();

		public AltitudeControl(VesselWrapper vsl) { vessel = vsl; }

		public override void Init()
		{
			rocket_pid.setPID(ALT.RocketPID);
			jets_pid.setPID(ALT.JetsPID);
		}
		public override void UpdateState()
		{ IsActive = CFG.ControlAltitude && vessel.OnPlanet; }

		public void Update()
		{
			if(!IsActive) return;
			SetState(TCAState.AltitudeControl);
			var alt_error = CFG.DesiredAltitude-vessel.Altitude;
			if((vessel.AccelSpeed > 0 || vessel.DecelSpeed > 0))
			{
				if(vessel.VerticalSpeed > 0)
					jets_pid.P = Mathf.Clamp(ALT.ErrF*Mathf.Abs(alt_error/vessel.VerticalSpeed), 
					                                    0, jets_pid.D);
				else if(vessel.VerticalSpeed < 0)
					jets_pid.P = Mathf.Clamp(Mathf.Pow(vessel.MaxTWR, ALT.TWRp)/Mathf.Abs(vessel.VerticalSpeed), 
					                                    0, Utils.ClampH(jets_pid.D/vessel.MaxTWR/ALT.TWRd, jets_pid.D));
				else jets_pid.P = ALT.JetsPID.P;
				jets_pid.Update(alt_error);
				CFG.VerticalCutoff = jets_pid.Action;
			}
			else 
			{
				rocket_pid.Update(alt_error);
				CFG.VerticalCutoff = rocket_pid.Action;
			}
//			Utils.CSV(vessel.Altitude, CFG.VerticalCutoff, vessel.VSF, vessel.VerticalSpeed);//debug
		}
	}
}

