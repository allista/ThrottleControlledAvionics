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

			[Persistent] public float AltAheadFilter = 0.95f; //altitude ahead should less than Altitude*AltAheadFilter to take effect
			[Persistent] public float MaxSpeed = 100f; //Maximum absolute vertical velocity
			[Persistent] public float ErrF  = 0.01f; //altitude error coefficient
			[Persistent] public float TWRp  = 2f;    //twr power factor
			[Persistent] public float TWRd  = 2f;    //twr denominator

			[Persistent] public float RelAltitudeFactor = 100f;
			[Persistent] public float RelVelocityErrF   = 100f;

			[Persistent] public float FallingTime       = 1f;
			[Persistent] public float TimeAhead         = 5f;

			[Persistent] public PID_Controller RocketPID = new PID_Controller(0.1f, 0.5f, 0.03f, -9.9f, -9.9f);
			[Persistent] public PID_Controller JetsPID   = new PID_Controller(0.5f, 0, 0.5f, -9.9f, -9.9f);

			public override void Init()
			{
				RocketPID.Max = JetsPID.Max =  MaxSpeed;
				RocketPID.Min = JetsPID.Min = -MaxSpeed;
			}
		}
		static Config ALT { get { return TCAConfiguration.Globals.ALT; } }

		readonly PIDf_Controller2 rocket_pid = new PIDf_Controller2();
		readonly PIDf_Controller  jets_pid   = new PIDf_Controller();
		readonly Timer            Falling    = new Timer();

		public AltitudeControl(VesselWrapper vsl) { VSL = vsl; }

		public override void Init()
		{
			rocket_pid.setPID(ALT.RocketPID);
			jets_pid.setPID(ALT.JetsPID);
			Falling.Period = ALT.FallingTime;
		}

		public override void UpdateState()
		{ IsActive = CFG.VF[VFlight.AltitudeControl] && VSL.OnPlanet; }

		public void SetAltitudeAboveTerrain(bool enable = true)
		{
			if(enable == CFG.AltitudeAboveTerrain) return;
			CFG.AltitudeAboveTerrain = enable;
			VSL.UpdateAltitude();
			VSL.AltitudeAhead = -1;
			Falling.Reset();
			if(CFG.AltitudeAboveTerrain)
				CFG.DesiredAltitude -= VSL.TerrainAltitude;
			else CFG.DesiredAltitude += VSL.TerrainAltitude;
		}

		public override void Enable(bool enable = true)
		{
			Falling.Reset();
			CFG.VF[VFlight.AltitudeControl] = enable;
			if(enable)
			{
				VSL.UpdateAltitude();
				CFG.DesiredAltitude = VSL.Altitude;
			}
		}

		public void Update()
		{
			if(!IsActive) return;
			SetState(TCAState.AltitudeControl);
			//calculate altitude error
			var alt = VSL.Altitude;
			if(CFG.AltitudeAboveTerrain && 
			   VSL.AltitudeAhead >= 0 &&
			   VSL.AltitudeAhead < VSL.Altitude)
			{
				if(VSL.AbsVerticalSpeed > 0) SetState(TCAState.Ascending);
				alt = VSL.AltitudeAhead;
			}
			var error = CFG.DesiredAltitude-alt; 
			//if following terrain, filter noise depending on the altitude
			if(CFG.AltitudeAboveTerrain && error < 0)
				error = error/Utils.ClampL(VSL.HorizontalSpeed, 1);
			//update pids
			if((VSL.AccelSpeed > 0 || VSL.DecelSpeed > 0))
			{
//				if(VSL.VerticalSpeed > 0)
//					jets_pid.P = Mathf.Clamp(ALT.ErrF*Mathf.Abs(error)/Utils.ClampL(VSL.VerticalSpeed, 1), 0, ALT.JetsPID.D);
//				else if(VSL.VerticalSpeed < 0)
				jets_pid.P = Utils.ClampH(ALT.JetsPID.P/VSL.MaxTWR/ALT.TWRd*Mathf.Clamp(Mathf.Abs(1/VSL.VerticalSpeed)*ALT.ErrF, 1, VSL.MaxTWR*ALT.TWRd), ALT.JetsPID.P);
//				else jets_pid.P = ALT.JetsPID.P;
				if(CFG.AltitudeAboveTerrain)
					jets_pid.D = ALT.JetsPID.D/Utils.ClampL(VSL.HorizontalSpeed, 1);
				jets_pid.Update(error);
				CFG.VerticalCutoff = jets_pid.Action;
			}
			else 
			{
				if(CFG.AltitudeAboveTerrain)
					rocket_pid.D = ALT.RocketPID.D/Utils.ClampL(VSL.HorizontalSpeed, 1);
				rocket_pid.Update(error);
				CFG.VerticalCutoff = rocket_pid.Action;
			}
			//correct for relative vertical speed and relative altitude
			if(CFG.AltitudeAboveTerrain)
			{
				var dV = (VSL.AbsVerticalSpeed-VSL.RelVerticalSpeed)/Utils.ClampL(alt/ALT.RelAltitudeFactor, 1);
//				if(Mathf.Abs(dV) > ALT.MaxSpeed/10)
//				{
//					dV -= dV > 0? ALT.MaxSpeed/10 : ALT.MaxSpeed/-10;
					if(error < 0) 
//						dV = dV/Utils.ClampL(alt/ALT.RelAltitudeFactor, 1) +
					dV += Utils.ClampL(error/CFG.DesiredAltitude/ALT.RelVelocityErrF*VSL.HorizontalSpeed, -10*ALT.MaxSpeed);
					else 
						dV = Utils.ClampL(dV, 0);//Utils.ClampL(alt/ALT.RelAltitudeFactor, 1);
					CFG.VerticalCutoff += dV;
//				} else dV = 0;
				//Loosing Altitude alert
				if(VSL.RelVerticalSpeed < 0 && VSL.CFG.VerticalCutoff-VSL.VerticalSpeed > 0 
				   && VSL.Altitude < CFG.DesiredAltitude-VSL.RelVerticalSpeed*ALT.TimeAhead)
				{ if(!Falling.Check) SetState(TCAState.LoosingAltitude); }
				else Falling.Reset();

				DebugUtils.CSV(VSL.vessel.altitude, VSL.TerrainAltitude, VSL.Altitude, VSL.AltitudeAhead, error, 
				               CFG.VerticalCutoff, VSL.VSF, VSL.MinVSF, VSL.AbsVerticalSpeed, VSL.RelVerticalSpeed, dV, VSL.MaxDTWR, VSL.MaxTWR, VSL.HorizontalVelocity.magnitude);//debug
			}
		}
	}
}

