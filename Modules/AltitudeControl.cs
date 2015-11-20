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

			[Persistent] public float MaxSpeedErrorF = 100f;
			[Persistent] public float MaxSpeedLow  = 10f;  //Maximum absolute vertical velocity
			[Persistent] public float MaxSpeedHigh = 100f; //Maximum absolute vertical velocity
			[Persistent] public float ErrF  = 1f;     //altitude error coefficient
			[Persistent] public float TWRd  = 2f;     //twr denominator

			[Persistent] public float RelAltitudeFactor = 100f;
			[Persistent] public float RelVelocityErrF   = 100f;

			[Persistent] public float FallingTime       = 1f;
			[Persistent] public float TimeAhead         = 5f;

			[Persistent] public PID_Controller RocketPID = new PID_Controller(0.1f, 0.5f, 0.03f, -9.9f, -9.9f);
			[Persistent] public PID_Controller JetsPID   = new PID_Controller(0.5f, 0, 0.5f, -9.9f, -9.9f);

			public override void Init()
			{
				RocketPID.Max = JetsPID.Max =  MaxSpeedLow;
				RocketPID.Min = JetsPID.Min = -MaxSpeedLow;
			}
		}
		static Config ALT { get { return TCAScenario.Globals.ALT; } }

		readonly PIDf_Controller2 rocket_pid = new PIDf_Controller2();
		readonly PIDf_Controller  jets_pid   = new PIDf_Controller();
		readonly Timer            Falling    = new Timer();

		public AltitudeControl(VesselWrapper vsl) { VSL = vsl; }

		public override void Init()
		{
			rocket_pid.setPID(ALT.RocketPID);
			jets_pid.setPID(ALT.JetsPID);
			Falling.Period = ALT.FallingTime;
			CFG.VF.AddCallback(VFlight.AltitudeControl, Enable);
			if(VSL.LandedOrSplashed) CFG.DesiredAltitude = -10;
		}

		protected override void UpdateState()
		{ IsActive = VSL.OnPlanet && CFG.VF[VFlight.AltitudeControl]; }

		public void SetAltitudeAboveTerrain(bool enable = true)
		{
			if(enable == CFG.AltitudeAboveTerrain) return;
			CFG.AltitudeAboveTerrain = enable;
			VSL.UpdateAltitudeInfo();
			Falling.Reset();
			if(CFG.AltitudeAboveTerrain)
				CFG.DesiredAltitude -= VSL.TerrainAltitude;
			else CFG.DesiredAltitude += VSL.TerrainAltitude;
		}

		public override void Enable(bool enable = true)
		{
			Falling.Reset();
			if(enable)
			{
				VSL.UpdateAltitudeInfo();
				CFG.DesiredAltitude = VSL.LandedOrSplashed? -10 : VSL.Altitude;
			}
		}

		protected override void Update()
		{
			if(!IsActive) return;
			SetState(TCAState.AltitudeControl);
			//calculate current altitude or apoapsis, if the vessel is moving upwards
			var alt = VSL.AbsAltitude;
			var ttAp = VSL.AbsVerticalSpeed/VSL.G;
			if(VSL.AbsVerticalSpeed > 0 && !VSL.LandedOrSplashed)
			{
				if(VSL.TimeAhead > 0 && VSL.TimeAhead < ttAp) ttAp = VSL.TimeAhead;
				alt = VSL.AbsAltitude+ttAp*(VSL.AbsVerticalSpeed - ttAp*VSL.G/2);
//				CSV(CFG.DesiredAltitude, alt-VSL.TerrainAltitude, VSL.RelAltitude, VSL.AltitudeAhead-VSL.AbsAltitude,
//				    VSL.VSF, -VSL.G, ttAp, VSL.TimeAhead);//debug
			}
			//correct for terrain altitude and radar data if following terrain
			if(CFG.AltitudeAboveTerrain) 
			{
				if(VSL.AltitudeAhead > VSL.TerrainAltitude)
				{
					alt -= VSL.AltitudeAhead;
					if(alt <= VSL.H) 
					{
						SetState(VSL.AbsVerticalSpeed < 0? TCAState.GroundCollision : TCAState.ObstacleAhead);
						if(VSL.TimeAhead > 0) 
						{
							CFG.VerticalCutoff = Mathf.Sqrt(2f*Utils.ClampL((VSL.AltitudeAhead+CFG.DesiredAltitude-VSL.AbsAltitude)*VSL.G, 0));
							ttAp = CFG.VerticalCutoff/VSL.G;
							if(ttAp > VSL.TimeAhead) 
								CFG.VerticalCutoff = (VSL.AltitudeAhead+CFG.DesiredAltitude-VSL.AbsAltitude)/VSL.TimeAhead+VSL.TimeAhead*VSL.G/2;
//							Log("VSP {0}, ttAp {1}, TimeAhead {2}, ApA {3}, Obst {4}", CFG.VerticalCutoff, ttAp, VSL.TimeAhead, 
//							    VSL.AbsAltitude+ttAp*(CFG.VerticalCutoff - ttAp*VSL.G/2), VSL.AltitudeAhead);//debug
							return;
						}
					}
					else if(VSL.AbsVerticalSpeed > 1 && CFG.VerticalCutoff > 1) SetState(TCAState.Ascending);
				}
				else alt -= VSL.TerrainAltitude;
			}
			//calculate altitude error
			var error = (CFG.DesiredAltitude-alt);
			//turn off the engines if landed
			if(VSL.LandedOrSplashed && error < 0 && VSL.RelAltitude <= VSL.R)
			{
				CFG.VerticalCutoff = -GLB.VSC.MaxSpeed;
				return;
			}
			//update pid parameters and vertical speed setpoint
			var min_speed = -ALT.MaxSpeedLow;
			var max_speed =  ALT.MaxSpeedLow;
			if(error < 0)
			{
				min_speed = Utils.Clamp(ALT.MaxSpeedLow*(error+ALT.MaxSpeedErrorF)/ALT.MaxSpeedErrorF, -ALT.MaxSpeedHigh, -ALT.MaxSpeedLow);
				if(VSL.AbsVerticalSpeed < 0)
				{
					
					var free_fall  = (VSL.AbsVerticalSpeed+Mathf.Sqrt(VSL.AbsVerticalSpeed*VSL.AbsVerticalSpeed-2*VSL.G*error))/VSL.G;
					var brake_time = -VSL.AbsVerticalSpeed/(VSL.MaxDTWR-1)/VSL.G;
					if(brake_time < 0 || brake_time >= free_fall) min_speed = 0;
					else if(brake_time > free_fall/100) 
						min_speed = Utils.Clamp(-ALT.MaxSpeedHigh*(1-brake_time/free_fall), min_speed, -ALT.MaxSpeedLow);
//					Log("free_fall {0}, brake_time {1}, min_speed {2}, error {3}", free_fall, brake_time, min_speed, error);//debug
				}
			}
			else if(error > 0) max_speed = alt <= VSL.H? ALT.MaxSpeedHigh :
				Utils.Clamp(ALT.MaxSpeedLow*(error-ALT.MaxSpeedErrorF)/ALT.MaxSpeedErrorF, ALT.MaxSpeedLow, ALT.MaxSpeedHigh);
			if(VSL.SlowThrust)
			{
				jets_pid.Min = min_speed;
				jets_pid.Max = max_speed; 
				jets_pid.P = Utils.ClampH(ALT.JetsPID.P/VSL.MaxTWR/ALT.TWRd*
				                          Mathf.Clamp(Mathf.Abs(1/VSL.VerticalSpeed)*ALT.ErrF, 1, VSL.MaxTWR*ALT.TWRd), 
				                          ALT.JetsPID.P);
				if(CFG.AltitudeAboveTerrain)
					jets_pid.D = ALT.JetsPID.D/Utils.ClampL(VSL.HorizontalSpeed, 1);
				jets_pid.Update(error);
				CFG.VerticalCutoff = jets_pid.Action;
			}
			else 
			{
				rocket_pid.Min = min_speed;
				rocket_pid.Max = max_speed;
				if(CFG.AltitudeAboveTerrain)
					rocket_pid.D = ALT.RocketPID.D/Utils.ClampL(VSL.HorizontalSpeed, 1);
				rocket_pid.Update(error);
				CFG.VerticalCutoff = rocket_pid.Action;
			}
			//correct for relative vertical speed
			//if following the terrain and flying below desired altitude
			if(CFG.AltitudeAboveTerrain)
			{
				var dV = 0f;
				if(error > 0) 
					dV = Utils.ClampL((VSL.AbsVerticalSpeed-VSL.RelVerticalSpeed)/
					                  Utils.ClampL(alt/ALT.RelAltitudeFactor, 1), 0);
				CFG.VerticalCutoff += dV;
				//Loosing Altitude alert
				Falling.RunIf(() => SetState(TCAState.LoosingAltitude),
				              VSL.RelVerticalSpeed < 0 && 
				              VSL.CFG.VerticalCutoff-VSL.VerticalSpeed > 0 && 
				              VSL.Altitude < CFG.DesiredAltitude-VSL.RelVerticalSpeed*ALT.TimeAhead);
//				Log("error {0}, dV: {1}, VSP: {2}, min speed {3}, max speed {4}", 
//				    error, dV, CFG.VerticalCutoff, min_speed, max_speed);//debug
//				CSV(alt, VSL.vessel.altitude, VSL.TerrainAltitude, VSL.Altitude, VSL.AltitudeAhead, error, 
//				    CFG.VerticalCutoff, VSL.VSF, VSL.MinVSF, VSL.AbsVerticalSpeed, VSL.RelVerticalSpeed, 
//				    dV, min_speed, max_speed);//debug
			}
		}
	}
}

