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
	[RequireModules(typeof(HorizontalSpeedControl))]
	[OptionalModules(typeof(PointNavigator),
	                 typeof(VerticalSpeedControl),
	                 typeof(AttitudeControl))]
	public class VTOLAssist : TCAModule
	{
		public class Config : ModuleConfig
		{
			[Persistent] public float MinHSpeed   = 0.1f;
			[Persistent] public float MaxHSpeed   = 10;
			[Persistent] public float GearTimer   = 1;
			[Persistent] public float LandedTimer = 2;
			//liftoff
			[Persistent] public float MinDTWR     = 0.5f;
			[Persistent] public float MinAngularVelocity = 0.001f; //(rad/s)^2 ~= 1.8deg/s
			[Persistent] public float GearOffAngularVelocity = 0.01f; //(rad/s)^2 ~= 1.8deg/s
			//landing
			[Persistent] public float GearOnMaxHSpeed = 1f;
			[Persistent] public float GearOnAtH       = 5f;
			[Persistent] public float GearOnTime      = 5f;
		}
		static Config TLA { get { return Globals.Instance.TLA; } }
		public VTOLAssist(ModuleTCA tca) : base(tca) {}

		VerticalSpeedControl VSC;
		AttitudeControl ATC;

		enum Stage { Landed, Flying, JustLanded1, JustLanded2, JustTookoff }
		Stage stage;

		bool was_landed;
		Vector3 srf_normal;
		readonly Timer GearTimer   = new Timer();
		readonly Timer LandedTimer = new Timer();
		readonly SingleAction StopAction = new SingleAction();

		void working(bool state = true)
		{
			if(state) 
			{
				CFG.Nav.Paused = true;
				SetState(TCAState.VTOLAssist);
			}
			else if(state != Working)
			{
				CFG.HF.OffIfOn(HFlight.Level);
				CFG.Nav.Paused = false;
			}
			Working = state;
		}

		public override void Init()
		{
			base.Init();
			GearTimer.Period = TLA.GearTimer;
			LandedTimer.Period = TLA.LandedTimer;
			StopAction.action = () => CFG.HF.OnIfNot(HFlight.Stop);
            Reset();
		}

        public override void Disable()
        {
            Reset();
        }

        protected override void Resume()
        {
            base.Resume();
            Reset();
        }

		protected override void UpdateState()
		{ 
			base.UpdateState();
			IsActive &= VSL.OnPlanet && CFG.VTOLAssistON;
		}

        protected override void Reset()
        {
            was_landed = VSL.LandedOrSplashed;
            stage = was_landed ? Stage.Landed : Stage.Flying;
            working(false);
        }

		protected override void Update()
		{
			//update state
			if(was_landed && !VSL.LandedOrSplashed) 
			{ stage = Stage.JustTookoff; GearTimer.Reset(); StopAction.Reset(); }
			else if(VSL.LandedOrSplashed && !was_landed) stage = Stage.JustLanded1;
			was_landed = VSL.LandedOrSplashed;
			switch(stage)
			{
			case Stage.JustLanded1:
				working();
				srf_normal = Vector3.zero;
				CFG.HF.OnIfNot(HFlight.Level);
				VSL.BrakesOn();
				LandedTimer.RunIf(() => stage = Stage.JustLanded2,
				                  VSL.HorizontalSpeed < TLA.MinHSpeed);
				break;
			case Stage.JustLanded2:
				if(ATC != null)
				{
					if(srf_normal.IsZero())
					{
						RaycastHit hit;
						if(Physics.Raycast(VSL.Physics.wCoM, -VSL.Physics.Up, out hit, VSL.Geometry.D, Radar.RadarMask))
						{
							if(hit.collider != null && !hit.normal.IsZero())
							{
								srf_normal = -hit.normal;
								break;
							}
						}
						stage = Stage.Landed;
						break;
					}
					working();
					CFG.HF.Off();
					CFG.AT.OnIfNot(Attitude.Custom);
					ATC.SetThrustDirW(srf_normal);
					LandedTimer.RunIf(() => { stage = Stage.Landed; CFG.AT.Off(); }, 
					                  VSL.Physics.NoRotation || VSL.Controls.AttitudeError < 1);
					break;
				}
				stage = Stage.Landed;
				break;
			case Stage.JustTookoff:
				working();
				StopAction.Run();
				GearTimer.RunIf(() =>
				{ 
					VSL.BrakesOn(false);
					VSL.GearOn(false);
					stage = Stage.Flying;
				}, VSL.Altitude.Relative > 2*VSL.Geometry.H);
				break;
			case Stage.Landed:
				if(CFG.VerticalCutoff <= 0) working(false);
				else
				{
					var avSqr = VSL.vessel.angularVelocity.sqrMagnitude;
					if(VSL.HorizontalSpeed < TLA.MaxHSpeed &&
					   avSqr > TLA.MinAngularVelocity)
					{
						working();
						CFG.HF.OnIfNot(HFlight.Level);
						if(avSqr > TLA.GearOffAngularVelocity && 
						   VSL.OnPlanetParams.DTWR > TLA.MinDTWR)
							VSL.GearOn(false);
					}
					else working(false);
				}
				break;
			case Stage.Flying:
				working(false);
				if(!VSL.vessel.ActionGroups[KSPActionGroup.Gear])
				{
					if(VSL.VerticalSpeed.Relative < 0 &&
					   VSL.HorizontalSpeed < TLA.GearOnMaxHSpeed &&
					   (!CFG.AT || !VSL.Altitude.AboveGround || VSL.Engines.Thrust.IsZero()) &&
					   VSL.Altitude.Relative+
					   VSL.VerticalSpeed.Relative*(VSL.OnPlanetParams.GearDeployTime+TLA.GearOnTime) 
					   < TLA.GearOnAtH*VSL.Geometry.H)
					{
						VSL.GearOn(); 
						VSL.BrakesOn();
					}
				}
				else if(VSL.OnPlanetParams.GearDeploying)
				{
					if(VSC != null) 
						VSC.SetpointOverride = Utils.ClampH((TLA.GearOnAtH*VSL.Geometry.H-VSL.Altitude.Relative)/
						                                    (VSL.OnPlanetParams.GearDeployTime+TLA.GearOnTime), 0);
				}
				else GearTimer.RunIf(() => 
				{
					VSL.GearOn(false); 
					VSL.BrakesOn(false);
				},
				                     VSL.VerticalSpeed.Relative > 5 ||
				                     VSL.HorizontalSpeed > TLA.GearOnMaxHSpeed || 
				                     VSL.VerticalSpeed.Relative > 0 && VSL.Altitude.Relative > VSL.Geometry.H*5);
				break;
			}
		}
	}
}

