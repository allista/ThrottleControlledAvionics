//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

using UnityEngine;
using AT_Utils;

namespace ThrottleControlledAvionics
{
	[CareerPart]
	[RequireModules(typeof(HorizontalSpeedControl),
					typeof(BearingControl),
	         	    typeof(SASBlocker))]
	public class CruiseControl : AutopilotModule
	{
		public class Config : ModuleConfig
		{
			[Persistent] public float PitchFactor = 0.2f;
			[Persistent] public float MaxRevSpeed = -4f;
            [Persistent] public float MaxIdleSpeed = 4f;
			[Persistent] public float UpdateDelay = 1;
		}
		static Config CC { get { return Globals.Instance.CC; } }

		BearingControl BRC;

		bool inited;
		float needed_velocity;
		readonly ActionDamper UpdateTimer = new ActionDamper();

		public CruiseControl(ModuleTCA tca) : base(tca) {}

		public override void Init()
		{
			base.Init();
			UpdateTimer.Period = CC.UpdateDelay;
			CFG.HF.AddHandler(this, HFlight.CruiseControl);
		}

        public override void Disable()
        {
            CFG.HF.OffIfOn(HFlight.CruiseControl);
        }

		protected override void UpdateState() 
		{ 
			base.UpdateState();
            IsActive &= VSL.refT != null && VSL.OnPlanet && CFG.HF[HFlight.CruiseControl]; 
			if(!inited && IsActive && !VSL.Physics.Up.IsZero())
			{
				UpdateNeededVelocity();
				inited = true;
			}
		}

		public void CruiseControlCallback(Multiplexer.Command cmd)
		{
			UpdateTimer.Reset();
			switch(cmd)
			{
			case Multiplexer.Command.Resume:
				RegisterTo<SASBlocker>();
				NeedCPSWhenMooving();
				break;

			case Multiplexer.Command.On:
				VSL.UpdateOnPlanetStats();
				var nV = VSL.HorizontalSpeed.Absolute;
				if(nV > GLB.PN.MaxSpeed) nV = GLB.PN.MaxSpeed;
                var nVdir = nV > CC.MaxIdleSpeed?
                    (Vector3)VSL.HorizontalSpeed.Vector.normalized :
                    VSL.OnPlanetParams.Fwd;
				CFG.BR.OnIfNot(BearingMode.User);
                BRC.UpdateBearing((float)VSL.Physics.Bearing(nVdir));
                VSL.HorizontalSpeed.SetNeeded(nVdir*nV);
                CFG.MaxNavSpeed = needed_velocity = nV;
				goto case Multiplexer.Command.Resume;

			case Multiplexer.Command.Off:
				VSL.HorizontalSpeed.SetNeeded(Vector3d.zero);
				UnregisterFrom<SASBlocker>();
				ReleaseCPS();
				CFG.BR.OffIfOn(BearingMode.User);
				break;
			}
		}

		public void UpdateNeededVelocity()
		{
            SetNeededVelocity(CFG.MaxNavSpeed.Equals(0)? 
			                  Vector3d.zero : 
                              VSL.Physics.Direction(BRC.Bearing));
		}

		void SetNeededVelocity(Vector3d nV)
		{
			needed_velocity = CFG.MaxNavSpeed;
			if(CFG.MaxNavSpeed.Equals(0)) 
				VSL.HorizontalSpeed.SetNeeded(Vector3d.zero);
			else if(nV.sqrMagnitude > 0.1 && CFG.MaxNavSpeed > 0)
			{
				nV.Normalize();
				VSL.HorizontalSpeed.SetNeeded(nV*CFG.MaxNavSpeed);
				BRC.UpdateBearing((float)VSL.Physics.Bearing(nV));
			}
			else VSL.HorizontalSpeed.SetNeeded(BRC.ForwardDirection*CFG.MaxNavSpeed);
		}

		protected override void OnAutopilotUpdate()
		{
			if(VSL.HasUserInput) 
			{ 
				if(!CS.pitch.Equals(0))
				{
					CFG.MaxNavSpeed = Utils.Clamp(CFG.MaxNavSpeed-CS.pitch*CC.PitchFactor, CC.MaxRevSpeed, GLB.PN.MaxSpeed);
					SetNeededVelocity(VSL.HorizontalSpeed.NeededVector);
					VSL.HasUserInput = !(CS.yaw.Equals(0) && CS.roll.Equals(0));
					VSL.AutopilotDisabled = VSL.HasUserInput;
					CS.pitch = 0;
				}
			}
			else if(!needed_velocity.Equals(CFG.MaxNavSpeed))
				SetNeededVelocity(VSL.HorizontalSpeed.NeededVector);
			else UpdateTimer.Run(UpdateNeededVelocity);
		}
	}
}

