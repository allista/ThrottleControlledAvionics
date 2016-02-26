//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

using UnityEngine;

namespace ThrottleControlledAvionics
{
	[CareerPart]
	[RequireModules(typeof(HorizontalSpeedControl),
	         	    typeof(SASBlocker))]
	public class CruiseControl : AutopilotModule
	{
		public class Config : ModuleConfig
		{
			new public const string NODE_NAME = "CC";

			[Persistent] public float UpdateDelay = 1;
		}
		static Config CC { get { return TCAScenario.Globals.CC; } }

		readonly ActionDamper UpdateTimer = new ActionDamper();
		bool inited;

		public CruiseControl(ModuleTCA tca) : base(tca) {}

		public override void Init()
		{
			base.Init();
			UpdateTimer.Period = CC.UpdateDelay;
			CFG.HF.AddHandler(this, HFlight.CruiseControl);
		}

		protected override void UpdateState() 
		{ 
			IsActive = VSL.OnPlanet && CFG.HF.Any(HFlight.Stop, HFlight.NoseOnCourse, HFlight.CruiseControl); 
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
				RegisterTo<Radar>();
				break;

			case Multiplexer.Command.On:
				VSL.UpdateOnPlanetStats();
				var nV = VSL.HorizontalSpeed.Absolute;
				if(nV < GLB.PN.MinSpeed) nV = GLB.PN.MinSpeed;
				else if(nV > GLB.PN.MaxSpeed) nV = GLB.PN.MaxSpeed;
				VSL.HorizontalSpeed.SetNeeded(VSL.HorizontalSpeed.Vector.normalized*nV);
				CFG.MaxNavSpeed = needed_velocity = nV;
				goto case Multiplexer.Command.Resume;

			case Multiplexer.Command.Off:
				VSL.HorizontalSpeed.SetNeeded(Vector3d.zero);
				UnregisterFrom<SASBlocker>();
				UnregisterFrom<Radar>();
				break;
			}
		}

		public void UpdateNeededVelocity()
		{
			VSL.HorizontalSpeed.SetNeeded(CFG.NeededHorVelocity.IsZero()? 
			                              Vector3.zero : 
			                              Quaternion.FromToRotation(CFG.SavedUp, VSL.Physics.Up)*CFG.NeededHorVelocity);
		}

		float needed_velocity;
		protected override void OnAutopilotUpdate(FlightCtrlState s)
		{
			//need to check all the prerequisites, because the callback is called asynchroniously
			if(!(CFG.Enabled && VSL.OnPlanet && VSL.refT != null &&
			     CFG.HF[HFlight.CruiseControl])) return;
			//allow user to intervene
			if(VSL.AutopilotDisabled) 
			{ 
				CFG.MaxNavSpeed = Utils.Clamp((float)VSL.HorizontalSpeed.NeededVector.magnitude-s.pitch, GLB.PN.MinSpeed, GLB.PN.MaxSpeed);
				var new_velocity = Quaternion.AngleAxis(s.yaw, VSL.Physics.Up) * VSL.HorizontalSpeed.NeededVector;
				VSL.HorizontalSpeed.SetNeeded(new_velocity.normalized * CFG.MaxNavSpeed);
				needed_velocity = CFG.MaxNavSpeed;
				s.pitch = s.yaw = 0;
				return; 
			}
			else if(Mathf.Abs(needed_velocity-CFG.MaxNavSpeed) > 1)
			{
				CFG.NeededHorVelocity = CFG.NeededHorVelocity.normalized * CFG.MaxNavSpeed;
				needed_velocity = CFG.MaxNavSpeed;
				UpdateNeededVelocity();
			}
			//update needed velocity
			else if(CFG.HF[HFlight.CruiseControl]) UpdateTimer.Run(UpdateNeededVelocity);
		}
	}
}

