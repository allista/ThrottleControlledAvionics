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
	public class Anchor : TCAModule
	{
		public class Config : ModuleConfig
		{
			[Persistent] public float MaxSpeed      = 10;
			[Persistent] public float DistanceF     = 50;
			[Persistent] public float AngularAccelF = 2f;
			[Persistent] public float LookAheadTime = 2f;
			[Persistent] public float SlowTorqueF   = 1f;
			[Persistent] public float DistanceCurve = 2f;
			[Persistent] public PID_Controller DistancePID = new PID_Controller(0.5f, 0f, 0.5f, 0, 100);
		}
		static Config ANC { get { return Globals.Instance.ANC; } }
		public Anchor(ModuleTCA tca) : base(tca) {}

		readonly PIDf_Controller pid = new PIDf_Controller();
		readonly EWA AccelCorrection = new EWA();

		public override void Init()
		{
			base.Init();
			pid.setPID(ANC.DistancePID);
			pid.Min = 0;
			pid.Max = ANC.MaxSpeed;
			pid.Reset();
			CFG.Nav.AddHandler(this, Navigation.Anchor, Navigation.AnchorHere);
		}

		protected override void UpdateState() 
		{ 
			base.UpdateState();
			IsActive &= VSL.OnPlanet && !VSL.LandedOrSplashed && CFG.Nav.Any(Navigation.Anchor, Navigation.AnchorHere); 
		}

		public void AnchorCallback(Multiplexer.Command cmd)
		{
			pid.Reset();
			switch(cmd)
			{
			case Multiplexer.Command.Resume:
				RegisterTo<SASBlocker>();
				break;
			case Multiplexer.Command.On:
				if(CFG.Anchor == null) return;
				VSL.UpdateOnPlanetStats();
				goto case Multiplexer.Command.Resume;

			case Multiplexer.Command.Off:
				UnregisterFrom<SASBlocker>();
				CFG.Anchor = null;
				break;
			}
		}

		public void AnchorHereCallback(Multiplexer.Command cmd)
		{
			if(cmd == Multiplexer.Command.On)
				CFG.Anchor = new WayPoint(VSL.Body.GetLatitude(VSL.Physics.wCoM), 
				                          VSL.Body.GetLongitude(VSL.Physics.wCoM));
			AnchorCallback(cmd);
		}

		protected override void Update()
		{
			if(!IsActive || CFG.Anchor == null) return;
			if(VSL.HorizontalSpeed > ANC.MaxSpeed)
				CFG.HF.OnIfNot(HFlight.NoseOnCourse);
			else CFG.HF.OnIfNot(HFlight.Move);
			CFG.Anchor.Update(VSL);
			//calculate direct distance
			var apos = CFG.Anchor.GetTransform().position;
			var vdir = Vector3.ProjectOnPlane(apos - (VSL.Physics.wCoM+VSL.HorizontalSpeed.Vector*ANC.LookAheadTime), VSL.Physics.Up);
			var distance = Mathf.Sqrt(vdir.magnitude);
			vdir.Normalize();
			VSL.Info.Destination = apos-VSL.Physics.wCoM;
			var real_dist = Vector3.ProjectOnPlane(VSL.Info.Destination, VSL.Physics.Up).magnitude;
			//tune the pid and update needed velocity
			AccelCorrection.Update(Mathf.Clamp(VSL.Torque.MaxPossible.AA_rad/ANC.AngularAccelF, 0.01f, 1), 0.01f);
			pid.P = ANC.DistancePID.P*AccelCorrection;
			pid.D = ANC.DistancePID.D*(2-AccelCorrection);
			if(real_dist <= Mathf.Max(CFG.Anchor.AbsRadius-VSL.Geometry.R, 1)) 
			{
				distance = 0;
				pid.D = 0;
			}
			// CFG.Anchor.AbsRadius*Mathf.Pow(real_dist/CFG.Anchor.AbsRadius, ANC.DistanceCurve);
			pid.Update(distance*ANC.DistanceF/(VSL.Engines.SlowTorque? 1+VSL.Engines.TorqueResponseTime*ANC.SlowTorqueF : 1f));
			VSL.HorizontalSpeed.SetNeeded(vdir*pid.Action);
//			CSV(pid.P, pid.D, distance, real_dist, pid.Action);//debug
		}

		public override void Draw()
		{
			if(Utils.ButtonSwitch("Anchor", CFG.Nav.Any(Navigation.AnchorHere, Navigation.Anchor), 
			                      "Hold current position", GUILayout.Width(60)))
				apply_cfg(cfg => cfg.Nav.XToggle(Navigation.AnchorHere));
		}

		#if DEBUG
		public void AnchorPointer()
		{
			if(CFG.Anchor == null) return;
			Utils.GLLine(VSL.Physics.wCoM, CFG.Anchor.GetTransform().position, Color.cyan);
		}
		#endif
	}
}

