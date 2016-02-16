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
	public class Anchor : TCAModule
	{
		public class Config : ModuleConfig
		{
			new public const string NODE_NAME = "ANC";

			[Persistent] public float MaxSpeed      = 10;
			[Persistent] public float DistanceF     = 50;
			[Persistent] public float AngularAccelF = 2f;
			[Persistent] public float MaxAccelF     = 4f;
			[Persistent] public float LookAheadTime = 2f;
			[Persistent] public float SlowTorqueF   = 1f;
			[Persistent] public PID_Controller DistancePID = new PID_Controller(0.5f, 0f, 0.5f, 0, 100);
		}
		static Config ANC { get { return TCAScenario.Globals.ANC; } }
		public Anchor(ModuleTCA tca) { TCA = tca; }

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
			#if DEBUG
			RenderingManager.AddToPostDrawQueue(1, AnchorPointer);
			#endif
		}

		protected override void UpdateState() 
		{ IsActive = VSL.OnPlanet && !VSL.LandedOrSplashed && CFG.Nav.Any(Navigation.Anchor, Navigation.AnchorHere); }

		public void AnchorCallback(Multiplexer.Command cmd)
		{
			pid.Reset();
			switch(cmd)
			{
			case Multiplexer.Command.Resume:
				TCA.SASC.Register(this);
				break;
			case Multiplexer.Command.On:
				if(CFG.Anchor == null) return;
				VSL.UpdateOnPlanetStats();
				goto case Multiplexer.Command.Resume;

			case Multiplexer.Command.Off:
				TCA.SASC.Unregister(this);
				CFG.Anchor = null;
				break;
			}
		}

		public void AnchorHereCallback(Multiplexer.Command cmd)
		{
			if(cmd == Multiplexer.Command.On)
				CFG.Anchor = new WayPoint(VSL.mainBody.GetLatitude(VSL.wCoM), 
				                          VSL.mainBody.GetLongitude(VSL.wCoM));
			AnchorCallback(cmd);
		}

		protected override void Update()
		{
			if(!IsActive || CFG.Anchor == null) return;
			if(VSL.HorizontalSpeed > ANC.MaxSpeed)
				CFG.HF.OnIfNot(HFlight.NoseOnCourse);
			else CFG.HF.OnIfNot(HFlight.Move);
			CFG.Anchor.Update(VSL.mainBody);
			//calculate direct distance
			var vdir = Vector3.ProjectOnPlane(CFG.Anchor.GetTransform().position - 
			                                  (VSL.wCoM+
			                                   VSL.HorizontalVelocity*ANC.LookAheadTime), 
			                                  VSL.Up);
			var distance = Mathf.Sqrt(vdir.magnitude);
			vdir.Normalize();
			//tune the pid and update needed velocity
			AccelCorrection.Update(Mathf.Clamp(VSL.MaxThrustM*VSL.MinVSFtwr/VSL.M/ANC.MaxAccelF, 0.01f, 1)*
			                       Mathf.Clamp(VSL.MaxPitchRollAA_m/ANC.AngularAccelF, 0.01f, 1), 0.01f);
			pid.P   = ANC.DistancePID.P*AccelCorrection;
			pid.D   = ANC.DistancePID.D*(2-AccelCorrection);
			pid.Update(distance*ANC.DistanceF/(VSL.SlowTorque? 1+VSL.TorqueResponseTime*ANC.SlowTorqueF : 1f));
			TCA.HSC.SetNeededHorVelocity(vdir*pid.Action);
//			Log("dist {0}, pid {1}, nHV {2}, slow {3}, torque response {4}", 
//			    distance, pid, TCA.HSC.NeededHorVelocity, slow, VSL.TorqueResponseTime);//debug
		}

		#if DEBUG
		public void AnchorPointer()
		{
			if(CFG.Anchor == null) return;
			GLUtils.GLLine(VSL.wCoM, CFG.Anchor.GetTransform().position, Color.cyan);
		}

		public override void Reset()
		{
			base.Reset();
			RenderingManager.RemoveFromPostDrawQueue(1, AnchorPointer);
		}
		#endif
	}
}

