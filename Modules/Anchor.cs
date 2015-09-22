//   Anchor.cs
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
	public class Anchor : TCAModule
	{
		public class Config : ModuleConfig
		{
			new public const string NODE_NAME = "ANC";

			[Persistent] public float MaxSpeed      = 10;
			[Persistent] public float DistanceF     = 50;
			[Persistent] public float AngularAccelF = 2f;
			[Persistent] public float MaxAccelF     = 4f;
			[Persistent] public PID_Controller DistancePID = new PID_Controller(0.5f, 0f, 0.5f, 0, 100);
		}
		static Config ANC { get { return TCAScenario.Globals.ANC; } }
		public Anchor(VesselWrapper vsl) { VSL = vsl; }

		readonly PIDf_Controller pid = new PIDf_Controller();
		readonly EWA AccelCorrection = new EWA();

		public override void Init()
		{
			base.Init();
			pid.setPID(ANC.DistancePID);
			pid.Min = 0;
			pid.Max = ANC.MaxSpeed;
			pid.Reset();
			CFG.HF.AddCallback(HFlight.Anchor, Enable);
			CFG.HF.AddCallback(HFlight.AnchorHere, AnchorHere);
			#if DEBUG
			RenderingManager.AddToPostDrawQueue(1, AnchorPointer);
			#endif
		}

		public override void UpdateState() { IsActive = CFG.HF.Any(HFlight.Anchor, HFlight.AnchorHere) && VSL.OnPlanet; }

		public override void Enable(bool enable = true)
		{
			if(enable && CFG.Anchor == null) return;
			pid.Reset();
			CFG.Nav.Off();
			BlockSAS(enable);
			if(enable) VSL.UpdateOnPlanetStats();
			else CFG.Anchor = null;
		}

		public void AnchorHere(bool enable = true)
		{
			CFG.Anchor = enable? 
				new WayPoint(VSL.vessel.latitude, VSL.vessel.longitude) : null;
			Enable(enable);
		}

		public void Update()
		{
			if(!IsActive || CFG.Anchor == null) return;
			CFG.Anchor.Update(VSL.mainBody);
			//calculate direct distance
			var vdir = Vector3.ProjectOnPlane(CFG.Anchor.GetTransform().position-VSL.vessel.transform.position, VSL.Up);
			var distance = vdir.magnitude;
			vdir.Normalize();
			//tune the pid and update needed velocity
			AccelCorrection.Update(Mathf.Clamp(VSL.MaxThrust.magnitude*VSL.MinVSFtwr/VSL.M/ANC.MaxAccelF, 0.01f, 1)*
			                       Mathf.Clamp(VSL.MaxPitchRollAA/ANC.AngularAccelF, 0.01f, 1), 0.01f);
			pid.P   = ANC.DistancePID.P*AccelCorrection;
			pid.D   = ANC.DistancePID.D*(2-AccelCorrection);
			pid.Update(distance*ANC.DistanceF);
			//set needed velocity and starboard
			CFG.NeededHorVelocity = vdir*pid.Action;
			CFG.Starboard = VSL.GetStarboard(CFG.NeededHorVelocity);
//			Log("dist {0}, pid {1}, nHV {2}", distance, pid, CFG.NeededHorVelocity);//debug
		}

		#if DEBUG
		public void AnchorPointer()
		{
			if(CFG.Anchor == null) return;
			GLUtils.GLTriangleMap(new Vector3[] { VSL.wCoM-VSL.refT.right*0.1f, VSL.wCoM+VSL.refT.right*0.1f, CFG.Anchor.GetTransform().position }, Color.cyan);
		}

		public override void Reset()
		{
			base.Reset();
			RenderingManager.RemoveFromPostDrawQueue(1, AnchorPointer);
		}
		#endif
	}
}

