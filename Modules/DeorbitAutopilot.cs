//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2016 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.
//
using System;
using UnityEngine;

namespace ThrottleControlledAvionics
{
//	[CareerPart]
	[RequireModules(typeof(ManeuverAutopilot))]
	public class DeorbitAutopilot : TCAModule
	{
		public class Config : TCAModule.ModuleConfig
		{
			[Persistent] public float StartOffset = 60f;  //sec
			[Persistent] public float PeR         = 0.9f; //of planet radius
			[Persistent] public float dVtol       = 0.01f; //of planet radius
		}
		static Config DEO { get { return TCAScenario.Globals.DEO; } }

		public DeorbitAutopilot(ModuleTCA tca) : base(tca) {}

		ManeuverAutopilot MAN;

		double Start = -1f;
//		double dVr = 0f;
//		double dVn = 0f;

		OrbitDBG new_orbit(Orbit old, Vector3d dV, double UT)
		{
			var obt = new OrbitDBG();
			var pos = old.getRelativePositionAtUT(UT);
			var vel = old.getOrbitalVelocityAtUT(UT)+dV;
			obt.UpdateFromStateVectors(pos, vel, old.referenceBody, UT);
			return obt;
		}

		double dV4Pe(Orbit old, double PeR, double UT)
		{
			var oldPeR = old.PeR;
			var vel = old.getOrbitalVelocityAtUT(UT);
			var up = oldPeR < PeR;
			var pg = up? 1 : -1;
			var velN = vel.normalized * pg;
			var min_dV = 0.0;
			var max_dV = 0.0;
			if(up)
			{
				max_dV = 1;
				while(new_orbit(old, velN*max_dV, UT).PeR < PeR)
				{ max_dV *= 2; if(max_dV > 100000) break; }
			}
			else max_dV = vel.magnitude;
			while(max_dV-min_dV > DEO.dVtol)
			{
				var dV = (max_dV+min_dV)/2;
				var nPe = new_orbit(old, velN*dV, UT).PeR;
				if(up && nPe > PeR || !up && nPe < PeR) max_dV = dV;
				else min_dV = dV;
			}
			return (max_dV+min_dV)/2*pg;
		}

		protected override void UpdateState()
		{
			base.UpdateState();
			IsActive = VSL.InOrbit;
		}

		public override void OnEnable(bool enabled)
		{
			base.OnEnable(enabled);
			if(enabled) Start = -1;
		}

		protected override void Update()
		{
			if(!IsActive || VSL.orbit.referenceBody == null) return;
			if(Start < 0) 
			{
				var dV = 0.0;
				var ttb = 0f;
				var offset = DEO.StartOffset;
				do
				{
					if(ttb > 0) offset = ttb+DEO.StartOffset;
					dV = dV4Pe(VSL.orbit, VSL.mainBody.Radius*DEO.PeR, VSL.Physics.UT+offset);
					ttb = MAN.TTB((float)Math.Abs(dV), 1)/2;
				} while(ttb > offset);

				Start = VSL.Physics.UT+offset;
				var node = VSL.vessel.patchedConicSolver.AddManeuverNode(Start);
				node.DeltaV = new Vector3d(0,0,dV);
				VSL.vessel.patchedConicSolver.UpdateFlightPlan();
			}
		}
	}
}

