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
using AT_Utils;
using CommNet;

namespace ThrottleControlledAvionics
{
	[CareerPart]
	public class SquadControl : TCAModule
	{
//		public class Config : ModuleConfig
//		{
//			new public const string NODE_NAME = "SQD";
//		}
//		static Config SQD { get { return Globals.Instance.SQD; } }

		public SquadControl(ModuleTCA tca) : base(tca) {}

		public bool SquadMode;

        public override void Disable() {}

        public static bool IsCommReachable(ModuleTCA A, ModuleTCA B)
        {
            if(A == B) return true;
            //if KerbNet is not enabled, always reachable
            if(!CommNetScenario.CommNetEnabled) return true;
            //null checks
            if(A.vessel.Connection == null) return false;
            if(B == null || B.vessel == null || B.vessel.Connection == null) return false;
            //if it is locally controllable, it is also remotely controllable
            if(A.IsControllable) return true;
            //check KerbNet path
            var start = A.vessel.Connection.Comm;
            var end = B.vessel.Connection.Comm;
            var path = new CommPath();
            if(A.vessel.Connection.Comm.Net.FindPath(start, path, end)) return true;
            //check direct ship2ship connection
            var sqrDist = (start.precisePosition-end.precisePosition).sqrMagnitude;
            var offset = start.distanceOffset + end.distanceOffset;
            if(!offset.Equals(0))
            {
                offset = Math.Sqrt(sqrDist) + offset;
                sqrDist = offset > 0 ? offset * offset : 0;
            }
            return CommNetScenario.RangeModel
                .InRange(Math.Max(start.antennaTransmit.power, start.antennaRelay.power),
                         Math.Max(end.antennaTransmit.power, end.antennaRelay.power), 
                         sqrDist);
        }

        public static void UnpackVessel(Vessel from_vessel, Vessel vessel)
        {
            if(vessel.packed) 
            {
                var dist = (vessel.transform.position-from_vessel.transform.position).magnitude;
                var sit = vessel.vesselRanges.GetSituationRanges(vessel.situation);
                sit.pack = dist*1.5f;
                sit.unpack = dist*1.2f;
                vessel.GoOffRails();
            }
        }

		bool is_comm_reachable(ModuleTCA tca)
		{
            return IsCommReachable(TCA, tca);
        }

		void apply_to_others(Action<ModuleTCA> action)
		{
			if(TCA.CFG.Squad == 0 || !SquadMode) return;
			bool executed = false;
			for(int i = 0, num_vessels = FlightGlobals.Vessels.Count; i < num_vessels; i++)
			{
				var v = FlightGlobals.Vessels[i];
				if(v == null || v == VSL.vessel || !v.loaded) continue;
				var tca = ModuleTCA.EnabledTCA(v);
				if(tca == null || !tca.Available) continue;
				if(tca.CFG.Squad == 0 || tca.CFG.Squad != TCA.CFG.Squad) continue;
				if(!is_comm_reachable(tca)) continue;
				//try to reach packed vessels
                UnpackVessel(TCA.vessel, v);
				action(tca);
				executed = true;
			}
			if(executed) Message("Squad Action Executed");
		}

		public void Apply(Action<ModuleTCA> action)
		{
			if(TCA == null || action == null) return;
			action(TCA);
			apply_to_others(action);
		}

		public void ApplyCFG(Action<VesselConfig> action)
		{
			if(TCA == null || action == null) return;
			action(TCA.CFG);
			apply_to_others(tca => action(tca.CFG));
		}

		public void SyncCFG(Action<VesselConfig> action)
		{
			if(TCA == null || action == null) return;
			apply_to_others(tca => action(tca.CFG));
		}

		public void FollowMe()
		{
			Apply(tca => 
			{
				if(tca == TCA) return;
				tca.CFG.Nav.XOff();
				tca.vessel.targetObject = TCA.vessel;
				tca.CFG.Nav.On(Navigation.FollowTarget);
			});
		}

		public override void Draw()
		{
			Utils.ButtonSwitch("Squadron Mode", ref SquadMode, 
			                   "Control autopilot on all squadron vessels", 
			                   GUILayout.ExpandWidth(false));
			if(SquadMode) CFG.Squad = Utils.IntSelector(CFG.Squad, 1, tooltip: "Squad ID");
		}
	}
}

