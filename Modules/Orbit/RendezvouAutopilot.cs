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
using KSP;

namespace ThrottleControlledAvionics
{
//	[CareerPart]
	[RequireModules(typeof(ManeuverAutopilot),
	                typeof(MatchVelocityAutopilot),
	                typeof(AttitudeControl),
	                typeof(ThrottleControl))]
	public class RendezvouAutopilot : TargetedTrajectoryCalculator<RendezvousTrajectory, Vessel>
	{
		public new class Config : TCAModule.ModuleConfig
		{
			[Persistent] public float Dtol                = 100f;   //m
			[Persistent] public float StartOffset         = 60f;    //s
			[Persistent] public float DeltaApThreshold    = 1000f;  //m
			[Persistent] public float MaxTTR              = 3f;     //1/VesselOrbit.period
			[Persistent] public float MaxDeltaV           = 100f;   //m/s
			[Persistent] public float CorrectionStart     = 10000f; //m
			[Persistent] public float CorrectionOffset    = 20f;    //s
			[Persistent] public float CorrectionTimer     = 10f;    //s
			[Persistent] public float MaxApproachV        = 20f;    //parts
			[Persistent] public float ApproachVelF        = 0.01f;  //parts
			[Persistent] public float MaxInclinationDelta = 30;  //deg
		}
		static Config REN { get { return TCAScenario.Globals.REN; } }

		public RendezvouAutopilot(ModuleTCA tca) : base(tca) {}

		AttitudeControl ATC;
		ThrottleControl THR;

		enum Stage { None, Start, ToOrbit, StartOrbit, ComputeRendezvou, Rendezvou, MatchOrbits, Approach, Brake }
		Stage stage;

		double CurrentDistance = -1;
		RendezvousTrajectory CurrentTrajectory 
		{ get { return new RendezvousTrajectory(VSL, Vector3d.zero, VSL.Physics.UT, Target, REN.StartOffset); } }

		public override void Init()
		{
			base.Init();
			Dtol = REN.Dtol;
			CorrectionTimer.Period = REN.CorrectionTimer;
			CFG.AP2.AddHandler(this, Autopilot2.Rendezvou);
		}

		public void RendezvouCallback(Multiplexer.Command cmd)
		{
			reset();
			switch(cmd)
			{
			case Multiplexer.Command.Resume:
				break;

			case Multiplexer.Command.On:
				if(!setup()) 
				{
					CFG.AP2.Off();
					return;
				}
				stage = Stage.Start;
				goto case Multiplexer.Command.Resume;

			case Multiplexer.Command.Off:
				break;
			}
		}

		protected override bool check_target()
		{
			if(!base.check_target()) return false;
			var tVSL = VSL.TargetVessel;
			if(tVSL == null) 
			{
				Status("Target should be a vessel");
				return false;
			}
			if(tVSL.LandedOrSplashed)
			{
				Status("Target vessel is landed");
				return false;
			}
			if(Math.Abs(tVSL.orbit.inclination-VesselOrbit.inclination) > REN.MaxInclinationDelta)
			{
				Status("Target orbit plane is tilted more than {0:F}° with respect to ours.\n" +
				       "You need to change orbit plane before the rendezvou maneuver.", REN.MaxInclinationDelta);
				return false;
			}
			return true;
		}

		protected override void setup_target()
		{ 
			Target = VSL.TargetVessel; 
			SetTarget(Target);
		}

		protected RendezvousTrajectory rendezvou_orbit(RendezvousTrajectory old, ref Vector3d NodeDeltaV)
		{
			double StartUT;
			Vector3d dV = Vector3d.zero;
			if(old != null) 
			{
				var distF = Utils.ClampH(old.DistanceToTarget/REN.CorrectionStart, 1);
				StartUT = AngleDelta2StartUT(old, old.DeltaTA, REN.StartOffset, VesselOrbit.period/4, 
				                             VesselOrbit.period*distF);
				NodeDeltaV += RadiusCorrection(old);
				NodeDeltaV += PlaneCorrection(old)*(1-distF*0.99);
				NodeDeltaV += AoPCorrection(old, -old.DeltaTA)*Math.Sign(Target.orbit.period-VesselOrbit.period);
				dV = Node2OrbitDeltaV(StartUT, NodeDeltaV);
				LogF("\nRO: dV {}, dUT {}", NodeDeltaV, StartUT-old.StartUT);//debug
				CSV(old.TimeToStart, old.TimeToTarget, old.DeltaTA, old.DeltaFi, old.DeltaR, old.DistanceToTarget, NodeDeltaV.x, NodeDeltaV.y, NodeDeltaV.z);//debug
			}
			else 
			{
				var start_offset = VesselOrbit.period/3;
				double alpha, resonanse;
				StartUT = VSL.Physics.UT+
					Utils.ClampL(TimeToResonance(VesselOrbit, Target.orbit, VSL.Physics.UT+REN.StartOffset, out resonanse, out alpha)*VesselOrbit.period
					             -start_offset, REN.StartOffset);
				double AtTargetUT;
				TrajectoryCalculator.ClosestApproach(VesselOrbit, Target.orbit, StartUT, out AtTargetUT, REN.StartOffset);
//				var ini = new RendezvousTrajectory(VSL, Vector3d.zero, StartUT, Target, REN.StartOffset);
//				NodeDeltaV += RadiusCorrection(ini);
//				ini = new RendezvousTrajectory(VSL, Node2OrbitDeltaV(StartUT, NodeDeltaV), StartUT, Target, REN.StartOffset);
				LogF("First Time TimeToStart: {}, Correction {}", StartUT-VSL.Physics.UT, AtTargetUT-start_offset-StartUT);//debug
				StartUT = AtTargetUT-start_offset;
			}
			return new RendezvousTrajectory(VSL, dV, StartUT, Target, REN.StartOffset);
		}

		protected RendezvousTrajectory orbit_correction(RendezvousTrajectory old, ref Vector3d NodeDeltaV)
		{
			double StartUT = VSL.Physics.UT+REN.CorrectionOffset;
			Vector3d dV = Vector3d.zero;
			if(old != null) 
			{
				NodeDeltaV += RadiusCorrection(old);
				NodeDeltaV += PlaneCorrection(old);
				NodeDeltaV += AoPCorrection(old, -old.DeltaTA);
				dV = Node2OrbitDeltaV(StartUT, NodeDeltaV);
				LogF("\nOC: dV {}, dUT {}", NodeDeltaV, StartUT-old.StartUT);//debug
				CSV(old.TimeToStart, old.TimeToTarget, old.DeltaTA, old.DeltaFi, old.DeltaR, old.DistanceToTarget, NodeDeltaV.x, NodeDeltaV.y, NodeDeltaV.z);//debug
			} 
			return new RendezvousTrajectory(VSL, dV, StartUT, Target);
		}
			
		void compute_rendezvou_trajectory()
		{
			trajectory = null;
			stage = Stage.ComputeRendezvou;
			Vector3d NodeDeltaV = Vector3d.zero;
			setup_calculation(t => rendezvou_orbit(t, ref NodeDeltaV));
		}

		void correct_trajectory()
		{
			trajectory = null;
			stage = Stage.ComputeRendezvou;
			Vector3d NodeDeltaV = Vector3d.zero;
			setup_calculation(t => orbit_correction(t, ref NodeDeltaV));
		}

		void start_orbit()
		{
			var dV = Vector3d.zero;
			var StartUT = VSL.Physics.UT+REN.StartOffset;
			var old = VesselOrbit;
			if(VesselOrbit.PeR < MinPeR) 
			{
				if(ApAhead) StartUT = VSL.Physics.UT+VesselOrbit.timeToAp;
				dV += dV4C(old, hV(StartUT), StartUT);
				old = NewOrbit(old, dV, StartUT);
			}
			LogF("\nstart orbit\n{}", Utils.formatOrbit(old));//debug
			dV += dV4MinTTR(old, Target.orbit, REN.MaxTTR, REN.MaxDeltaV, MinPeR, StartUT);
			if(!dV.IsZero())
			{
				add_node(dV, StartUT);
				CFG.AP1.On(Autopilot1.Maneuver);
			}
			stage = Stage.StartOrbit;
		}

		void to_orbit()
		{
			stage = Stage.ToOrbit;
		}

		void match_orbits()
		{
			SetTarget(Target);
			update_trajectory();
			add_target_node();
			CFG.AP1.On(Autopilot1.Maneuver);
			stage = Stage.MatchOrbits;
			MAN.MinDeltaV = 1;
		}

		void approach()
		{
			CFG.AT.On(Attitude.Target);
			stage = Stage.Approach;
		}

		void brake()
		{
			SetTarget(Target);
			stage = Stage.Brake;
			CFG.AP1.On(Autopilot1.MatchVelNear);	
		}

		void update_trajectory(bool update_distance=true)
		{
			if(trajectory == null) trajectory = CurrentTrajectory;
			else trajectory.UpdateOrbit(VesselOrbit);
			if(update_distance) CurrentDistance = trajectory.DistanceToTarget;
		}

		protected override void reset()
		{
			base.reset();
			stage = Stage.None;
			CurrentDistance = -1;
			CFG.AP1.Off();
		}

		protected override void Update()
		{
			if(Target == null || VSL.orbit.referenceBody == null) return;
			switch(stage)
			{
			case Stage.Start:
				if(VSL.InOrbit && 
				   VesselOrbit.ApR > MinPeR &&
				   VesselOrbit.radius > MinPeR) start_orbit();
				else to_orbit();
				break;
			case Stage.ToOrbit:
				//FIXME: this is just for testing
				var dApA = Math.Max(MinPeR, Target.orbit.PeR)-VesselOrbit.ApR;
				if(dApA > 0)
				{
					Status("<color=red>Not Implemented</color>");
					CFG.AT.OnIfNot(Attitude.Prograde);
					if(ATC.AttitudeError > 1) break;
					CFG.DisableVSC();
					THR.Throttle = (float)Utils.Clamp(dApA/REN.DeltaApThreshold, 0, 1);
					break;
				}
				THR.Throttle = 0;
				start_orbit();
				break;
			case Stage.StartOrbit:
				if(CFG.AP1[Autopilot1.Maneuver]) break;
				CurrentDistance = -1;
				update_trajectory(false);
				if(trajectory.DistanceToTarget < REN.CorrectionStart) stage = Stage.Rendezvou;
				else compute_rendezvou_trajectory();
				break;
			case Stage.ComputeRendezvou:
				if(!trajectory_computed()) break;
				if(trajectory.ManeuverDeltaV.magnitude > GLB.THR.MinDeltaV*2 &&
				   trajectory.DistanceToTarget < REN.CorrectionStart &&
				   (CurrentDistance < 0 || trajectory.DistanceToTarget < CurrentDistance))
				{
					CurrentDistance = trajectory.DistanceToTarget;
					add_trajectory_node();
					CFG.AP1.On(Autopilot1.Maneuver);
					stage = Stage.Rendezvou;
				}
				else if(trajectory.DistanceToTarget < REN.CorrectionStart) match_orbits();
				else 
				{
					Status("<color=red>Failed to compute rendezvou trajectory.\nPlease, try again.</color>");
					CFG.AP2.Off();
					return;
				}
				break;
			case Stage.Rendezvou:
				if(CFG.AP1[Autopilot1.Maneuver]) break;
				if(!CorrectionTimer.Check && CurrentDistance >= 0) break;
				update_trajectory();
				if(CurrentDistance > REN.CorrectionStart) start_orbit();
				else if(CurrentDistance > REN.Dtol*2) correct_trajectory();
				else match_orbits();
				break;
			case Stage.MatchOrbits:
				if(CFG.AP1[Autopilot1.Maneuver]) break;
				update_trajectory();
				if(CurrentDistance > REN.CorrectionStart) start_orbit();
				else if(CurrentDistance > REN.CorrectionStart/2) correct_trajectory();
				else approach();
				break;
			case Stage.Approach:
				if(ATC.AttitudeError > 1) break;
				var dP = Target.orbit.pos-VesselOrbit.pos;
				var dPm = dP.magnitude;
				var dV = Vector3d.Dot(VesselOrbit.vel-Target.orbit.vel, dP/dPm);
				var nV = Utils.Clamp(dPm*REN.ApproachVelF, 1, REN.MaxApproachV);
				if(dV < nV) THR.DeltaV = (float)(nV-dV);
				else brake();
				break;
			case Stage.Brake:
				if(CFG.AP1[Autopilot1.MatchVelNear]) break;
				if((VSL.Physics.wCoM-Target.CurrentCoM).magnitude-VSL.Geometry.R > REN.Dtol)
				{ approach(); break; }
				CFG.AP2.Off();
				break;
			}
		}

		public override void Draw()
		{
			if(computing) 
				GUILayout.Label("Computing...", Styles.grey, GUILayout.ExpandWidth(false));
			else if(VSL.TargetVessel == null)
				GUILayout.Label(new GUIContent("Rendezvou", "No target"),
				                Styles.grey, GUILayout.ExpandWidth(false));
			else if(Utils.ButtonSwitch("Rendezvou", CFG.AP2[Autopilot2.Rendezvou],
			                           "Compute and perform a rendezvou maneuver, then brake near the target.", 
			                           GUILayout.ExpandWidth(false)))
				CFG.AP2.XToggle(Autopilot2.Rendezvou);
		}
	}
}

