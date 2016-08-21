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

namespace ThrottleControlledAvionics
{
	[CareerPart]
	[RequireModules(typeof(ManeuverAutopilot),
	                typeof(MatchVelocityAutopilot),
	                typeof(AttitudeControl),
	                typeof(ThrottleControl),
	                typeof(TranslationControl),
	                typeof(BearingControl),
	                typeof(TimeWarpControl))]
	public class RendezvousAutopilot : TargetedTrajectoryCalculator<RendezvousTrajectory>
	{
		public new class Config : TCAModule.ModuleConfig
		{
			[Persistent] public float Dtol                = 100f;   //m
			[Persistent] public float MaxTTR              = 3f;     //1/VesselOrbit.period
			[Persistent] public float MaxDeltaV           = 100f;   //m/s
			[Persistent] public float CorrectionStart     = 10000f; //m
			[Persistent] public float CorrectionOffset    = 20f;    //s
			[Persistent] public float CorrectionTimer     = 10f;    //s
			[Persistent] public float ApproachThreshold   = 500f;   //m
			[Persistent] public float MaxApproachV        = 20f;    //parts
			[Persistent] public float ApproachVelF        = 0.01f;  //parts
			[Persistent] public float MaxInclinationDelta = 30;     //deg
		}
		static Config REN { get { return Globals.Instance.REN; } }

		public RendezvousAutopilot(ModuleTCA tca) : base(tca) {}

		ThrottleControl THR;

		protected override RendezvousTrajectory CurrentTrajectory
		{ get { return new RendezvousTrajectory(VSL, Vector3d.zero, VSL.Physics.UT, CFG.Target, MinPeR); } }

		public enum Stage { None, Start, Launch, ToOrbit, StartOrbit, ComputeRendezvou, Rendezvou, MatchOrbits, Approach, Brake }
		[Persistent] public Stage stage;
		[Persistent] public Vector3 ToOrbitTarget;

		MinimumD MinDist = new MinimumD();
		ToOrbitExecutor ToOrbit;
		Vector3d ToOrbitIniApV;

		double CurrentDistance = -1;
		Orbit TargetOrbit { get { return CFG.Target.GetOrbit(); } }
		Vessel TargetVessel { get { return CFG.Target.GetVessel(); } }

		public override void Init()
		{
			base.Init();
			Dtol = REN.Dtol;
			CorrectionTimer.Period = REN.CorrectionTimer;
			CFG.AP2.AddHandler(this, Autopilot2.Rendezvous);
		}

		public override void Save(ConfigNode node)
		{
			if(ToOrbit != null) ToOrbitTarget = ToOrbit.Target;
			base.Save(node);
		}

		void resume_to_orbit()
		{
			ToOrbit = new ToOrbitExecutor(TCA);
			ToOrbit.Target = ToOrbitTarget;
		}

		public void RendezvousCallback(Multiplexer.Command cmd)
		{
			switch(cmd)
			{
			case Multiplexer.Command.Resume:
//				LogFST("Resuming: stage {}", stage);//debug
				if(!check_patched_conics()) return;
				NeedRadarWhenMooving();
				switch(stage)
				{
				case Stage.None:
				case Stage.ComputeRendezvou:
					stage = Stage.Start;
					break;
				case Stage.Launch:
					if(VSL.LandedOrSplashed) stage = Stage.Start;
					else resume_to_orbit();
					break;
				case Stage.ToOrbit:
					resume_to_orbit();
					break;
				}
				if(VSL.HasManeuverNode) 
					CFG.AP1.OnIfNot(Autopilot1.Maneuver);
				break;

			case Multiplexer.Command.On:
				reset();
				if(setup()) 
				{
					stage = Stage.Start;
					goto case Multiplexer.Command.Resume;
				}
				CFG.AP2.Off();
				return;

			case Multiplexer.Command.Off:
				UnregisterFrom<Radar>();
				SetTarget(); 
				reset();
				break;
			}
		}

		protected override bool check_target()
		{
			if(!base.check_target()) return false;
			if(TargetVessel == null) 
			{
				Status("yellow", "Target should be a vessel");
				return false;
			}
			if(TargetVessel.LandedOrSplashed)
			{
				Status("yellow", "Target vessel is landed");
				return false;
			}
			if(Math.Abs(TargetOrbit.inclination-VesselOrbit.inclination) > REN.MaxInclinationDelta)
			{
				Status("yellow", "Target orbit plane is tilted more than {0:F}° with respect to ours.\n" +
				       "You need to change orbit plane before the rendezvou maneuver.", REN.MaxInclinationDelta);
				return false;
			}
			return true;
		}

		RendezvousTrajectory new_trajectory(double StartUT, double transfer_time)
		{
			var solver = new LambertSolver(VesselOrbit, TargetOrbit.getRelativePositionAtUT(StartUT+transfer_time), StartUT);
			var dV = solver.dV4Transfer(transfer_time);
			return new RendezvousTrajectory(VSL, dV, StartUT, CFG.Target, MinPeR, transfer_time);
		}

		protected RendezvousTrajectory orbit_correction(RendezvousTrajectory old, RendezvousTrajectory best, ref double dT)
		{ return orbit_correction(old, best, VSL.Physics.UT + REN.CorrectionOffset, ref dT); }

		protected RendezvousTrajectory orbit_correction(RendezvousTrajectory old, RendezvousTrajectory best, double StartUT, ref double dT)
		{
			double transfer_time;
			if(VSL.Physics.UT+REN.CorrectionOffset > StartUT)
				StartUT = VSL.Physics.UT+REN.CorrectionOffset;
			if(old != null) 
			{
				transfer_time = old.TransferTime+dT;
				if(transfer_time > TargetOrbit.period ||
				   transfer_time < TRJ.ManeuverOffset ||
				   old.ManeuverDeltaV.sqrMagnitude > best.ManeuverDeltaV.sqrMagnitude &&
				   !old.KillerOrbit && !best.KillerOrbit)
				{
					dT /= -2.1;
					transfer_time = best.TransferTime+dT;
				}
			}
			else transfer_time = TRJ.ManeuverOffset;
			return new_trajectory(StartUT, transfer_time);
		}

		bool trajectories_converging(RendezvousTrajectory cur, RendezvousTrajectory best)
		{
			return cur == best ||
				target_is_far(cur, best) || 
				Math.Abs(cur.ManeuverDeltaV.sqrMagnitude-best.ManeuverDeltaV.sqrMagnitude) > 1;
		}

		protected override bool trajectory_is_better(RendezvousTrajectory first, RendezvousTrajectory second)
		{ return !first.KillerOrbit && base.trajectory_is_better(first, second); }

		protected override void setup_calculation(NextTrajectory next)
		{
			next_trajectory = next;
			end_predicate = trajectories_converging;
			better_predicate = trajectory_is_better;
		}

		void compute_rendezvou_trajectory()
		{
			Status("Performing rendezvous maneuver...");
			trajectory = null;
			stage = Stage.ComputeRendezvou;
			var transfer_time = VesselOrbit.period/4;
			var StartUT = VSL.Physics.UT+
				Utils.ClampL(TimeToResonance(VesselOrbit, TargetOrbit, VSL.Physics.UT+TRJ.ManeuverOffset)
				             *VesselOrbit.period-transfer_time, TRJ.ManeuverOffset);
			double dT = TargetOrbit.period/4;
//			LogF("transfer time: {}, StartT {}, TTR {}", 
//			     transfer_time, StartUT-VSL.Physics.UT, 
//			     TimeToResonance(VesselOrbit, TargetOrbit, VSL.Physics.UT+TRJ.ManeuverOffset)*VesselOrbit.period);//debug
			setup_calculation((o, b) => orbit_correction(o, b, StartUT, ref dT));
		}

		protected override void fine_tune_approach()
		{
			Status("Fine-tuning nearest approach...");
			trajectory = null;
			stage = Stage.ComputeRendezvou;
			double dT = TargetOrbit.period/4;
			setup_calculation((o, b) => orbit_correction(o, b, ref dT));
		}

		protected void compute_start_orbit(double StartUT)
		{
			Status("Achiving rendezvous orbit...");
			trajectory = null;
			stage = Stage.ComputeRendezvou;
			double dT = TargetOrbit.period/4;
			setup_calculation((o, b) => orbit_correction(o, b, StartUT, ref dT));
		}

		void start_orbit()
		{
			ToOrbit = null;
			var dV = Vector3d.zero;
			var old = VesselOrbit;
			var StartUT = VSL.Physics.UT+REN.CorrectionOffset;
			CFG.BR.OffIfOn(BearingMode.Auto);
			if(VesselOrbit.PeR < MinPeR) 
			{
				update_trajectory();
				StartUT = Math.Min(trajectory.AtTargetUT, VSL.Physics.UT+(VesselOrbit.ApAhead()? VesselOrbit.timeToAp : REN.CorrectionOffset));
				if(trajectory.DistanceToTarget < REN.ApproachThreshold*2 && StartUT.Equals(trajectory.AtTargetUT)) 
				{ //approach is close enough to directly match orbits
					match_orbits(); 
					return; 
				}
				var transfer_time = Utils.ClampL(TargetOrbit.period*(0.25-AngleDelta(VesselOrbit, TargetOrbit, StartUT)/360), 1);
				var solver = new LambertSolver(VesselOrbit, TargetOrbit.getRelativePositionAtUT(StartUT+transfer_time), StartUT);
				dV = solver.dV4Transfer(transfer_time);
				var trj = new RendezvousTrajectory(VSL, dV, StartUT, CFG.Target, MinPeR, transfer_time);
				if(!dV.IsZero() && !trj.KillerOrbit)
				{ //approach orbit is possible
					compute_start_orbit(StartUT);
					return;
				}
				//starting from circular orbit and proceeding to TTR fitting...
				StartUT = VesselOrbit.ApAhead()? VSL.Physics.UT+VesselOrbit.timeToAp : VSL.Physics.UT+REN.CorrectionOffset;
				dV = dV4C(old, hV(StartUT), StartUT);
				old = NewOrbit(old, dV, StartUT);
			}
			dV += dV4TTR(old, TargetOrbit, REN.MaxTTR, REN.MaxDeltaV, MinPeR, StartUT);
			if(!dV.IsZero())
			{
				if(old == VesselOrbit) Status("Correcting orbit for resonance...");
				else Status("Achiving starting orbit...");
				add_node(dV, StartUT);
				CFG.AP1.On(Autopilot1.Maneuver);
			}
			stage = Stage.StartOrbit;
		}

		void to_orbit()
		{
			if(!LiftoffPossible) return;
			//calculate target vector
			var ApR = Math.Max(MinPeR, (TargetOrbit.PeR+TargetOrbit.ApR)/2);
			var hVdir = Vector3d.Cross(TargetOrbit.GetOrbitNormal(), VesselOrbit.pos).normalized;
			var ascO = AscendingOrbit(ApR, hVdir, GLB.ORB.LaunchTangentK);
			//tune target vector
			ToOrbit = new ToOrbitExecutor(TCA);
			ToOrbit.LaunchUT = VSL.Physics.UT;
			ToOrbit.ApAUT    = VSL.Physics.UT+ascO.timeToAp;
			ToOrbit.Target = ToOrbitIniApV = ascO.getRelativePositionAtUT(ToOrbit.ApAUT);
			if(VSL.LandedOrSplashed)
			{
				double TTR;
				do { TTR = correct_launch(); } 
				while(Math.Abs(TTR) > 1);
			}
			//setup launch
			CFG.DisableVSC();
			stage = VSL.LandedOrSplashed? Stage.Launch : Stage.ToOrbit;
		}

		double correct_launch(bool allow_wait = true)
		{
			var TTR = AngleDelta(TargetOrbit, ToOrbit.Target, ToOrbit.ApAUT)/360*TargetOrbit.period;
			ToOrbit.LaunchUT += TTR;
			if(allow_wait && ToOrbit.LaunchUT-VSL.Physics.UT <= 0) ToOrbit.LaunchUT += TargetOrbit.period;
			ToOrbit.ApAUT = ToOrbit.LaunchUT+
				AtmoSim.FromSurfaceTTA(VSL, ToOrbit.TargetR-Body.Radius, 
				                       ToOrbit.ArcDistance, GLB.ORB.GTurnCurve, 
				                       Vector3d.Dot(SurfaceVel, Vector3d.Exclude(VesselOrbit.pos, ToOrbit.Target-VesselOrbit.pos).normalized));
			ToOrbit.Target = QuaternionD.AngleAxis((VSL.Physics.UT-ToOrbit.LaunchUT)/Body.rotationPeriod*360, Body.angularVelocity.xzy)*
				ToOrbitIniApV.normalized*TargetOrbit.getRelativePositionAtUT(ToOrbit.ApAUT).magnitude;
//			LogF("TTR: {}, LaunchT {}, ApAT {}", TTR, ToOrbit.LaunchUT-VSL.Physics.UT, ToOrbit.ApAUT-ToOrbit.LaunchUT);//debug
			return TTR;
		}

		void match_orbits()
		{
			SetTarget(CFG.Target);
			update_trajectory();
			add_target_node();
			CFG.AP1.On(Autopilot1.Maneuver);
			stage = Stage.MatchOrbits;
			MAN.MinDeltaV = 0.5f;
		}

		void approach()
		{
			CFG.AT.On(Attitude.Target);
			stage = Stage.Approach;
		}

		void brake()
		{
			SetTarget(CFG.Target);
			stage = Stage.Brake;
			var dV = VesselOrbit.vel-TargetOrbit.vel;
			var dVm = dV.magnitude;
			var dist = TargetVessel.CurrentCoM-VSL.Physics.wCoM;
			var distm = dist.magnitude;
			if(distm < REN.Dtol && dVm < GLB.THR.MinDeltaV*2) return;
			if(distm > REN.Dtol && Vector3.Dot(dist, dV.xzy) > 0)
				CFG.AP1.On(Autopilot1.MatchVelNear);
			else CFG.AP1.On(Autopilot1.MatchVel);
		}

		protected override void update_trajectory()
		{
			base.update_trajectory();
			CurrentDistance = trajectory.DistanceToTarget;
		}

		protected override void reset()
		{
			base.reset();
			stage = Stage.None;
			CurrentDistance = -1;
			CFG.AP1.Off();
		}

		protected override void UpdateState()
		{
			base.UpdateState();
			IsActive &= CFG.AP2[Autopilot2.Rendezvous];
			ControlsActive &= IsActive || VSL.TargetVessel != null;
		}

//		double startAlpha; //debug
//		double startUT = -1; //debug
//		void log_flight() //debug
//		{
//			var target = ToOrbit != null? ToOrbit.Target : VesselOrbit.getRelativePositionAtUT(VSL.Physics.UT+VesselOrbit.timeToAp);
//			var alpha = Utils.ProjectionAngle(VesselOrbit.pos, target, target-VesselOrbit.pos) * Mathf.Deg2Rad;
//			var arc = (startAlpha-alpha)*VesselOrbit.radius;
//			CSV(VSL.Physics.UT-startUT, VSL.VerticalSpeed.Absolute, Vector3d.Exclude(VesselOrbit.pos, VesselOrbit.vel).magnitude, 
//			    arc, VesselOrbit.radius-Body.Radius, 
//			    VSL.Engines.Thrust.magnitude,
//			    VSL.Physics.M, 90-Vector3d.Angle(VesselOrbit.vel.xzy, VSL.Physics.Up));//debug
//		}

		protected override void Update()
		{
			if(!IsActive) { CFG.AP2.OffIfOn(Autopilot2.Rendezvous); return; }
			switch(stage)
			{
			case Stage.Start:
				if(VSL.InOrbit && 
				   VesselOrbit.ApR > MinPeR &&
				   VesselOrbit.radius > MinPeR)
				{
					update_trajectory();
					if(CurrentDistance > REN.CorrectionStart) start_orbit();
					else fine_tune_approach();
				}
				else to_orbit();
				break;
			case Stage.Launch:
				if(ToOrbit.LaunchUT > VSL.Physics.UT) 
				{
					Status("Waiting for launch window...");
					correct_launch(false); //FIXME: sometimes AtmoSim returns inconsistent TTA which results in jumps of the Countdown.
					VSL.Info.Countdown = ToOrbit.LaunchUT-VSL.Physics.UT;
					VSL.Controls.WarpToTime = ToOrbit.LaunchUT;
					break;
				}
//				if(startUT < 0) //debug
//				{
//					startAlpha = Utils.ProjectionAngle(VesselOrbit.pos, ToOrbit.Target, ToOrbit.Target-VesselOrbit.pos) * Mathf.Deg2Rad;
//					startUT = VSL.Physics.UT;
//				}
//				log_flight();//debug
				if(ToOrbit.Liftoff()) break;
				stage = Stage.ToOrbit;
				MinDist.Reset();
				break;
			case Stage.ToOrbit:
//				log_flight();//debug
				if(ToOrbit.GravityTurn(TRJ.ManeuverOffset, GLB.ORB.GTurnCurve, GLB.ORB.Dist2VelF, REN.Dtol))
				{
					if(ToOrbit.dApA < REN.Dtol)
					{
						update_trajectory();
						MinDist.Update(CurrentDistance);
						if(MinDist) start_orbit(); 
					}
				} else start_orbit();
				break;
			case Stage.StartOrbit:
//				log_flight();//debug
				if(CFG.AP1[Autopilot1.Maneuver]) break;
				update_trajectory();
				CurrentDistance = -1;
				if(trajectory.DistanceToTarget < REN.CorrectionStart ||
				   (trajectory.TransferTime+trajectory.TimeToStart)/VesselOrbit.period > REN.MaxTTR) 
					stage = Stage.Rendezvou;
				else compute_rendezvou_trajectory();
				break;
			case Stage.ComputeRendezvou:
//				log_flight();//debug
				if(!trajectory_computed()) break;
				if(trajectory.ManeuverDeltaV.magnitude > GLB.THR.MinDeltaV*5 &&
				   trajectory.DistanceToTarget < REN.CorrectionStart &&
				   (CurrentDistance < 0 || trajectory.DistanceToTarget < CurrentDistance))
				{
					CorrectionTimer.Start();
					CurrentDistance = trajectory.DistanceToTarget;
					add_trajectory_node();
					CFG.AP1.On(Autopilot1.Maneuver);
					stage = Stage.Rendezvou;
				}
				else if(trajectory.DistanceToTarget < REN.CorrectionStart) match_orbits();
				else 
				{
					Status("red", "Failed to compute rendezvou trajectory.\nPlease, try again.");
					CFG.AP2.Off();
					return;
				}
				break;
			case Stage.Rendezvou:
//				log_flight();//debug
				if(CFG.AP1[Autopilot1.Maneuver]) break;
				if(!CorrectionTimer.TimePassed && CurrentDistance >= 0) break;
				CorrectionTimer.Reset();
				update_trajectory();
				if(CurrentDistance < REN.ApproachThreshold) match_orbits();
				else fine_tune_approach();
				break;
			case Stage.MatchOrbits:
//				log_flight();//debug
				Status("Matching orbits at nearest approach...");
				if(CFG.AP1[Autopilot1.Maneuver]) break;
				update_trajectory();
				var dist = Utils.ClampL((TargetOrbit.pos-VesselOrbit.pos).magnitude-VSL.Geometry.R-TargetVessel.Radius(), 0);
				if(dist > REN.CorrectionStart) start_orbit();
				else if(dist > REN.CorrectionStart/4) fine_tune_approach();
				else if(dist > REN.Dtol) approach();
				else brake();
				break;
			case Stage.Approach:
				Status("Approaching...");
				var dP = TargetOrbit.pos-VesselOrbit.pos;
				var dPm = dP.magnitude;
				if(dPm - VSL.Geometry.R - TargetVessel.Radius() < REN.Dtol) 
				{ brake(); break; }
				if(VSL.Controls.AttitudeError > 1) break;
				var dV = Vector3d.Dot(VesselOrbit.vel-TargetOrbit.vel, dP/dPm);
				var nV = Utils.Clamp(dPm*REN.ApproachVelF, 1, REN.MaxApproachV);
				if(dV+GLB.THR.MinDeltaV < nV) 
				{
					VSL.Engines.ActivateEngines();
					THR.DeltaV = (float)(nV-dV);
				}
				else brake();
				break;
			case Stage.Brake:
				Status("Braking near target...");
				if(CFG.AP1[Autopilot1.MatchVelNear]) break;
				if(CFG.AP1[Autopilot1.MatchVel])
				{
					if((TargetOrbit.vel-VesselOrbit.vel).magnitude > GLB.THR.MinDeltaV) break;
					CFG.AP1.Off();
					THR.Throttle = 0;
				}
				if((VSL.Physics.wCoM-TargetVessel.CurrentCoM).magnitude-VSL.Geometry.R-TargetVessel.Radius() > REN.Dtol)
				{ approach(); break; }
				CFG.AP2.Off();
				CFG.AT.OnIfNot(Attitude.KillRotation);
				ClearStatus();
				break;
			}
		}

		public override void Draw()
		{
			#if DEBUG
//			if(CFG.Target != null && Body != null && TargetOrbit != null)
//			{
//				if(ToOrbit != null)
//				{
//					Utils.GLVec(Body.position, ToOrbit.Target.xzy, Color.green);
//					Utils.GLVec(Body.position, Vector3d.Cross(VesselOrbit.pos, ToOrbit.Target).normalized.xzy*Body.Radius*1.1, Color.red);
//					Utils.GLVec(Body.position, TargetOrbit.getRelativePositionAtUT(ToOrbit.ApAUT).xzy, Color.yellow);
//				}
//				else Utils.GLVec(Body.position, TargetOrbit.getRelativePositionAtUT(VSL.Physics.UT+VesselOrbit.timeToAp).xzy, Color.yellow);
//				Utils.GLVec(Body.position, VesselOrbit.getRelativePositionAtUT(VSL.Physics.UT+VesselOrbit.timeToAp).xzy, Color.magenta);
//				Utils.GLVec(Body.position, VesselOrbit.GetOrbitNormal().normalized.xzy*Body.Radius*1.1, Color.cyan);
//			}
			#endif
			if(ControlsActive)
			{
				if(computing) 
				{
					if(GUILayout.Button(new GUIContent("Rendezvous", "Computing trajectory. Push to cancel."), 
					                    Styles.inactive_button, GUILayout.ExpandWidth(false)))
						CFG.AP2.Off();
//					#if DEBUG
//					if(current != null)
//					{
//						Utils.GLVec(Body.position, current.AtTargetPos.xzy, Color.green);//debug
//						Utils.GLVec(Body.position, current.TargetPos.xzy, Color.magenta);//debug
//						Utils.GLVec(Body.position+current.StartPos.xzy, current.ManeuverDeltaV.normalized.xzy*Body.Radius/4, Color.yellow);//debug
//						Utils.GLLine(Body.position+current.StartPos.xzy, Body.position+current.TargetPos.xzy, Color.cyan);//debug
//					}
//					#endif
				}
				else if(Utils.ButtonSwitch("Rendezvous", CFG.AP2[Autopilot2.Rendezvous],
				                           "Compute and perform a rendezvous maneuver, then brake near the target.", 
				                           GUILayout.ExpandWidth(false)))
					CFG.AP2.XToggle(Autopilot2.Rendezvous);
			}
			else GUILayout.Label(new GUIContent("Rendezvous", "Compute and perform a rendezvous maneuver, then brake near the target."), 
			                     Styles.inactive_button, GUILayout.ExpandWidth(false));
		}
	}
}
