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
	[RequireModules(typeof(ManeuverAutopilot),
	                typeof(MatchVelocityAutopilot),
	                typeof(AttitudeControl),
	                typeof(ThrottleControl),
	                typeof(TranslationControl),
	                typeof(BearingControl),
	                typeof(TimeWarpControl))]
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
			[Persistent] public float GTurnCurve          = 60;
			[Persistent] public float LaunchDeltaVf       = 3;
			[Persistent] public float LaunchTangentK      = 1f;
			[Persistent] public float RendezvouCorrection = 0.8f;
		}
		static Config REN { get { return TCAScenario.Globals.REN; } }

		public RendezvouAutopilot(ModuleTCA tca) : base(tca) {}

		AttitudeControl ATC;
		ThrottleControl THR;
		TranslationControl TRA;
		BearingControl BRC;
		TimeWarpControl WRP;

		enum Stage { None, Start, Launch, ToOrbit, StartOrbit, ComputeRendezvou, Rendezvou, MatchOrbits, Approach, Brake }
		Stage stage;

		double CurrentDistance = -1;
		RendezvousTrajectory CurrentTrajectory 
		{ get { return new RendezvousTrajectory(VSL, Vector3d.zero, VSL.Physics.UT, Target, REN.StartOffset); } }

		double ApAUT, TTA, MinAccelTime;
		double LaunchUT  = -1;
		double LaunchApR = -1;
		Vector3d TargetApV, ApV;
		FuzzyThreshold<float> ToOrbitDeltaV = new FuzzyThreshold<float>(5, 1);
		SingleAction GearAction = new SingleAction();

		public override void Init()
		{
			base.Init();
			Dtol = REN.Dtol;
			CorrectionTimer.Period = REN.CorrectionTimer;
			CFG.AP2.AddHandler(this, Autopilot2.Rendezvou);
			GearAction.action = () => VSL.vessel.ActionGroups.SetGroup(KSPActionGroup.Gear, false);
			#if DEBUG
			RenderingManager.AddToPostDrawQueue(1, RadarBeam);
			#endif
		}

		#if DEBUG
		public void RadarBeam()
		{
			if(VSL == null || VSL.vessel == null || VSL.refT == null || !CFG.AP2[Autopilot2.Rendezvou]) return;
			if(stage == Stage.ToOrbit)
			{
				GLUtils.GLVec(VSL.Physics.wCoM, tNorm.normalized.xzy*12, Color.green);
				GLUtils.GLVec(VSL.Physics.wCoM, norm.normalized.xzy*10, Color.yellow);
				GLUtils.GLVec(VSL.Physics.wCoM, hVdir.normalized.xzy*10, Color.red);
				GLUtils.GLVec(VSL.Physics.wCoM, VesselOrbit.pos.normalized.xzy*10, Color.blue);
				GLUtils.GLVec(VSL.Physics.wCoM, velN.xzy*10, Color.cyan);
				GLUtils.GLVec(VSL.Physics.wCoM, -Body.angularVelocity.normalized*10, Color.white);
				GLUtils.GLVec(VSL.Physics.wCoM, TargetApV.normalized.xzy*10, Color.magenta);
			}
		}

		public override void Reset()
		{
			base.Reset();
			RenderingManager.RemoveFromPostDrawQueue(1, RadarBeam);
		}
		#endif

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
				Status("yellow", "Target should be a vessel");
				return false;
			}
			if(tVSL.LandedOrSplashed)
			{
				Status("yellow", "Target vessel is landed");
				return false;
			}
			if(Math.Abs(tVSL.orbit.inclination-VesselOrbit.inclination) > REN.MaxInclinationDelta)
			{
				Status("yellow", "Target orbit plane is tilted more than {0:F}° with respect to ours.\n" +
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
				NodeDeltaV += AoPCorrection(old, old.DeltaTA)*Math.Sign(Target.orbit.period-VesselOrbit.period);
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
				NodeDeltaV += PlaneCorrection(old)/10;
				NodeDeltaV += AoPCorrection(old, old.DeltaTA)*Math.Sign(Target.orbit.period-VesselOrbit.period)/10;//test
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
			LogF("\nini orbit\n{}", old);//debug
			dV += dV4TTR(old, Target.orbit, REN.MaxTTR, REN.MaxDeltaV, MinPeR, StartUT);
			LogF("\nstart orbit\n{}", NewOrbit(VesselOrbit, dV, StartUT));//debug
			if(!dV.IsZero())
			{
				add_node(dV, StartUT);
				CFG.AP1.On(Autopilot1.Maneuver);
			}
			stage = Stage.StartOrbit;
		}

		Vector3d hVdir, tNorm, norm, velN; //debug
		void to_orbit()
		{
			//compute trajectory to get the needed apoapsis at needed time
			tNorm = Target.orbit.GetOrbitNormal();//test
			LaunchApR = Math.Max(MinPeR, (Target.orbit.PeR+Target.orbit.ApR)/2);
			hVdir = Vector3d.Cross(Target.orbit.GetOrbitNormal(), VesselOrbit.pos).normalized;
			norm  = Vector3d.Cross(VesselOrbit.pos, hVdir).normalized;
			var LaunchRad = Utils.ClampH(Math.Atan(1/(Body.Radius*REN.LaunchTangentK/(2*LaunchApR) - 
			                                          Body.angularV/Math.Sqrt(2*VSL.Physics.StG*(LaunchApR-Body.Radius)))), 
			                             Math.PI/2);
			velN  = (Math.Sin(LaunchRad)*VesselOrbit.pos.normalized + Math.Cos(LaunchRad)*hVdir).normalized;
			var vel = Math.Sqrt(2*VSL.Physics.G*(LaunchApR-Body.Radius)) / Math.Sin(LaunchRad);
			var v   = 0.0;
			while(vel-v > TRJ.dVtol)
			{
				var V = (v+vel)/2;
				var o = NewOrbit(VesselOrbit, velN*V-VesselOrbit.vel, VSL.Physics.UT);
				if(o.ApR > LaunchApR) vel = V;
				else v = V;
			} vel = (v+vel)/2;
			var ascO = NewOrbit(VesselOrbit, velN*vel-VesselOrbit.vel, VSL.Physics.UT);
			LaunchUT = VSL.Physics.UT;
			TTA = ascO.timeToAp;
			ApAUT = LaunchUT+TTA;
			if(VSL.LandedOrSplashed)
			{
				ApV = ascO.getRelativePositionAtUT(ApAUT);
				double TTR;
				do { TTR = correct_launch(); } 
				while(Math.Abs(TTR) > 1);
			}
			else TargetApV = ascO.getRelativePositionAtUT(ApAUT);
			MinAccelTime = 0;//Math.Sin(LaunchRad)*(ascO.vel-VesselOrbit.vel).magnitude/(VSL.Engines.MaxThrustM/VSL.Physics.M-VSL.Physics.G);
			LogF("LaunchAngle {}, Omega0 {}, TTA {}, MinAccelTime {}", LaunchRad*Mathf.Rad2Deg, Body.angularV, TTA, MinAccelTime);//debug
//			if(MinAccelTime <= 0)
//			{
//				Status("red", "TWR < 1, impossible to achive orbit");
//				CFG.AP2.Off();
//				return;
//			}
			CFG.DisableVSC();
			stage = VSL.LandedOrSplashed? Stage.Launch : Stage.ToOrbit;
		}

		double correct_launch()
		{
			var TTR = Utils.ProjectionAngle(Target.orbit.getRelativePositionAtUT(ApAUT), 
			    	                        TargetApV, 
			        	                    Target.orbit.getOrbitalVelocityAtUT(ApAUT)) /
				360*Target.orbit.period;
			LaunchUT += TTR;
			if(LaunchUT-VSL.Physics.UT <= 0) 
				LaunchUT += Target.orbit.period;
			ApAUT= LaunchUT+TTA;
			TargetApV = QuaternionD.AngleAxis((VSL.Physics.UT-LaunchUT)/Body.rotationPeriod*360, Body.angularVelocity.xzy)*ApV;

			var angle = Utils.ProjectionAngle(VesselOrbit.pos, TargetApV, VesselOrbit.vel);//debug
			LogF("\nTTR {}, AngleToApV {}, DistToApV {}\n pos {}\nTargetApV {}", 
			     TTR, angle, angle*Mathf.Deg2Rad*LaunchApR/(LaunchApR-Body.Radius),
			     VesselOrbit.pos, TargetApV);//debug

			return TTR;
		}

		void correct_rendezvou()
		{
			var angle = Utils.ProjectionAngle(Target.orbit.getRelativePositionAtUT(ApAUT), 
			                                  TargetApV, 
			                                  Target.orbit.getOrbitalVelocityAtUT(ApAUT));
			TargetApV = QuaternionD.AngleAxis(angle, Body.angularVelocity.xzy)*TargetApV;

			angle = Utils.ProjectionAngle(VesselOrbit.pos, TargetApV, VesselOrbit.vel);//debug
			LogF("\nAngleToApV {}, DistToApV {}\n pos {}\nTargetApV {}", 
			     angle, angle*Mathf.Deg2Rad*LaunchApR/(LaunchApR-Body.Radius),
			     VesselOrbit.pos, TargetApV);//debug
		}

		void match_orbits()
		{
			SetTarget(Target);
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
			case Stage.Launch:
				//correct launch time
				if(LaunchUT-VSL.Physics.UT > MinAccelTime) 
				{
					correct_launch();
					VSL.Info.Countdown = LaunchUT-VSL.Physics.UT-MinAccelTime;
					WRP.WarpToTime = LaunchUT-MinAccelTime;
					break;
				}
				if(VSL.VerticalSpeed.Absolute/VSL.Physics.G < 5)
				{ 
					CFG.VTOLAssistON = true;
					THR.Throttle = 1; 
					break; 
				}
				CFG.VTOLAssistON = false;
				CFG.StabilizeFlight = false;
				CFG.HF.Off();
				stage = Stage.ToOrbit;
				break;
			case Stage.ToOrbit:
				ApAUT = VSL.Physics.UT+VesselOrbit.timeToAp;
				var cApV   = VesselOrbit.getRelativePositionAtUT(ApAUT);
				var in_atm = Body.atmosphere && VesselOrbit.radius < Body.Radius+Body.atmosphereDepth;
				var h2ApA  = VesselOrbit.ApA-VSL.Altitude.Absolute;
				var dFi    = 90-Vector3d.Angle(VesselOrbit.GetOrbitNormal(), TargetApV);
				var dApA   = LaunchApR-VesselOrbit.ApR;
				var vel    = Vector3d.zero;
				if(dApA > REN.Dtol)  
					vel += VSL.Physics.Up.xzy*dApA/Utils.ClampL(VSL.VerticalSpeed.Absolute, 1);
				if(VesselOrbit.ApA/(LaunchApR-Body.Radius) > REN.RendezvouCorrection) correct_rendezvou();
				var hv     = Vector3d.Exclude(VesselOrbit.pos, VesselOrbit.vel);
				var alpha  = Utils.ProjectionAngle(cApV, TargetApV, VesselOrbit.getOrbitalVelocityAtUT(ApAUT))*Mathf.Deg2Rad*Body.Radius;
				if(alpha > REN.Dtol)
				{
					var hvel = alpha/Utils.ClampL(dApA, REN.GTurnCurve)*Utils.Clamp(h2ApA/100, 0, 1)*REN.GTurnCurve;
					if(Body.atmosphere) hvel *= Math.Sqrt(Utils.Clamp(VSL.Altitude.Absolute/Body.atmosphereDepth, 0, 1));
					vel += hv.normalized*hvel;
				}
				vel *= VSL.Physics.StG/9.81;
				if(!in_atm || dApA > 0 || alpha > 0)
					vel += (QuaternionD.AngleAxis(dFi, VesselOrbit.pos) * hv) - hv;
				
				LogF("\nApV {}\ndApA {}, {}\nalpha {}, {}\ndFi {}, {}", cApV,
				     dApA, VSL.Physics.Up.xzy*dApA/Utils.ClampL(VSL.VerticalSpeed.Absolute, 1)*VSL.Physics.StG/9.81,
				     alpha, hv.normalized*alpha/Utils.ClampL(dApA, REN.GTurnCurve)*Utils.Clamp(h2ApA/100, 0, 1)*REN.GTurnCurve*VSL.Physics.StG/9.81,
				     dFi, (!in_atm || dApA > 0 || alpha > 0)? (QuaternionD.AngleAxis(dFi, VesselOrbit.pos) * hv) - hv : Vector3d.zero);//debug
				
				ToOrbitDeltaV.Value = (float)vel.magnitude;
				if(ToOrbitDeltaV.Value > 1 && (dApA > REN.Dtol || VesselOrbit.timeToAp > 180))
				{
					if(ToOrbitDeltaV && VSL.Controls.RCSAvailable) 
						CFG.AT.OnIfNot(Attitude.KillRotation);
					else 
					{
						CFG.AT.OnIfNot(Attitude.Custom);
						BRC.ForwardDirection = hV(VSL.Physics.UT).xzy;
	//					var velW = VSL.IsActiveVessel? vel.xzy : (vel+Body.GetFrameVel()).xzy;
	//					if(Body.inverseRotation) velW -= Body.getRFrmVel(VesselOrbit.pos.xzy + Body.position);
						ATC.AddCustomRotationW(-vel.xzy, VSL.Engines.MaxThrust);
						LogF("\nvel {}\nThrust {}\nvel*Thrust {}", -vel.xzy, VSL.Engines.MaxThrust, Vector3d.Dot(VSL.Engines.MaxThrust, -vel.normalized.xzy));//debug
					}
					if(VSL.Controls.RCSAvailable)
					{
						if(ToOrbitDeltaV || ATC.AttitudeError > GLB.ATC.AttitudeErrorThreshold)
							TRA.AddDeltaV(-VSL.LocalDir(vel.xzy));
						if(ToOrbitDeltaV) THR.Throttle = 0;
						else THR.DeltaV = ToOrbitDeltaV;
					}
					else THR.DeltaV = ToOrbitDeltaV;
					break;
				}
				CFG.AT.OnIfNot(Attitude.KillRotation);
				THR.Throttle = 0;
				if(in_atm) break;
				LaunchUT = -1;
				start_orbit();
				break;
			case Stage.StartOrbit:
				if(CFG.AP1[Autopilot1.Maneuver]) break;
				CurrentDistance = -1;
				update_trajectory(false);
				if(trajectory.DistanceToTarget < REN.CorrectionStart ||
				   (trajectory.TimeToTarget+trajectory.TimeToStart)/VesselOrbit.period > REN.MaxTTR) 
					stage = Stage.Rendezvou;
				else compute_rendezvou_trajectory();
				break;
			case Stage.ComputeRendezvou:
				if(!trajectory_computed()) break;
				if(trajectory.ManeuverDeltaV.magnitude > GLB.THR.MinDeltaV*5 &&
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
					Status("red", "Failed to compute rendezvou trajectory.\nPlease, try again.");
					CFG.AP2.Off();
					return;
				}
				break;
			case Stage.Rendezvou:
				if(CFG.AP1[Autopilot1.Maneuver]) break;
				if(!CorrectionTimer.Check && CurrentDistance >= 0) break;
				update_trajectory();
				if(CurrentDistance > REN.Dtol*2) correct_trajectory();
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
				if(dPm - VSL.Geometry.R < REN.Dtol) 
				{ brake(); break; }
				var dV = Vector3d.Dot(VesselOrbit.vel-Target.orbit.vel, dP/dPm);
				var nV = Utils.Clamp(dPm*REN.ApproachVelF, 1, REN.MaxApproachV);
				if(dV < nV) THR.DeltaV = (float)(nV-dV);
				else brake();
				break;
			case Stage.Brake:
				if(CFG.AP1[Autopilot1.MatchVelNear]) break;
				if((VSL.Physics.wCoM-Target.CurrentCoM).magnitude-VSL.Geometry.R > REN.Dtol)
				{ approach(); break; }
				CFG.AT.OnIfNot(Attitude.KillRotation);
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

