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
	[CareerPart]
	[RequireModules(typeof(ManeuverAutopilot), 
	                typeof(AutoLander))]
	public class DeorbitAutopilot : LandingTrajectoryCalculator
	{
		public new class Config : TCAModule.ModuleConfig
		{
			[Persistent] public float StartOffset = 60f;   //s
			[Persistent] public float StartPeR    = 0.49f; //of planet radius
		}
		static Config DEO { get { return TCAScenario.Globals.DEO; } }

		public DeorbitAutopilot(ModuleTCA tca) : base(tca) {}

		public override void Init()
		{
			base.Init();
			CFG.AP2.AddHandler(this, Autopilot2.Deorbit);
		}

		enum Stage { None, Deorbit, Waiting }
		Stage stage;
		Vector3d dVn;
		double CurrentPeR;
		bool compute_node;

		protected override void bootstrap_computation()
		{ dVn = Vector3d.zero; }

		protected override LandingTrajectory compute_next_trajectory(LandingTrajectory old)
		{
			var StartUT = 0.0;
			var targetAlt = 0.0;
			if(old != null) 
			{
				StartUT = old.StartUT;
				targetAlt = old.TargetAlt;
				if(Math.Abs(old.DeltaLon) > Math.Abs(old.DeltaLat))
				{
					StartUT = old.StartUT+old.DeltaLon/360*VesselOrbit.period;
					if(StartUT-VSL.Physics.UT < DEO.StartOffset) StartUT += VesselOrbit.period;
					if(old.ManeuverDuration > StartUT-VSL.Physics.UT) StartUT = VSL.Physics.UT+old.ManeuverDuration+DEO.StartOffset;
				}
				else dVn += VesselOrbit.GetOrbitNormal().normalized*
						VesselOrbit.vel.magnitude*Math.Sin(old.DeltaLat*Mathf.Deg2Rad/2);
//				Log("StartUT change: {0}\ndVn {1}", StartUT-old.StartUT, dVn);//debug
			}
			else 
			{
				StartUT = VSL.Physics.UT+DEO.StartOffset;
				targetAlt = TargetAlt;
			}
			return new LandingTrajectory(VSL, dV4Pe(VesselOrbit, Body.Radius*CurrentPeR, StartUT, dVn), StartUT, targetAlt);
		}

		protected override void reset()
		{
			base.reset();
			stage = Stage.None;
			CurrentPeR = DEO.StartPeR;
			CFG.AP1.Off();
		}

		public void DeorbitCallback(Multiplexer.Command cmd)
		{
			reset();
			switch(cmd)
			{
			case Multiplexer.Command.Resume:
				break;

			case Multiplexer.Command.On:
				if(VSL.Target == null) 
				{
					CFG.AP2.Off();
					return;
				}
				if(VesselOrbit.PeR < Body.Radius)
				{
					ThrottleControlledAvionics.StatusMessage = "Unable to perform the <b>Ballistic Jump</b> from orbit.\n" +
						"Use <b>Land at Target</b> instead.";
					CFG.AP2.Off();
					return;
				}
				clear_nodes();
				setup_target();
				compute_node = true;
				goto case Multiplexer.Command.Resume;

			case Multiplexer.Command.Off:
				break;
			}
		}

		protected override void Update()
		{
			if(Target == null || VSL.orbit.referenceBody == null) return;
			if(CFG.Target != Target) SetTarget(Target);
			if(compute_node && trajectory_computed())
			{
				if(trajectory.DistanceToTarget < TRJ.Dtol || CurrentPeR >= 1)
				{
					Working = true;
					compute_node = false;
					ManeuverAutopilot.AddNode(VSL, trajectory.ManeuverDeltaV, trajectory.StartUT);
					if(trajectory.DistanceToTarget < TRJ.Dtol) 
					{ CFG.AP1.On(Autopilot1.Maneuver); stage = Stage.Deorbit; }
					else 
					{
						ThrottleControlledAvionics.StatusMessage = "<color=red>Predicted landing site is too far from the target.\n" +
							"<i>To proceed, activate maneuver execution manually.</i></color>";
						stage = Stage.Waiting;
					}
				}
				else 
				{
					CurrentPeR += 0.1;
					if(CurrentPeR < 1) trajectory = null;
				}
				return;
			}
			if(Working) 
			{ 
				if(landing) { do_land(); return; }
				switch(stage)
				{
				case Stage.Waiting:
					if(!CFG.AP1[Autopilot1.Maneuver]) break;
					stage = Stage.Deorbit;
					break;
				case Stage.Deorbit:
					if(CFG.AP1[Autopilot1.Maneuver]) break;
					stage = Stage.None;
					start_landing();
					break;
				}
			}
		}

		public override void Draw()
		{
			if(compute_node) GUILayout.Label("Computing...", Styles.grey_button, GUILayout.ExpandWidth(false));
			else if(Utils.ButtonSwitch("Land at Target", CFG.AP2[Autopilot2.Deorbit],
			                           "Compute and perform a deorbit maneuver, then land at the target.", 
			                           GUILayout.ExpandWidth(false)))
				CFG.AP2.XToggle(Autopilot2.Deorbit);
		}
	}
}

