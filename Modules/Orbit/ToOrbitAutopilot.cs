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
	[RequireModules(typeof(ThrottleControl),
	                typeof(TranslationControl))]
	public class ToOrbitAutopilot : TrajectoryCalculator
	{
		public new class Config : ModuleConfig
		{
			[Persistent] public float Dtol           = 100f;
			[Persistent] public float RadiusOffset   = 10000f;
			[Persistent] public float GTurnCurve     = 0.6f;
			[Persistent] public float LaunchTangentK = 1f;
			[Persistent] public float Dist2VelF      = 0.1f;
		}
		static Config ORB { get { return TCAScenario.Globals.ORB; } }

		public enum Stage { None, Start, Liftoff, GravityTurn, ChangeApA, Circularize }

		[Persistent] public FloatField ApA = new FloatField();
		[Persistent] public FloatField Inclination = new FloatField(format: "F1", min: -180, max: 180);
		[Persistent] public Vector3 Target;
		[Persistent] public Stage stage;

		public bool ShowEditor { get; private set; }

		double ApR { get { return ApA*1000+Body.Radius; } }
		ToOrbitExecutor ToOrbit;

		public ToOrbitAutopilot(ModuleTCA tca) : base(tca) {}

		public override void Init()
		{
			base.Init();
			CFG.AP2.AddHandler(this, Autopilot2.ToOrbit);
		}

		protected override void UpdateState()
		{
			base.UpdateState();
			IsActive &= CFG.AP2[Autopilot2.ToOrbit] && stage != Stage.None;
		}

		public void ToOrbitCallback(Multiplexer.Command cmd)
		{
			reset();
			switch(cmd)
			{
			case Multiplexer.Command.Resume:
				ToOrbit = new ToOrbitExecutor(TCA);
				ToOrbit.Target = Target;
				break;

			case Multiplexer.Command.On:
				var hVdir = Vector3d.Cross(VesselOrbit.pos, Body.zUpAngularVelocity.normalized).normalized;
				var ascO = AscendingOrbit(Utils.ClampH(ApR, MinPeR+ORB.RadiusOffset), hVdir, ORB.LaunchTangentK);
				Target = ascO.getRelativePositionAtUT(VSL.Physics.UT+ascO.timeToAp);
				stage = Stage.Start;
				goto case Multiplexer.Command.Resume;

			case Multiplexer.Command.Off:
				ClearStatus();
				break;
			}
		}

		protected override void reset()
		{
			base.reset();
			ApA.Min = (float)(MinPeR-Body.Radius)/1000;
			ApA.Max = (float)(Body.sphereOfInfluence-Body.Radius)/1000;
			ApA.Value = Utils.Clamp(ApA.Value, ApA.Min, ApA.Max);
			ToOrbit = null;
			Target = Vector3d.zero;
			stage = Stage.None;
		}

		void change_ApR(double UT)
		{
			var dV = dV4Ap(VesselOrbit, ApR, UT);
			ManeuverAutopilot.AddNode(VSL, dV, UT);
			CFG.AP1.On(Autopilot1.Maneuver);
			stage = Stage.ChangeApA;
		}

		void circularize(double UT)
		{
			var dV = dV4C(VesselOrbit, hV(UT), UT);
			ManeuverAutopilot.AddNode(VSL, dV, UT);
			CFG.AP1.On(Autopilot1.Maneuver);
			stage = Stage.Circularize;
		}

		protected override void Update()
		{
			if(!IsActive) return;
			switch(stage)
			{
			case Stage.Start:
				stage = VSL.LandedOrSplashed ? Stage.Liftoff : Stage.GravityTurn;
				break;
			case Stage.Liftoff:
				if(ToOrbit.Liftoff()) break;
				stage = Stage.GravityTurn;
				break;
			case Stage.GravityTurn:
				if(ToOrbit.GravityTurn(ORB.GTurnCurve, ORB.Dist2VelF, ORB.Dtol)) break;
				var UT = VSL.Physics.UT+VesselOrbit.timeToAp;
				if(ApR > MinPeR + ORB.RadiusOffset) change_ApR(UT);
				else circularize(UT);
				break;
			case Stage.ChangeApA:
				Status("Achieving target apoapsis...");
				if(CFG.AP1[Autopilot1.Maneuver]) break;
				circularize(VSL.Physics.UT+VesselOrbit.timeToAp);
				stage = Stage.Circularize;
				break;
			case Stage.Circularize:
				Status("Circularization...");
				if(CFG.AP1[Autopilot1.Maneuver]) break;
				CFG.AP2.Off();
				break;
			}
		}

		public override void Draw()
		{
			if(stage == Stage.None)
			{
				if(Utils.ButtonSwitch("ToOrbit", ShowEditor, 
				                   	  "Achieve a circular orbit with desired radius and inclination", 
				                      GUILayout.ExpandWidth(false)))
				{
					ShowEditor = !ShowEditor;
					if(ShowEditor) reset();
				}
			}
			else if(GUILayout.Button(new GUIContent("ToOrbit", "Abort Autopilot"), Styles.danger_button, GUILayout.ExpandWidth(false)))
				CFG.AP2.XOff();
		}

		public void DrawOrbitEditor()
		{
			GUILayout.BeginVertical();
			GUILayout.BeginHorizontal();
			ApA.Draw("km", true, 10);
			GUILayout.EndHorizontal();
			GUILayout.BeginHorizontal();
			Inclination.Draw("°", true, 5);
			GUILayout.EndHorizontal();
			GUILayout.BeginHorizontal();
			if(GUILayout.Button("Cancel", Styles.close_button, GUILayout.ExpandWidth(true)))
			{
				ShowEditor = false;
				reset();
			}
			if(GUILayout.Button("Launch", Styles.confirm_button, GUILayout.ExpandWidth(true)))
			{
				ShowEditor = false;
				CFG.AP2.XOn(Autopilot2.ToOrbit);
			}
			GUILayout.EndHorizontal();
			GUILayout.EndVertical();
		}
	}


	public class ToOrbitExecutor : TCAComponent
	{
		ThrottleControl THR;
		BearingControl BRC;

		readonly SingleAction GearAction = new SingleAction();
		readonly FuzzyThreshold<double> ErrorThreshold = new FuzzyThreshold<double>();
		readonly ManeuverExecutor Executor;

		Vector3d target;
		public Vector3d Target 
		{ 
			get { return target; } 
			set { target = value; TargetR = target.magnitude; } 
		}
		public double TargetR { get; private set; }

		public double LaunchUT;
		public double ApAUT;

		Orbit VesselOrbit { get { return VSL.vessel.orbitDriver.orbit; } }
		CelestialBody Body { get { return VesselOrbit.referenceBody; } }
		Vector3d hV(double UT) { return TrajectoryCalculator.hV(VesselOrbit, UT); }
		public double dApA { get; private set; }

		public ToOrbitExecutor(ModuleTCA tca) : base(tca) 
		{ 
			InitModuleFields();
			Executor = new ManeuverExecutor(tca);
			GearAction.action = () =>VSL.GearOn(false);
			ErrorThreshold.Lower = 2*GLB.ORB.Dtol;
			ErrorThreshold.Upper = 10*GLB.ORB.Dtol;
		}

		public bool Liftoff()
		{
			VSL.Engines.ActivateEnginesIfNeeded();
			if(VSL.VerticalSpeed.Absolute/VSL.Physics.G < 5)
			{ 
				Status("Liftoff...");
				CFG.DisableVSC();
				CFG.VTOLAssistON = true;
				THR.Throttle = 1;
				return true;
			}
			GearAction.Run();
			CFG.VTOLAssistON = false;
			CFG.StabilizeFlight = false;
			CFG.HF.Off();
			return false;
		}

		public bool GravityTurn(double gturn_curve, double dist2vel, double Dtol)
		{
			dApA = TargetR-VesselOrbit.ApR;
			var vel   = Vector3d.zero;
			var cApV  = VesselOrbit.getRelativePositionAtUT(VSL.Physics.UT+VesselOrbit.timeToAp);
			var hv    = Vector3d.Exclude(VesselOrbit.pos, VesselOrbit.vel).normalized;
			var alpha = Utils.ProjectionAngle(cApV, target, Vector3d.Cross(VesselOrbit.GetOrbitNormal(), cApV))*Mathf.Deg2Rad*Body.Radius;
			ErrorThreshold.Value = dApA+alpha;
			if(!ErrorThreshold)
			{
				if(alpha > Dtol)
				{
					var hvel = Utils.ClampL(alpha-dApA*gturn_curve, 0)*dist2vel*
						Utils.Clamp((VesselOrbit.ApA-VSL.Altitude.Absolute)/100, 0, 1);
					if(Body.atmosphere) hvel *= Math.Sqrt(Utils.Clamp(VSL.Altitude.Absolute/Body.atmosphereDepth, 0, 1));
					vel += hv*hvel;
				}
				if(dApA > Dtol)
					vel += VSL.Physics.Up.xzy*dApA*gturn_curve*dist2vel;
				vel *= VSL.Physics.StG/Utils.G0;
				if(!vel.IsZero())
				{
					var norm = VesselOrbit.GetOrbitNormal();
					vel += norm*Math.Sin((90-Vector3d.Angle(norm, target))*Mathf.Deg2Rad)*vel.magnitude*10;
				}
				vel = vel.xzy;
				if(Executor.Execute(vel, 1)) 
				{
					if(CFG.AT.Not(Attitude.KillRotation)) 
					{
						CFG.BR.OnIfNot(BearingMode.Auto);
						BRC.ForwardDirection = hV(VSL.Physics.UT).xzy;
					}
					Status("Gravity turn...");
					return true;
				}
			}
			CFG.BR.OffIfOn(BearingMode.Auto);
			Status("Coasting...");
			CFG.AT.OnIfNot(Attitude.KillRotation);
			THR.Throttle = 0;
			return (Body.atmosphere && VesselOrbit.radius < Body.Radius+Body.atmosphereDepth);
		}
	}
}

