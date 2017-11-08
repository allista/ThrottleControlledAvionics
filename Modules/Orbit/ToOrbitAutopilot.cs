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
	[RequireModules(typeof(AttitudeControl),
	                typeof(BearingControl),
					typeof(ThrottleControl),
	                typeof(ManeuverAutopilot))]
	public class ToOrbitAutopilot : TrajectoryCalculator
	{
		public new class Config : ModuleConfig
		{
			[Persistent] public float Dtol           = 100f;
			[Persistent] public float RadiusOffset   = 10000f;
			[Persistent] public float GTurnCurve     = 0.9f;
			[Persistent] public float GTurnOffset    = 1000f;
			[Persistent] public float LaunchSlope    = 50f;
			[Persistent] public float MinSlope       = 30f;
			[Persistent] public float MaxSlope       = 70f;
			[Persistent] public float Dist2VelF      = 0.1f;
			[Persistent] public float DragK          = 0.0008f;
		}
		static Config ORB { get { return Globals.Instance.ORB; } }

		public enum Stage { None, Start, Liftoff, GravityTurn, ChangeApA, Circularize }

		[Persistent] public TargetOrbitInfo TargetOrbit = new TargetOrbitInfo();
		[Persistent] public Vector3 Target;
		[Persistent] public Stage stage;

		public bool ShowOptions { get; private set; }

		double ApR { get { return TargetOrbit.ApA*1000+Body.Radius; } }
		ToOrbitExecutor ToOrbit;

		public ToOrbitAutopilot(ModuleTCA tca) : base(tca) {}

		public override void Save(ConfigNode node)
		{
			if(ToOrbit != null && !ToOrbit.Target.IsZero())
				Target = ToOrbit.Target;
			base.Save(node);
		}

		public override void Init()
		{
			base.Init();
			CFG.AP2.AddHandler(this, Autopilot2.ToOrbit);
			TargetOrbit.Slope.Value = Mathf.InverseLerp(ORB.MinSlope, ORB.MaxSlope, ORB.LaunchSlope)*100;
		}

		protected override void UpdateState()
		{
			base.UpdateState();
			IsActive &= CFG.AP2[Autopilot2.ToOrbit] && stage != Stage.None;
		}

		public void ToOrbitCallback(Multiplexer.Command cmd)
		{
			switch(cmd)
			{
			case Multiplexer.Command.Resume:
				if(!check_patched_conics()) return;
				ToOrbit = new ToOrbitExecutor(TCA);
				ToOrbit.CorrectOnlyAltitude = true;
				ToOrbit.Target = Target;
				break;

			case Multiplexer.Command.On:
				Reset();
				if(!check_patched_conics()) return;
				Vector3d hVdir;
				if(TargetOrbit.Inclination.Range > 1e-5f)
				{
					var angle = Utils.Clamp((TargetOrbit.Inclination.Value-TargetOrbit.Inclination.Min)/TargetOrbit.Inclination.Range*180, 0, 180);
					if(TargetOrbit.DescendingNode) angle = -angle;
					hVdir = QuaternionD.AngleAxis(angle, VesselOrbit.pos) * Vector3d.Cross(Vector3d.forward, VesselOrbit.pos).normalized;
				}
				else hVdir = Vector3d.Cross(VesselOrbit.pos, Body.orbit.vel).normalized;
				if(TargetOrbit.RetrogradeOrbit) hVdir *= -1;
				var ApR0 = Utils.ClampH(ApR, MinPeR+ORB.RadiusOffset);
				var ascO = AscendingOrbit(ApR0, hVdir, Mathf.Lerp(ORB.MinSlope, ORB.MaxSlope, TargetOrbit.Slope.Value/100));
				Target = ascO.getRelativePositionAtUT(VSL.Physics.UT+ascO.timeToAp);
				stage = Stage.Start;
				goto case Multiplexer.Command.Resume;

			case Multiplexer.Command.Off:
				Reset();
				break;
			}
		}

		void update_limits()
		{
			TargetOrbit.ApA.Min = (float)(MinPeR-Body.Radius)/1000;
			TargetOrbit.ApA.Max = (float)(Body.sphereOfInfluence-Body.Radius)/1000;
			TargetOrbit.ApA.Value = Utils.Clamp(TargetOrbit.ApA.Value, TargetOrbit.ApA.Min, TargetOrbit.ApA.Max);
			update_inclination_limits();
		}

		void update_inclination_limits()
		{
			//pos x [fwd x pos] = fwd(pos*pos) - pos(fwd*pos)
			var h = Vector3d.forward*VesselOrbit.pos.sqrMagnitude - VesselOrbit.pos * VesselOrbit.pos.z; 
			TargetOrbit.Inclination.Min = (float)Math.Acos(h.z/h.magnitude)*Mathf.Rad2Deg;
			TargetOrbit.Inclination.Max = 180-TargetOrbit.Inclination.Min;
			TargetOrbit.Inclination.ClampValue();
		}

		protected override void Reset()
		{
			base.Reset();
			update_limits();
			ToOrbit = null;
			Target = Vector3d.zero;
			stage = Stage.None;
		}

		double inclination_error(double inclination)
		{
			var error = TargetOrbit.RetrogradeOrbit? 
				180-inclination-TargetOrbit.Inclination.Value : 
				inclination-TargetOrbit.Inclination.Value;
			return TargetOrbit.DescendingNode? -error : error;
		}

		double inclination_correction(double inclination, double chord)
		{ 
			return Utils.Clamp(chord*Math.Tan(inclination_error(inclination)*Mathf.Deg2Rad)
			                   /VesselOrbit.radius*Mathf.Rad2Deg, -10, 10);
		}

		Vector3d correct_dV(Vector3d dV, double UT)
		{
			var v  = VesselOrbit.getOrbitalVelocityAtUT(UT);
			var nV = dV + v;
			return QuaternionD.AngleAxis(-inclination_error(VesselOrbit.inclination), 
			                             VesselOrbit.getRelativePositionAtUT(UT)) * nV - v;
		}

		void change_ApR(double UT)
		{
			var dV = correct_dV(dV4Ap(VesselOrbit, ApR, UT), UT);
			ManeuverAutopilot.AddNode(VSL, dV, UT);
			CFG.AP1.On(Autopilot1.Maneuver);
			stage = Stage.ChangeApA;
		}

		void circularize(double UT)
		{
			var dV = correct_dV(dV4C(VesselOrbit, hV(UT), UT), UT);
			ManeuverAutopilot.AddNode(VSL, dV, UT);
			CFG.AP1.On(Autopilot1.Maneuver);
			stage = Stage.Circularize;
		}

		protected override void Update()
		{
			switch(stage)
			{
			case Stage.Start:
                if(VSL.LandedOrSplashed)
                    stage = Stage.Liftoff;
                else
                {
                    ToOrbit.StartGravityTurn();
                    stage = Stage.GravityTurn;
                }
				break;
			case Stage.Liftoff:
				if(ToOrbit.Liftoff()) break;
				stage = Stage.GravityTurn;
				break;
			case Stage.GravityTurn:
				update_inclination_limits();
				var norm = VesselOrbit.GetOrbitNormal();
				var ApV = VesselOrbit.getRelativePositionAtUT(VSL.Physics.UT+VesselOrbit.timeToAp);
				var arcT = AngleDelta(VesselOrbit, ToOrbit.Target);
				if(arcT > 0 && arcT < AngleDelta(VesselOrbit, ApV))
				{
					ApV.Normalize();
					var chord = ApV*VesselOrbit.radius - VesselOrbit.pos;
					var alpha = inclination_correction(VesselOrbit.inclination, chord.magnitude);
					ToOrbit.Target = QuaternionD.AngleAxis(alpha, Vector3d.Cross(norm, ApV))
						*ApV*ToOrbit.TargetR;
				}
				else
				{
					var n = Vector3d.Cross(VesselOrbit.pos, ToOrbit.Target.normalized);
					var inclination = Math.Acos(n.z/n.magnitude)*Mathf.Rad2Deg;
					var chord = Vector3d.Exclude(norm, ToOrbit.Target).normalized*VesselOrbit.radius - VesselOrbit.pos;
					var alpha = inclination_correction(inclination, chord.magnitude);
					ToOrbit.Target = QuaternionD.AngleAxis(alpha, Vector3d.Cross(norm, VesselOrbit.pos))
						*ToOrbit.Target;
				}
				if(ToOrbit.GravityTurn(ManeuverOffset, ORB.GTurnCurve, ORB.Dist2VelF, ORB.Dtol)) break;
				CFG.BR.OffIfOn(BearingMode.Auto);
				var ApAUT = VSL.Physics.UT+VesselOrbit.timeToAp;
				if(ApR > MinPeR + ORB.RadiusOffset) change_ApR(ApAUT);
				else circularize(ApAUT);
				break;
			case Stage.ChangeApA:
                TmpStatus("Achieving target apoapsis...");
				if(CFG.AP1[Autopilot1.Maneuver]) break;
				circularize(VSL.Physics.UT+VesselOrbit.timeToAp);
				stage = Stage.Circularize;
				break;
			case Stage.Circularize:
                TmpStatus("Circularization...");
				if(CFG.AP1[Autopilot1.Maneuver]) break;
                Disable();
				ClearStatus();
				break;
			}
		}

		void toggle_orbit_editor()
		{
			ShowOptions = !ShowOptions;
			if(ShowOptions) update_limits();
		}

		public override void Draw()
		{
			#if DEBUG
			if(ToOrbit != null)
			{
				Utils.GLVec(Body.position, ToOrbit.Target.xzy, Color.green);
				Utils.GLVec(Body.position, VesselOrbit.getRelativePositionAtUT(VSL.Physics.UT+VesselOrbit.timeToAp).xzy, Color.magenta);
				Utils.GLVec(Body.position, VesselOrbit.GetOrbitNormal().normalized.xzy*Body.Radius*1.1, Color.cyan);
				Utils.GLVec(Body.position, Vector3d.Cross(VesselOrbit.pos, ToOrbit.Target).normalized.xzy*Body.Radius*1.1, Color.red);
			}
			#endif
			if(stage == Stage.None)
			{
				if(Utils.ButtonSwitch("ToOrbit", ShowOptions, 
				                   	  "Achieve a circular orbit with desired radius and inclination", 
				                      GUILayout.ExpandWidth(true)))
					toggle_orbit_editor();
			}
			else if(GUILayout.Button(new GUIContent("ToOrbit", "Change target orbit or abort"), 
			                         Styles.danger_button, GUILayout.ExpandWidth(true)))
				toggle_orbit_editor();
		}

		public void DrawOptions()
		{
			GUILayout.BeginVertical();
			TargetOrbit.Draw();
			GUILayout.BeginHorizontal();
			ShowOptions = !GUILayout.Button("Cancel", Styles.active_button, GUILayout.ExpandWidth(true));
			if(stage != Stage.None && 
			   GUILayout.Button("Abort", Styles.danger_button, GUILayout.ExpandWidth(true)))
			{
				ShowOptions = false;
				CFG.AP2.XOff();
			}
			if(GUILayout.Button(stage == Stage.None? "Launch" : "Change", 
			                    Styles.confirm_button, GUILayout.ExpandWidth(true)))
			{
				ShowOptions = false;
                TargetOrbit.UpdateValues();
                VSL.Engines.ActivateEnginesAndRun(() => CFG.AP2.XOn(Autopilot2.ToOrbit));
			}
			GUILayout.EndHorizontal();
			GUILayout.EndVertical();
		}
	}

	public class TargetOrbitInfo : ConfigNodeObject
	{
		[Persistent] public FloatField ApA = new FloatField();
		[Persistent] public FloatField Inclination = new FloatField(format: "F3", min: 0, max: 180);
		[Persistent] public FloatField Slope = new FloatField(format: "F1", min: 0, max: 100);
		[Persistent] public bool DescendingNode;
		[Persistent] public bool RetrogradeOrbit;

		public void UpdateValues()
		{
			ApA.UpdateValue();
			Inclination.UpdateValue();
		}

		public void Draw()
		{
			GUILayout.BeginHorizontal();
            GUILayout.BeginVertical();
            GUILayout.Label("Radius:", GUILayout.ExpandWidth(true));
            GUILayout.Label("Inclination:", GUILayout.ExpandWidth(true));
            GUILayout.Label("Steepness:", GUILayout.ExpandWidth(true));
            GUILayout.EndVertical();
            GUILayout.BeginVertical();
            GUILayout.FlexibleSpace();
            GUILayout.BeginHorizontal();
            GUILayout.FlexibleSpace();
            if(GUILayout.Button(new GUIContent(DescendingNode? "DN" : "AN", "Launch from Ascending or Descending Node?"), 
                                DescendingNode? Styles.danger_button : Styles.enabled_button,
                                GUILayout.ExpandWidth(false)))
                DescendingNode = !DescendingNode;
            if(GUILayout.Button(new GUIContent(RetrogradeOrbit? "RG" : "PG", "Prograde or retrograde orbit?"), 
                                RetrogradeOrbit? Styles.danger_button : Styles.enabled_button,
                                GUILayout.ExpandWidth(false)))
                RetrogradeOrbit = !RetrogradeOrbit;
            GUILayout.EndHorizontal();
            GUILayout.FlexibleSpace();
            GUILayout.EndVertical();
            GUILayout.BeginVertical();
			ApA.Draw("km", 5, suffix_width: 25);
            Inclination.Draw("°", 5, suffix_width: 25);
            Slope.Draw("%", 5, suffix_width: 25);
            GUILayout.EndVertical();
			GUILayout.EndHorizontal();
		}
	}
}

