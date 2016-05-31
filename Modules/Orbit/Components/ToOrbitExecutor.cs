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
	public class ToOrbitExecutor : TCAComponent
	{
		const double MinClimbTime = 5;

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
			if(VSL.VerticalSpeed.Absolute/VSL.Physics.G < MinClimbTime)
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
					vel += norm*Math.Sin((90-Vector3d.Angle(norm, target))*Mathf.Deg2Rad)*vel.magnitude*
						Utils.Clamp(VSL.VerticalSpeed.Absolute/VSL.Physics.G-MinClimbTime, 0, 100);
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

