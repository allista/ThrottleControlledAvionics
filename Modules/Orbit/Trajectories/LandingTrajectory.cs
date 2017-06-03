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
using System.Collections.Generic;
using UnityEngine;
using AT_Utils;

namespace ThrottleControlledAvionics
{
	public class LandingTrajectory : TargetedTrajectory
	{
		public double TargetAltitude;
		public double LandingAngle;
		public WayPoint SurfacePoint { get; private set; }

		public double VslStartLat { get; private set; }
		public double VslStartLon { get; private set; }

		public double DeltaLat { get; private set; }
		public double DeltaLon { get; private set; }
        Coordinates approach;

		/// <summary>
		/// Radial difference between the target and the landing site in degrees.
        /// Is positive if the target is located after the landing site along the surface velocity,
        /// negative otherwise.
		/// </summary>
		public double DeltaR { get; private set; } = 180;

		public float  BrakeOffset;
		public double BrakeStartUT { get; private set; }
		public double BrakeEndUT { get; private set; }
		public double BrakeEndDeltaAlt { get; private set; }
        public double AerobrakeStartUT { get; private set; }

		public LandingTrajectory(VesselWrapper vsl, Vector3d dV, double startUT, 
		                         WayPoint target, double target_altitude = 0, bool with_brake = true)
			: base(vsl, dV, startUT, target)
		{
			TargetAltitude = target_altitude;
			update(with_brake);
		}

        void update_landing_site()
        {
            var at_target_rot = TrajectoryCalculator.BodyRotationAtdT(Body, -TimeToTarget);
            approach = new Coordinates((at_target_rot*Orbit.getRelativePositionAtUT(BrakeEndUT-1)).xzy+Body.position, Body);
            SurfacePoint = new WayPoint((at_target_rot*AtTargetPos).xzy+Body.position, Body);
            SurfacePoint.Pos.SetAlt2Surface(Body);
            SurfacePoint.Name = "Landing Site";
        }

		void update_from_orbit(Orbit orb, double UT)
		{
			//calculate the position of a landing site
			if(orb.ApA <= TargetAltitude) 
				AtTargetUT = orb.StartUT+(orb.ApAhead()? orb.timeToAp : 1);
			else if(orb.PeA < TargetAltitude) 
				AtTargetUT = TrajectoryCalculator.NearestRadiusUT(orb, Body.Radius+TargetAltitude, UT);
			else AtTargetUT = orb.StartUT+orb.timeToPe;
			TransferTime = AtTargetUT-StartUT;
			AtTargetPos = orb.getRelativePositionAtUT(AtTargetUT);
			AtTargetVel = orb.getOrbitalVelocityAtUT(AtTargetUT);
            update_landing_site();
		}

		void ClampBrakeDeltaV()
		{
            BrakeFuel = 0;
            FullBrake = true;
			var dVm = BrakeDeltaV.magnitude;
			if(dVm > 0) 
			{
				var fuel = VSL.Engines.GetAvailableFuelMass()-ManeuverFuel;
				if(fuel <= 0) BrakeDeltaV = Vector3d.zero;
				else
				{
					BrakeFuel = VSL.Engines
						.FuelNeededAtAlt((float)dVm, (float)(BrakeEndDeltaAlt+TargetAltitude));
					if(BrakeFuel > fuel)
					{
						BrakeDeltaV = BrakeDeltaV*VSL.Engines.DeltaV((float)fuel)/dVm;
						BrakeFuel = fuel;
                        FullBrake = false;
					}
				}
			}
		}

		void SetBrakeEndUT(double UT)
		{ 
			BrakeEndUT = UT;
			BrakeEndDeltaAlt = Orbit.getRelativePositionAtUT(BrakeEndUT).magnitude-Body.Radius-TargetAltitude; 
		}

		void SetBrakeDeltaV(Vector3d dV)
		{
			BrakeDeltaV = dV;
			ClampBrakeDeltaV();
		}

        double GetAerobrakeStartUT()
        {
            if(!Body.atmosphere) return AtTargetUT-1;
            var start = StartUT;
            var end = AtTargetUT;
            while(end-start > 0.1)
            {
                var UT = start+(end-start)/2;
                var pos = Orbit.getRelativePositionAtUT(UT);
                var alt = pos.magnitude-Body.Radius;
                var density = Body.GetDensity(Body.GetPressure(alt), Body.GetTemperature(alt));
                if(density > GLB.LTRJ.MinAerobrakeDensity) end = UT;
                else start = UT;
            }
            return start+(end-start)/2;
        }

		void update(bool with_brake)
		{
            update_from_orbit(Orbit, StartUT);
			LandingAngle = 90-Vector3d.Angle(AtTargetPos, -AtTargetVel);
            AerobrakeStartUT = GetAerobrakeStartUT();
			//correct for brake maneuver
			if(with_brake)
			{
				//estimate time needed to rotate the ship downwards
				var rotation_time = VSL.Torque.NoEngines? 
					VSL.Torque.NoEngines.MinRotationTime(90) :
					VSL.Torque.MaxPossible.RotationTime(90, 0.1f);
                //estimate amount of fuel needed for the maneuver
                var vertical_vel = Vector3d.Project(AtTargetVel, AtTargetPos);
                SetBrakeEndUT(Math.Max(AtTargetUT-GLB.LTRJ.CorrectionOffset+rotation_time, StartUT));
                SetBrakeDeltaV(vertical_vel);
				if(BrakeFuel > 0)
				{
                    //calculate braking maneuver
                    BrakeDuration = VSL.Engines.OnPlanetTTB(BrakeDeltaV, 
                                                            Orbit.getRelativePositionAtUT(BrakeEndUT), 
                                                            (float)(BrakeEndDeltaAlt+TargetAltitude));
					BrakeDuration += rotation_time;
                    if(FullBrake)
                    {
                        //find appropriate point to perform the maneuver
                        var brake_end_UT = Math.Max(AtTargetUT-Mathf.Max(GLB.LTRJ.CorrectionOffset, BrakeDuration*1.1f), StartUT);
                        var vertical_speed = vertical_vel.magnitude;
                        double fly_over_error;
                        do {
                            SetBrakeEndUT(brake_end_UT);
                            fly_over_error = BrakeEndDeltaAlt - GLB.LTRJ.FlyOverAlt;
                            brake_end_UT -= Math.Abs(fly_over_error/vertical_speed);
                        } while(brake_end_UT > StartUT && fly_over_error < -1);
                    }
                    else SetBrakeEndUT(AerobrakeStartUT);
                    //update landing site
                    var atBrakeVel = Orbit.getOrbitalVelocityAtUT(BrakeEndUT);
                    AtTargetPos = Orbit.getRelativePositionAtUT(BrakeEndUT).normalized*(Body.Radius+TargetAltitude);
                    var srfVel = Vector3d.Cross(Body.zUpAngularVelocity, AtTargetPos);
                    SetBrakeDeltaV(-Vector3d.Exclude(AtTargetPos, atBrakeVel)+srfVel);
                    BrakeStartUT = Math.Max(BrakeEndUT-MatchVelocityAutopilot.BrakingOffset((float)BrakeDeltaV.magnitude, VSL, out BrakeDuration), StartUT);
                    AtTargetVel = Vector3d.Project(atBrakeVel, AtTargetPos);
                    var v2 = AtTargetVel.sqrMagnitude;
                    var v0 = Math.Sqrt(v2);
                    AtTargetUT = (Math.Sqrt(v2+2*VSL.Physics.StG*BrakeEndDeltaAlt)-v0)/VSL.Physics.StG;
                    AtTargetVel *= VSL.Physics.StG*AtTargetUT/v0+1;
                    AtTargetVel += srfVel;
                    AtTargetUT += BrakeEndUT;
                    update_landing_site();
				}
				else //no brake maneuver
				{
                    SetBrakeEndUT(AerobrakeStartUT);
                    BrakeStartUT = BrakeEndUT;	
					BrakeDuration = 0;
				}
			}
			else
			{
				SetBrakeEndUT(TrajectoryCalculator.FlyAboveUT(Orbit, Target.RelOrbPos(Body), StartUT));
				SetBrakeDeltaV(-AtTargetVel+Vector3d.Cross(Body.zUpAngularVelocity, AtTargetPos));
				if(BrakeFuel > 0)
				{
					var offset = MatchVelocityAutopilot.BrakingOffset((float)BrakeDeltaV.magnitude, VSL, out BrakeDuration);
//                    if(!FullBrake) SetBrakeEndUT(AerobrakeStartUT);
					BrakeStartUT = Math.Max(BrakeEndUT-offset, StartUT);
				}
				else
				{
                    SetBrakeEndUT(AtTargetUT-1);
                    BrakeStartUT = BrakeEndUT;  
                    BrakeDuration = 0;
				}
			}
			BrakeOffset = (float)Utils.ClampL(BrakeEndUT-BrakeStartUT, 0);
			//compute vessel coordinates at maneuver start
			if(VSL.LandedOrSplashed)
			{
				VslStartLat = Utils.ClampAngle(VSL.vessel.latitude);
				VslStartLon = Utils.ClampAngle(VSL.vessel.longitude);
			}
			else
			{
				var start_pos = (TrajectoryCalculator.BodyRotationAtdT(Body, -TimeToStart)*StartPos).xzy + Body.position;
				VslStartLat = Utils.ClampAngle(Body.GetLatitude(start_pos));
				VslStartLon = Utils.ClampAngle(Body.GetLongitude(start_pos));
			}
			//compute distance to target
			DistanceToTarget = Target.AngleTo(SurfacePoint)*Body.Radius;
            SurfacePoint.Name += string.Format("\n{0} from target", Utils.formatBigValue((float)DistanceToTarget, "m"));
			//compute distance in lat-lon coordinates
			DeltaLat = Utils.AngleDelta(SurfacePoint.Pos.Lat, Target.Pos.Lat)*
                Math.Sign(Utils.AngleDelta(approach.Lat, SurfacePoint.Pos.Lat));
			DeltaLon = Utils.AngleDelta(SurfacePoint.Pos.Lon, Target.Pos.Lon)*
                Math.Sign(Utils.AngleDelta(approach.Lon, SurfacePoint.Pos.Lon));
			//compute distance in radial coordinates
			DeltaFi = 90-Vector3d.Angle(Orbit.GetOrbitNormal(),
			                            TrajectoryCalculator.BodyRotationAtdT(Body, TimeToTarget) * 
			                            Body.GetRelSurfacePosition(Target.Pos.Lat, Target.Pos.Lon, TargetAltitude).xzy);
			DeltaR = Utils.RadDelta(SurfacePoint.AngleTo(VslStartLat, VslStartLon), Target.AngleTo(VslStartLat, VslStartLon))*Mathf.Rad2Deg;
//            Utils.Log("{}", this);//debug
		}

		public List<AtmosphericConditions> GetAtmosphericCurve(double dT, double endUT = -1)
		{
			if(!Body.atmosphere) return null;
			var atmoR = Body.Radius+Body.atmosphereDepth;
			var startUT = StartPos.magnitude < atmoR? StartUT : TrajectoryCalculator.NearestRadiusUT(Orbit, atmoR, StartUT);
			if(endUT < 0) endUT = BrakeEndUT;
			if(startUT > endUT) return null;
			var samples = (int)Math.Ceiling((endUT-startUT)/dT)+1;
			var curve = new List<AtmosphericConditions>(samples);
			dT = (endUT-startUT)/samples;
			for(int i = 1; i <= samples; i++)
			{
				var cond = new AtmosphericConditions(Orbit, startUT+dT*i);
				cond.Duration = dT;
				curve.Add(cond);
			}
			return curve;
		}

		public Vector3d GetOrbitVelocityAtSurface()
		{ return Orbit.getOrbitalVelocityAtUT(AtTargetUT); }

		public override void UpdateOrbit(Orbit current)
		{
			base.UpdateOrbit(current);
			update(false);
		}

		public void UpdateOrbit(Orbit current, bool with_brake)
		{
			base.UpdateOrbit(current);
			update(with_brake);
		}

		public override string ToString()
		{
			return base.ToString()+
				Utils.Format("\nLanding Site: {},\n" +
		                     "Delta R: {} deg\n" +
				             "Delta Lat: {} deg, Delta Lon: {} deg\n" +
                             "Brake End UT {}\n" +
				             "Time to Brake: {} s\n" +
				             "BrakeEnd Altitude {} m\n" +
				             "Atmo Conditions: {}\n" +
				             "Landing Angle {} deg",
				             SurfacePoint,
				             DeltaR, DeltaLat, DeltaLon,
                             BrakeEndUT, BrakeStartUT-VSL.Physics.UT, BrakeEndDeltaAlt,
				             GetAtmosphericCurve(5),
				             LandingAngle);
		}
	}

    public class AtmosphericConditionsGeneric
    {
        public double Altitude;
        public double Speed;

        public bool   Atmosphere;
        public double Pressure;
        public double Density;
        public double AtmosphericTemperature;

        public double DynamicPressure;
        public double ShockTemperature;
        public double ConvectiveCoefficient;

//      ptd.finalCoeff = this.convectiveCoefficient * ptd.convectionArea * 0.001 * part.heatConvectiveConstant * ptd.convectionCoeffMultiplier;
//      ptd.finalCoeff = Math.Min(ptd.finalCoeff, part.skinThermalMass * part.skinExposedAreaFrac);

        public AtmosphericConditionsGeneric(CelestialBody Body, double altitude, double speed)
        {
            if(!Body.atmosphere) return;
            Altitude = altitude-Body.Radius;
            Pressure = Body.GetPressure(Altitude);
            Speed = speed;
            if(Pressure > 0)
            {
                Atmosphere = true;
                AtmosphericTemperature = Body.GetTemperature(Altitude);
                Density = Body.GetDensity(Pressure, AtmosphericTemperature);
                DynamicPressure = 0.0005 * Density * Speed*Speed;

                var soundV = Body.GetSpeedOfSound(Pressure, Density);
                var mach = soundV > 0? Speed/soundV : 0;
                var convectiveMachLerp = Math.Pow(UtilMath.Clamp01((mach - PhysicsGlobals.NewtonianMachTempLerpStartMach) / 
                                                                   (PhysicsGlobals.NewtonianMachTempLerpEndMach - PhysicsGlobals.NewtonianMachTempLerpStartMach)), 
                                                  PhysicsGlobals.NewtonianMachTempLerpExponent);
                ShockTemperature = Speed * PhysicsGlobals.NewtonianTemperatureFactor;
                if(convectiveMachLerp > 0.0)
                {
                    double b = PhysicsGlobals.MachTemperatureScalar * Math.Pow(Speed, PhysicsGlobals.MachTemperatureVelocityExponent);
                    ShockTemperature = UtilMath.LerpUnclamped(ShockTemperature, b, convectiveMachLerp);
                }
                ShockTemperature *= (double)HighLogic.CurrentGame.Parameters.Difficulty.ReentryHeatScale * Body.shockTemperatureMultiplier;
                ShockTemperature = Math.Max(AtmosphericTemperature, ShockTemperature);
                //calculate convective coefficient for speed > Mach1; lower speed is not a concern
                ConvectiveCoefficient = 1E-10 * PhysicsGlobals.MachConvectionFactor;
                ConvectiveCoefficient *= Density > 1? Density : Math.Pow(Density, PhysicsGlobals.MachConvectionDensityExponent);
                ConvectiveCoefficient *= Math.Pow(Speed, PhysicsGlobals.MachConvectionVelocityExponent) * Body.convectionMultiplier;
            }
        }

        public override string ToString()
        {
            return Utils.Format("Altitude {} m, Density {}, Pressure {} kPa, Atm.T {} K\n" +
                                "Speed {} m/s, Dyn.Pressure {} kPa, Shock.T {} K\n" +
                                "ConvectiveCoefficient {}",
                                Altitude, Density, Pressure, AtmosphericTemperature, 
                                Speed, DynamicPressure, ShockTemperature, 
                                ConvectiveCoefficient);
        }
    }

	public class AtmosphericConditions
	{
		public double UT = -1;
		public double Duration;

		public double Altitude;
		public double Speed;

		public bool   Atmosphere;
		public double Pressure;
		public double Density;
		public double AtmosphericTemperature;

		public double DynamicPressure;
		public double ShockTemperature;
		public double ConvectiveCoefficient;

		public AtmosphericConditions(double UT)
		{ this.UT = UT; }


//		ptd.finalCoeff = this.convectiveCoefficient * ptd.convectionArea * 0.001 * part.heatConvectiveConstant * ptd.convectionCoeffMultiplier;
//		ptd.finalCoeff = Math.Min(ptd.finalCoeff, part.skinThermalMass * part.skinExposedAreaFrac);

		public AtmosphericConditions(Orbit orb, double UT) : this(UT)
		{
			var Body = orb.referenceBody;
			if(!Body.atmosphere) return;
			var pos = orb.getRelativePositionAtUT(UT);
			Altitude = pos.magnitude-Body.Radius;
			Pressure = Body.GetPressure(Altitude);
			if(Pressure > 0)
			{
				Speed = orb.getOrbitalSpeedAtRelativePos(pos);
				Atmosphere = true;
				AtmosphericTemperature = Body.GetTemperature(Altitude);
				Density = Body.GetDensity(Pressure, AtmosphericTemperature);
				DynamicPressure = 0.0005 * Density * Speed*Speed;

				var soundV = Body.GetSpeedOfSound(Pressure, Density);
				var mach = soundV > 0? Speed/soundV : 0;
				var convectiveMachLerp = Math.Pow(UtilMath.Clamp01((mach - PhysicsGlobals.NewtonianMachTempLerpStartMach) / 
				                                                   (PhysicsGlobals.NewtonianMachTempLerpEndMach - PhysicsGlobals.NewtonianMachTempLerpStartMach)), 
				                                      PhysicsGlobals.NewtonianMachTempLerpExponent);
				ShockTemperature = Speed * PhysicsGlobals.NewtonianTemperatureFactor;
				if (convectiveMachLerp > 0.0)
				{
					double b = PhysicsGlobals.MachTemperatureScalar * Math.Pow(Speed, PhysicsGlobals.MachTemperatureVelocityExponent);
					ShockTemperature = UtilMath.LerpUnclamped(ShockTemperature, b, convectiveMachLerp);
				}
				ShockTemperature *= (double)HighLogic.CurrentGame.Parameters.Difficulty.ReentryHeatScale * Body.shockTemperatureMultiplier;
				ShockTemperature = Math.Max(AtmosphericTemperature, ShockTemperature);
				//calculate convective coefficient for speed > Mach1; lower speed is not a concern
				ConvectiveCoefficient = 1E-10 * PhysicsGlobals.MachConvectionFactor;
				ConvectiveCoefficient *= Density > 1? Density : Math.Pow(Density, PhysicsGlobals.MachConvectionDensityExponent);
				ConvectiveCoefficient *= Math.Pow(Speed, PhysicsGlobals.MachConvectionVelocityExponent) * Body.convectionMultiplier;
			}
		}

		public override string ToString()
		{
			return Utils.Format("Altitude {} m, Density {}, Pressure {} kPa, Atm.T {} K\n" +
			                    "Speed {} m/s, Dyn.Pressure {} kPa, Shock.T {} K\n" +
			                    "Duration {} s, ConvectiveCoefficient {}",
			                    Altitude, Density, Pressure, AtmosphericTemperature, 
			                    Speed, DynamicPressure, ShockTemperature, 
			                    Duration, ConvectiveCoefficient);
		}
	}
}

