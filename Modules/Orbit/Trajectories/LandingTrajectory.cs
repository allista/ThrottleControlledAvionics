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
using System.Linq;
using System.Collections.Generic;
using UnityEngine;
using AT_Utils;

namespace ThrottleControlledAvionics
{
	public class LandingTrajectory : TargetedTrajectory
	{
		public double TargetAltitude;
        public double ActualLandingAngle { get; private set; }
        public double LandingSteepness { get; private set; }
		public WayPoint SurfacePoint { get; private set; }

        public AtmosphericTrajoctory AtmoTrajectory { get; private set; }

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
        public double FlyAboveUT { get; private set; }
        public double AerobrakeStartUT { get; private set; }
        public double AerobrakeStopUT { get; private set; }
        public AtmosphericTrajoctory AfterBrakeTrajectory { get; private set; }
        public AtmosphericTrajoctory.Point BrakePoint { get; private set; }
        public double MaxDynamicPressure { get; private set; }
        public double MaxShipTemperature { get; private set; }
        public bool WillOverheat { get; private set; }

        public override float GetTotalFuel()
        {
            return base.GetTotalFuel() + 
                VSL.Engines.FuelNeededAtAlt((float)AtTargetVel.magnitude, (float)TargetAltitude);
        }

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
            approach = AtmoTrajectory != null?
                new Coordinates((at_target_rot*AtmoTrajectory.PointAtUT(BrakeEndUT-1).pos).xzy+Body.position, Body) :
                new Coordinates((at_target_rot*Orbit.getRelativePositionAtUT(BrakeEndUT-1)).xzy+Body.position, Body);
            SurfacePoint = new WayPoint((at_target_rot*AtTargetPos).xzy+Body.position, Body);
            SurfacePoint.Pos.SetAlt2Surface(Body);
            SurfacePoint.Name = "Landing Site";
        }

        void update_overheat_info(AtmosphericTrajoctory trj, double startT)
        {
            trj.UpdateOverheatInto(startT);
            MaxShipTemperature = trj.MaxShipTemperature;
            MaxDynamicPressure = Math.Max(MaxDynamicPressure, trj.MaxDynamicPressure/1000);
            WillOverheat = MaxShipTemperature > VSL.Physics.MinMaxTemperature;
        }

        void update_landing_site_after_brake(Vector3d brake_pos, Vector3d brake_vel)
        {
            SetBrakeDeltaV(LandingTrajectoryAutopilot.CorrectedBrakeVelocity(VSL, brake_vel, brake_pos, 
                                                                             BrakePoint.DynamicPressure / 1000 / GLB.LTRJ.MinDPressure,
                                                                             AtTargetUT-BrakePoint.UT));
            BrakeStartUT = Math.Max(BrakeEndUT-MatchVelocityAutopilot.BrakingOffset((float)BrakeDeltaV.magnitude, VSL, out BrakeDuration), StartUT);
            Vector3d pos, vel;
            if(AtmoTrajectory != null)
            {
                pos = BrakePoint.pos;
                vel = BrakePoint.vel+BrakeDeltaV;
            }
            else
            {
                pos = Orbit.getRelativePositionAtUT(BrakeEndUT);
                vel = Orbit.getOrbitalVelocityAtUT(BrakeEndUT)+BrakeDeltaV;
            }
            AfterBrakeTrajectory = new AtmosphericTrajoctory(VSL, 
                                                             TrajectoryCalculator.NewOrbit(Body, pos, vel, BrakeEndUT), 
                                                             TargetAltitude, 
                                                             BrakeEndUT, 
                                                             GLB.LTRJ.AtmoTrajectoryResolution);
            if(AtmoTrajectory != null)
                update_overheat_info(AfterBrakeTrajectory, BrakePoint.ShipTemperature);
            AtTargetVel = AfterBrakeTrajectory.LastPoint.vel;
            AtTargetPos = AfterBrakeTrajectory.LastPoint.pos;
            AtTargetUT = AfterBrakeTrajectory.LastPoint.UT;
            update_landing_site();
        }

		void update_from_orbit(Orbit orb, double UT)
		{
			//calculate the position of a landing site
			if(orb.ApA <= TargetAltitude) 
            {
				AtTargetUT = orb.StartUT+(orb.ApAhead()? orb.timeToAp : 1);
                AtTargetPos = orb.getRelativePositionAtUT(AtTargetUT);
                AtTargetVel = orb.getOrbitalVelocityAtUT(AtTargetUT);
            }
            else if(Body.atmosphere && TargetAltitude < Body.atmosphereDepth)
            {
                AtTargetUT = orb.altitude > Body.atmosphereDepth?
                    TrajectoryCalculator.NearestRadiusUT(orb, Body.Radius+Body.atmosphereDepth, UT)+1 : UT;
                var end_alt = Math.Max(orb.PeA+10, TargetAltitude);
                AtmoTrajectory = new AtmosphericTrajoctory(VSL, orb, end_alt, 
                                                           AtTargetUT,
                                                           GLB.LTRJ.AtmoTrajectoryResolution);
                update_overheat_info(AtmoTrajectory, VSL.TCA.part.temperature);
                AtTargetVel = AtmoTrajectory.LastPoint.vel;
                AtTargetPos = AtmoTrajectory.LastPoint.pos;
                AtTargetUT = AtmoTrajectory.LastPoint.UT;
            }
            else
            {
                if(orb.PeA < TargetAltitude) 
                    AtTargetUT = TrajectoryCalculator.NearestRadiusUT(orb, Body.Radius+TargetAltitude, UT);
                else AtTargetUT = orb.StartUT+orb.timeToPe;
                AtTargetPos = orb.getRelativePositionAtUT(AtTargetUT);
                AtTargetVel = orb.getOrbitalVelocityAtUT(AtTargetUT);
            }
			TransferTime = AtTargetUT-StartUT;
            update_landing_site();
		}

		void ClampBrakeDeltaV()
		{
            BrakeFuel = 0;
            FullBrake = true;
			var dVm = BrakeDeltaV.magnitude;
			if(dVm > 0) 
			{
                var fuel = VSL.Engines.AvailableFuelMass-ManeuverFuel-VSL.Engines.MaxMassFlow*GLB.LTRJ.LandingThrustTime;
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

        void SetBrakePoint(AtmosphericTrajoctory.Point p)
        {
            BrakePoint = p;
            BrakeEndUT = BrakePoint.UT;
            BrakeEndDeltaAlt = BrakePoint.Altitude-TargetAltitude;
        }

		void SetBrakeEndUT(double UT)
		{ 
			BrakeEndUT = UT;
            if(AtmoTrajectory != null)
                SetBrakePoint(AtmoTrajectory.PointAtUT(UT));
            else
                BrakeEndDeltaAlt = Orbit.getRelativePositionAtUT(BrakeEndUT).magnitude-Body.Radius-TargetAltitude; 
		}

		void SetBrakeDeltaV(Vector3d dV)
		{
			BrakeDeltaV = dV;
			ClampBrakeDeltaV();
		}

        void update_landing_steepness()
        {
            if(Orbit.PeA >= TargetAltitude)
                LandingSteepness = 0;
            else
            {
                var tA = Orbit.TrueAnomalyAtRadius(Body.Radius+TargetAltitude);
                var cosA = Math.Cos(tA);
                var sinA = Math.Sin(tA);
                var velN = Math.Sqrt(1+2*Orbit.eccentricity*cosA + Orbit.eccentricity * Orbit.eccentricity);
                var pos2vel_cos = Utils.Clamp(sinA*Orbit.eccentricity/velN, -1, 1);
                LandingSteepness = 90-Math.Acos(pos2vel_cos)/Math.PI*180;
            }
        }

		void update(bool with_brake)
		{
            //reset state
            AtmoTrajectory = null;
            AfterBrakeTrajectory = null;
            WillOverheat = false;
            MaxDynamicPressure = -1;
            MaxShipTemperature = -1;
            //update everything
            update_from_orbit(Orbit, StartUT);
            update_landing_steepness();
            ActualLandingAngle = AtmoTrajectory != null?
                90-Vector3d.Angle(AtTargetPos, -AtTargetVel) :
                LandingSteepness;
            AerobrakeStartUT = AtmoTrajectory != null? AtmoTrajectory.AtmoStartUT : AtTargetUT-1;
            AerobrakeStopUT = AtmoTrajectory != null? AtmoTrajectory.AtmoStopUT : AtTargetUT-1;
			//correct for brake maneuver
			if(with_brake)
			{
				//estimate time needed to rotate the ship downwards
				var rotation_time = VSL.Torque.NoEngines? 
                    VSL.Torque.NoEngines.RotationTime2Phase(90) :
					VSL.Torque.MaxPossible.RotationTime2Phase(90, 0.1f);
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
                        var brake_end_UT = Math.Max(AtTargetUT-Mathf.Max(GLB.LTRJ.CorrectionOffset, BrakeDuration*1.1f), StartUT);
                        //find appropriate point to perform the maneuver
                        if(AtmoTrajectory != null)
                        {
                            var fly_over_alt = TargetAltitude+GLB.LTRJ.FlyOverAlt;
                            if(WillOverheat)
                                SetBrakePoint(AtmoTrajectory.PointAtShipTemp(VSL.Physics.MinMaxTemperature-100));
                            else if(AtmoTrajectory.UT2Altitude(brake_end_UT) < fly_over_alt)
                                SetBrakePoint(AtmoTrajectory.PointAtAltitude(fly_over_alt));
                            else 
                                SetBrakeEndUT(brake_end_UT);
                        }
                        else
                        {
                            var vertical_speed = vertical_vel.magnitude;
                            double fly_over_error;
                            do {
                                SetBrakeEndUT(brake_end_UT);
                                fly_over_error = BrakeEndDeltaAlt - GLB.LTRJ.FlyOverAlt;
                                brake_end_UT -= Math.Abs(fly_over_error/vertical_speed);
                            } while(brake_end_UT > StartUT && fly_over_error < -1);
                        }
                    }
                    else 
                        SetBrakeEndUT(AerobrakeStartUT);
                    //update landing site
                    if(AtmoTrajectory != null)
                        update_landing_site_after_brake(BrakePoint.pos, BrakePoint.vel);
                    else
                        update_landing_site_after_brake(Orbit.getRelativePositionAtUT(BrakeEndUT), 
                                                        Orbit.getOrbitalVelocityAtUT(BrakeEndUT));
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
                if(AtmoTrajectory != null)
                {
                    var FlyAbovePoint = AtmoTrajectory.FlyAbovePoint(TrajectoryCalculator
                                                                     .BodyRotationAtdT(Body, AtmoTrajectory.StartUT-VSL.Physics.UT) *
                                                                     Target.RelOrbPos(Body));
                    FlyAboveUT = FlyAbovePoint.UT;
                    if(WillOverheat)
                        SetBrakePoint(AtmoTrajectory.PointAtShipTemp(VSL.Physics.MinMaxTemperature-100));
                    else
                        SetBrakePoint(FlyAbovePoint);
                }
                else 
                {
                    FlyAboveUT = TrajectoryCalculator.FlyAboveUT(Orbit, 
                                                                 TrajectoryCalculator
                                                                 .BodyRotationAtdT(Body, TimeToStart) *
                                                                 Target.RelOrbPos(Body), 
                                                                 StartUT);
                    SetBrakeEndUT(FlyAboveUT);
                }
				SetBrakeDeltaV(-AtTargetVel-Vector3d.Cross(Body.zUpAngularVelocity, AtTargetPos));
				if(BrakeFuel > 0)
				{
					var offset = MatchVelocityAutopilot.BrakingOffset((float)BrakeDeltaV.magnitude, VSL, out BrakeDuration);
					BrakeStartUT = Math.Max(BrakeEndUT-offset, StartUT);
				}
				else
				{
                    SetBrakeEndUT(AerobrakeStartUT);
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
                                        TrajectoryCalculator.BodyRotationAtdT(Body, TimeToTarget) * Target.RelOrbPos(Body));
			DeltaR = Utils.RadDelta(SurfacePoint.AngleTo(VslStartLat, VslStartLon), Target.AngleTo(VslStartLat, VslStartLon))*Mathf.Rad2Deg;
//            Utils.Log("{}", this);//debug
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
				             "Delta Lat: {} deg\n" +
                             "Delta Lon: {} deg\n\n" +

                             "Fly Above UT {}\n" +
                             "Brake Start UT {}\n" +
                             "Brake End UT {}\n" +
                             "Brake Offset {}\n" +
				             "Time to Brake {} s\n" +
				             "BrakeEnd Altitude {} m\n\n" +

				             "Landing Angle {} deg\n" +
                             "Landing Steepness {} deg\n" +
                             "WillOverheat {}\n" +
                             "MaxShipTemp {}\n" +
                             "MaxDynPressure {}\n\n" +

                             "AtmoTrajectory:\n{}\n\n" +

                             "AfterBrakeTrajectory:\n{}\n",
				             SurfacePoint,
                             DeltaR, DeltaLat, DeltaLon, FlyAboveUT,
                             BrakeStartUT, BrakeEndUT, BrakeOffset, BrakeStartUT-VSL.Physics.UT, 
                             BrakeEndDeltaAlt, ActualLandingAngle, LandingSteepness,
                             WillOverheat, MaxShipTemperature, MaxDynamicPressure,
                             AtmoTrajectory, AfterBrakeTrajectory);
		}
	}

    public class AtmosphericTrajoctory
    {
        public struct Point
        {
            public VesselWrapper VSL;
            public Orbit Orbit;

            public double UT;
            public double Duration;

            public Vector3d vel, srf_vel, pos;

            public double Altitude;
            public bool   Atmosphere;
            public double Pressure;
            public double AtmosphereTemperature;
            public double Density;
            public double Mach1;

            /// <summary>
            /// The dynamic pressure in Pa
            /// </summary>
            public double DynamicPressure;
            public double SrfSpeed;
            public double Mach;
            public double SpecificDrag;

            public double ShockTemperature;
            public double ConvectiveCoefficient;
            public double ShipTemperature;

            public CelestialBody Body { get { return Orbit.referenceBody; } }

            public void Update(double UT)
            {
                this.UT = UT;
                pos = Orbit.getRelativePositionAtUT(UT);
                vel = Orbit.getOrbitalVelocityAtUT(UT);
                Update();
            }

            public void Update()
            {
                Altitude = pos.magnitude-Body.Radius;
                Atmosphere = Altitude < Body.atmosphereDepth;

                if(Atmosphere)
                {
                    Pressure = Body.GetPressure(Altitude);
                    AtmosphereTemperature = Body.GetTemperature(Altitude);
                    Density = Body.GetDensity(Pressure, AtmosphereTemperature);
                    Mach1 = Body.GetSpeedOfSound(Pressure, Density);

                    srf_vel = vel+Vector3d.Cross(Body.zUpAngularVelocity, pos);
                    SrfSpeed = srf_vel.magnitude;
                    var Rho_v = Density*SrfSpeed;
                    DynamicPressure = Rho_v * SrfSpeed;
                    Mach = SrfSpeed/Mach1;
                    this.SpecificDrag = AtmoSim.Cd * DynamicPressure *
                        PhysicsGlobals.DragCurveMultiplier.Evaluate((float)Mach) *
                        PhysicsGlobals.DragCurvePseudoReynolds.Evaluate((float)(Rho_v));

                    var convectiveMachLerp = Math.Pow(UtilMath.Clamp01((Mach - PhysicsGlobals.NewtonianMachTempLerpStartMach) / 
                                                                       (PhysicsGlobals.NewtonianMachTempLerpEndMach - PhysicsGlobals.NewtonianMachTempLerpStartMach)), 
                                                      PhysicsGlobals.NewtonianMachTempLerpExponent);
                    ShockTemperature = SrfSpeed * PhysicsGlobals.NewtonianTemperatureFactor;
                    if (convectiveMachLerp > 0.0)
                    {
                        double b = PhysicsGlobals.MachTemperatureScalar * Math.Pow(SrfSpeed, PhysicsGlobals.MachTemperatureVelocityExponent);
                        ShockTemperature = UtilMath.LerpUnclamped(ShockTemperature, b, convectiveMachLerp);
                    }
                    ShockTemperature *= (double)HighLogic.CurrentGame.Parameters.Difficulty.ReentryHeatScale * Body.shockTemperatureMultiplier;
                    ShockTemperature = Math.Max(AtmosphereTemperature, ShockTemperature);
                    //calculate convective coefficient for speed > Mach1; lower speed is not a concern
                    ConvectiveCoefficient = 1E-10 * PhysicsGlobals.MachConvectionFactor;
                    ConvectiveCoefficient *= Density > 1? Density : Math.Pow(Density, PhysicsGlobals.MachConvectionDensityExponent);
                    ConvectiveCoefficient *= Math.Pow(SrfSpeed, PhysicsGlobals.MachConvectionVelocityExponent) * Body.convectionMultiplier;
                }
                else AtmosphereTemperature = -273;
            }

            public double UpdateShipTemp(double startT)
            {
                var K = VSL.Physics.MMT_ThermalMass > ConvectiveCoefficient? 
                        -ConvectiveCoefficient/VSL.Physics.MMT_ThermalMass : -1;
                ShipTemperature = ShockTemperature + (startT-ShockTemperature) * Math.Exp(K*Globals.Instance.LTRJ.HeatingCoefficient*Duration);
                return ShipTemperature;
            }

            public override string ToString()
            {
                return Utils.Format("Altitude {} m\n" +
                                    "Density {}, Pressure {} kPa, Atm.T {} K\n" +
                                    "SrfSpeed {} m/s, Dyn.Pressure {} kPa, Shock.T {} K\n" +
                                    "ConvectiveCoefficient {}, Ship.T {} K\n" +
                                    "UT {}, Duration {} s\n",
                                    Altitude, Density, Pressure, AtmosphereTemperature, 
                                    SrfSpeed, DynamicPressure/1000, ShockTemperature, 
                                    ConvectiveCoefficient, ShipTemperature, UT, Duration);
            }
        }

        public readonly VesselWrapper VSL;
        public readonly Orbit Orbit;
        public readonly double TargetAltitude;

        public List<Point> Points = new List<Point>();
        public Point LastPoint { get; private set; }
        public bool Atmosphere { get; private set; }
        public double StartUT = -1;
        public double AtmoStartUT = -1;
        public double AtmoStopUT = -1;
        public double EndUT = -1;
        public double MaxShipTemperature = -1;
        public double MaxDynamicPressure = -1;

        Point newP(double UT)
        { 
            var p = new Point{VSL=VSL, Orbit=Orbit}; 
            p.Update(UT); 
            return p; 
        }

        public AtmosphericTrajoctory(VesselWrapper vsl, Orbit orb, double target_altitude, double startUT, double dt)
        {
            VSL = vsl;
            Orbit = orb;
            TargetAltitude = target_altitude;
            StartUT = startUT;
            EndUT = startUT+orb.timeToPe;
            var p = newP(StartUT);
            var m = (double)VSL.Physics.M;
            var s = (double)VSL.Geometry.BoundsSideAreas.MinComponentF();
            Atmosphere = false;
            p.Duration = dt;
            while(p.Altitude > TargetAltitude && p.UT < EndUT)
            {
                var drag_dv = p.SpecificDrag*s/m*dt;
                Atmosphere |= drag_dv > 0;
                if(Atmosphere)
                {
                    if(Points.Count == 0)
                        AtmoStartUT = p.UT;
                    Points.Add(p);
                    var r = p.pos.magnitude;
                    p.vel -= p.pos*p.Body.gMagnitudeAtCenter/r/r/r*dt + p.srf_vel/p.SrfSpeed*Math.Min(drag_dv, p.SrfSpeed);
                    p.pos += p.vel*dt;
                    p.UT += dt;
                    p.Update();
                    if(AtmoStopUT < 0 && p.SrfSpeed < 10)
                        AtmoStopUT = p.UT;
                }
                else p.Update(p.UT+dt);
                if(dt > 0.01) 
                {
                    var dAlt = p.Altitude-TargetAltitude;
                    if(dAlt/p.SrfSpeed < dt)
                    {
                        dt = dAlt/p.SrfSpeed*0.9;
                        p.Duration = dt;
                    }
                }
            }
            if(Atmosphere) 
            {
                Points.Add(p);
                if(AtmoStopUT < 0)
                    AtmoStopUT = p.UT;
            }
            EndUT = p.UT;
            LastPoint = p;
        }

        public void UpdateOverheatInto(double startT)
        {
            if(!Atmosphere) return;
            MaxShipTemperature = startT;
            for(int i = 0, count = Points.Count; i < count; i++)
            {
                var p = Points[i];
                startT = p.UpdateShipTemp(startT);
                if(startT > MaxShipTemperature) 
                    MaxShipTemperature = startT;
                if(p.DynamicPressure > MaxDynamicPressure)
                    MaxDynamicPressure = p.DynamicPressure;
                Points[i] = p;
            }
        }

        public double Altitude2UT(double altitude)
        {
            if(Points.Count == 0 || altitude > Points[0].Altitude)
                return TrajectoryCalculator.NearestRadiusUT(Orbit, altitude+Orbit.referenceBody.Radius, StartUT);
            for(int i = 1, count = Points.Count; i < count; i++)
            {
                var p1 = Points[i];
                if(p1.Altitude > altitude) continue;
                var p0 = Points[i-1];
                return p0.UT + p0.Duration * (altitude-p0.Altitude)/(p1.Altitude-p0.Altitude);
            }
            return EndUT;
        }

        public double UT2Altitude(double UT)
        {
            if(Points.Count == 0 || UT < AtmoStartUT)
                return Orbit.RadiusAtTrueAnomaly(Orbit.TrueAnomalyAtUT(UT))-Orbit.referenceBody.Radius;
            for(int i = 1, count = Points.Count; i < count; i++)
            {
                var p1 = Points[i];
                if(p1.UT < UT) continue;
                var p0 = Points[i-1];
                var t = (UT-p0.UT)/(p0.Duration);
                return p0.Altitude + (p1.Altitude-p0.Altitude)*t;
            }
            return TargetAltitude;
        }

        public Point PointAtShipTemp(double T)
        {
            if(Points.Count == 0)
                return newP(StartUT);
            for(int i = 1, count = Points.Count; i < count; i++)
            {
                var p1 = Points[i];
                if(p1.ShipTemperature < T) continue;
                var p0 = Points[i-1];
                var p = p0;
                var t = (T-p0.ShipTemperature)/(p1.ShipTemperature-p0.ShipTemperature);
                p.UT = p0.UT+p0.Duration*t;
                p.Duration = p1.UT-p.UT;
                p.pos = Vector3d.Lerp(p0.pos, p1.pos, t);
                p.ShipTemperature = T;
                p.Update();
                return p;
            }
            return LastPoint;
        }

        public Point PointAtUT(double UT)
        {
            if(Points.Count == 0 || UT < AtmoStartUT)
                return newP(UT);
            for(int i = 1, count = Points.Count; i < count; i++)
            {
                var p1 = Points[i];
                if(p1.UT < UT) continue;
                var p0 = Points[i-1];
                var p = p0;
                var t = (UT-p0.UT)/(p0.Duration);
                p.UT = UT;
                p.Duration = p1.UT-UT;
                p.pos = Vector3d.Lerp(p0.pos, p1.pos, t);
                p.Update();
                return p;
            }
            return LastPoint;
        }

        public Point PointAtAltitude(double altitude)
        {
            if(Points.Count == 0 || altitude > Points[0].Altitude)
                return newP(TrajectoryCalculator.NearestRadiusUT(Orbit, altitude+Orbit.referenceBody.Radius, StartUT));
            for(int i = 1, count = Points.Count; i < count; i++)
            {
                var p1 = Points[i];
                if(p1.Altitude > altitude) continue;
                var p0 = Points[i-1];
                var p = p0;
                var t = (p0.Altitude-altitude)/(p0.Altitude-p1.Altitude);
                p.UT += p0.Duration*t;
                p.Duration = p1.UT-p.UT;
                p.pos = Vector3d.Lerp(p0.pos, p1.pos, t);
                p.Update();
                return p;
            }
            return LastPoint;
        }

        public Point FlyAbovePoint(Vector3d pos)
        {
            if(Utils.ProjectionAngle(Orbit.getRelativePositionAtUT(StartUT), pos, 
                                     Orbit.getOrbitalVelocityAtUT(StartUT)) < 0)
                return newP(StartUT);
            if(Points.Count == 0 ||
               Utils.ProjectionAngle(Orbit.getRelativePositionAtUT(AtmoStartUT), 
                                     TrajectoryCalculator.BodyRotationAtdT(Orbit.referenceBody, AtmoStartUT-StartUT)*pos, 
                                     Orbit.getOrbitalVelocityAtUT(AtmoStartUT)) < 0)
                return newP(TrajectoryCalculator.FlyAboveUT(Orbit, pos, StartUT));
            var p0 = Points[0];
            var angle0 = Utils.ProjectionAngle(p0.pos, 
                                               TrajectoryCalculator
                                               .BodyRotationAtdT(Orbit.referenceBody, p0.UT-StartUT)*pos, 
                                               p0.vel);
            for(int i = 1, count = Points.Count; i < count; i++)
            {
                var p1 = Points[i];
                var angle1 = Utils.ProjectionAngle(p1.pos, 
                                                   TrajectoryCalculator
                                                   .BodyRotationAtdT(Orbit.referenceBody, p1.UT-StartUT)*pos, 
                                                   p1.vel);
                if(angle1 > 0) 
                {
                    angle0 = angle1;
                    continue;
                }
                p0 = Points[i-1];
                var p = p0;
                var t = angle0/(angle0-angle1);
                p.UT += p0.Duration*t;
                p.Duration = p1.UT-p.UT;
                p.pos = Vector3d.Lerp(p0.pos, p1.pos, t);
                p.Update();
                return p;
            }
            return LastPoint;
        }

        public Vector3[] ToCBFramePath(double UT)
        {
            return Points
                .Select(p => (Vector3)((TrajectoryCalculator
                                        .BodyRotationAtdT(Orbit.referenceBody, UT-p.UT) * p.pos).xzy
                                       +Orbit.referenceBody.position))
                .ToArray();
        }

        public override string ToString()
        {
            return Utils.Format("StartUT {}\n" +
                                "AtmoStartUT {}\n" +
                                "AtmoStopUT {}\n" +
                                "EndUT {}\n" +
                                "MaxShipT {}\n" +
                                "MaxDynP {}\n" +
                                "Points: {}", 
                                StartUT, AtmoStartUT, AtmoStopUT, EndUT,
                                MaxShipTemperature, MaxDynamicPressure/1000,
                                Points);
        }
    }
}

