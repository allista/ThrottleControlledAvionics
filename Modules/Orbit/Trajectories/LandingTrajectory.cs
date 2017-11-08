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

        public LandingPath Path { get; private set; }

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

        public LandingPath AfterBrakePath { get; private set; }

        public LandingPath.Point FlyAbovePoint { get; private set; }

        public LandingPath.Point BrakeStartPoint { get; private set; }

        public LandingPath.Point BrakeEndPoint { get; private set; }

        public double BrakeEndPointDeltaAlt { get; private set; }

        public float BrakeOffset;

        public double MaxDynamicPressure { get; private set; }

        public double MaxShipTemperature { get; private set; }

        public bool WillOverheat { get; private set; }

        public override float GetTotalFuel()
        {
            return base.GetTotalFuel() +
            VSL.Engines.FuelNeededAtAlt((float)AtTargetVel.magnitude, (float)TargetAltitude);
        }

        public LandingTrajectory(VesselWrapper vsl, Vector3d dV, double startUT, 
                                 WayPoint target, double target_altitude = 0, 
                                 bool with_brake = true, bool brake_at_fly_above = false)
            : base(vsl, dV, startUT, target)
        {
            TargetAltitude = target_altitude;
            update(with_brake, brake_at_fly_above);
        }

        void update_landing_site(LandingPath path)
        {
            var at_target_rot = TrajectoryCalculator.BodyRotationAtdT(Body, -TimeToTarget);
            approach = new Coordinates((at_target_rot * path.PointAtUT(path.EndUT - 1).pos).xzy + Body.position, Body);
            SurfacePoint = new WayPoint((at_target_rot * AtTargetPos).xzy + Body.position, Body);
            SurfacePoint.Pos.SetAlt2Surface(Body);
            SurfacePoint.Name = "Landing Site";
        }

        void update_overheat_info(LandingPath trj, double startT)
        {
            trj.UpdateOverheatInto(startT);
            MaxShipTemperature = trj.MaxShipTemperature;
            MaxDynamicPressure = Math.Max(MaxDynamicPressure, trj.MaxDynamicPressure / 1000);
            WillOverheat = MaxShipTemperature > VSL.Physics.MinMaxTemperature;
        }

        void update_landing_site_after_brake()
        {
            SetBrakeDeltaV(-LandingTrajectoryAutopilot
                           .CorrectedBrakeVelocity(VSL, BrakeEndPoint.vel, BrakeEndPoint.pos, 
                                                   BrakeEndPoint.DynamicPressure / 1000 / GLB.LTRJ.MinDPressure,
                                                   AtTargetUT - BrakeEndPoint.UT));
            BrakeStartPoint = Path.PointAtUT(Math.Max(BrakeEndPoint.UT - MatchVelocityAutopilot
                                                      .BrakingOffset((float)BrakeDeltaV.magnitude, VSL, out BrakeDuration), StartUT));
            AfterBrakePath = new LandingPath(VSL, 
                                             BrakeStartPoint.OrbitFromHere(),
                                             TargetAltitude, 
                                             BrakeStartPoint.UT, 
                                             (AtTargetUT - BrakeStartPoint.UT) / 20,
                                             VSL.Physics.M - ManeuverFuel,
                                             VSL.Engines.AvailableFuelMass - ManeuverFuel,
                                             BrakeDeltaV.magnitude);
            if(Path.Atmosphere)
                update_overheat_info(AfterBrakePath, BrakeStartPoint.ShipTemperature);
            AtTargetVel = AfterBrakePath.LastPoint.vel;
            AtTargetPos = AfterBrakePath.LastPoint.pos;
            AtTargetUT = AfterBrakePath.LastPoint.UT;
            TransferTime = AtTargetUT - StartUT;
            update_landing_site(AfterBrakePath);
        }

        void update_from_orbit()
        {
            AtTargetUT = Body.atmosphere && Orbit.altitude > Body.atmosphereDepth ?
                TrajectoryCalculator.NearestRadiusUT(Orbit, Body.Radius + Body.atmosphereDepth, StartUT) + 1 : StartUT;
            var end_alt = Math.Max(Orbit.PeA + 10, TargetAltitude);
            Path = new LandingPath(VSL, Orbit, end_alt, 
                                   AtTargetUT,
                                   Math.Min(GLB.LTRJ.AtmoTrajectoryResolution,
                                            Utils.ClampL((VSL.Altitude.Absolute - TargetAltitude) / Math.Abs(VSL.VerticalSpeed.Absolute) / 20, 0.1)),
                                   VSL.Physics.M - ManeuverFuel);
            update_overheat_info(Path, VSL.TCA.part.temperature);
            AtTargetVel = Path.LastPoint.vel;
            AtTargetPos = Path.LastPoint.pos;
            AtTargetUT = Path.LastPoint.UT;
            TransferTime = AtTargetUT - StartUT;
            update_landing_site(Path);
        }

        void ClampBrakeDeltaV()
        {
            BrakeFuel = 0;
            FullBrake = true;
            var dVm = BrakeDeltaV.magnitude;
            if(dVm > 0)
            {
                var fuel = VSL.Engines.AvailableFuelMass - ManeuverFuel - VSL.Engines.MaxMassFlow * GLB.LTRJ.LandingThrustTime;
                if(fuel <= 0)
                    BrakeDeltaV = Vector3d.zero;
                else
                {
                    BrakeFuel = VSL.Engines
						.FuelNeededAtAlt((float)dVm, (float)(BrakeEndPointDeltaAlt + TargetAltitude));
                    if(BrakeFuel > fuel)
                    {
                        BrakeDeltaV = BrakeDeltaV * VSL.Engines.DeltaV((float)fuel) / dVm;
                        BrakeFuel = fuel;
                        FullBrake = false;
                    }
                }
            }
        }

        void SetBrakeEndPoint(LandingPath.Point p)
        {
            BrakeEndPoint = p;
            BrakeEndPointDeltaAlt = BrakeEndPoint.Altitude - TargetAltitude;
        }

        void SetBrakeEndUT(double UT)
        { 
            SetBrakeEndPoint(Path.PointAtUT(UT));
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
                var tA = Orbit.TrueAnomalyAtRadius(Body.Radius + TargetAltitude);
                var cosA = Math.Cos(tA);
                var sinA = Math.Sin(tA);
                var velN = Math.Sqrt(1 + 2 * Orbit.eccentricity * cosA + Orbit.eccentricity * Orbit.eccentricity);
                var pos2vel_cos = Utils.Clamp(sinA * Orbit.eccentricity / velN, -1, 1);
                LandingSteepness = 90 - Math.Acos(pos2vel_cos) / Math.PI * 180;
            }
        }

        void reset()
        {
            Path = null;
            AfterBrakePath = null;
            WillOverheat = false;
            MaxDynamicPressure = -1;
            MaxShipTemperature = -1;
        }

        void update(bool with_brake, bool brake_at_fly_above)
        {
            reset();
            //update everything
            update_from_orbit();
            update_landing_steepness();
            ActualLandingAngle = 90 - Utils.Angle2(AtTargetPos, -AtTargetVel);
            var AerobrakeStartUT = Path.Atmosphere ? Path.AtmoStartUT : AtTargetUT - 1;
            //correct for brake maneuver
            if(with_brake)
            {
                //estimate time needed to rotate the ship downwards
                var rotation_time = VSL.Torque.NoEngines ? 
                    VSL.Torque.NoEngines.RotationTime2Phase(90) :
					VSL.Torque.MaxPossible.RotationTime2Phase(90, 0.1f);
                //estimate amount of fuel needed for the maneuver
                var vertical_vel = Vector3d.Project(AtTargetVel, AtTargetPos);
                SetBrakeEndUT(Math.Max(AtTargetUT - GLB.LTRJ.CorrectionOffset + rotation_time, StartUT));
                SetBrakeDeltaV(vertical_vel);
                if(BrakeFuel > 0)
                {
                    //calculate braking maneuver
                    BrakeDuration = VSL.Engines.OnPlanetTTB(BrakeDeltaV, 
                                                            Orbit.getRelativePositionAtUT(BrakeEndPoint.UT), 
                                                            (float)(BrakeEndPointDeltaAlt + TargetAltitude));
                    BrakeDuration += rotation_time;
                    if(FullBrake)
                    {
                        if(brake_at_fly_above)
                        {
                            FlyAbovePoint = Path.FlyAbovePoint(Target.OrbPos(Body));
                            if(WillOverheat)
                                SetBrakeEndPoint(Path.PointAtShipTemp(VSL.Physics.MinMaxTemperature - 100));
                            else
                                SetBrakeEndPoint(FlyAbovePoint);
                        }
                        else
                        {
                            //find appropriate point to perform the maneuver
                            var brake_end_UT = Math.Max(AtTargetUT - Mathf.Max(GLB.LTRJ.CorrectionOffset, BrakeDuration * 1.1f), StartUT);
                            var fly_over_alt = TargetAltitude + GLB.LTRJ.FlyOverAlt;
                            if(WillOverheat)
                                SetBrakeEndPoint(Path.PointAtShipTemp(VSL.Physics.MinMaxTemperature - 100));
                            else if(Path.UT2Altitude(brake_end_UT) < fly_over_alt)
                                SetBrakeEndPoint(Path.PointAtAltitude(fly_over_alt));
                            else
                                SetBrakeEndUT(brake_end_UT);
                        }
                    }
                    else
                        SetBrakeEndUT(AerobrakeStartUT);
                    //update landing site
                    update_landing_site_after_brake();
                }
                else //no brake maneuver
                {
                    SetBrakeEndUT(AerobrakeStartUT);
                    BrakeStartPoint = BrakeEndPoint;
                    BrakeDuration = 0;
                }
            }
            else
            {
                FlyAbovePoint = Path.FlyAbovePoint(Target.OrbPos(Body));
                if(WillOverheat)
                    SetBrakeEndPoint(Path.PointAtShipTemp(VSL.Physics.MinMaxTemperature - 100));
                else
                    SetBrakeEndPoint(FlyAbovePoint);
                SetBrakeDeltaV(-AtTargetVel - Vector3d.Cross(Body.zUpAngularVelocity, AtTargetPos));
                if(BrakeFuel > 0)
                {
                    var offset = MatchVelocityAutopilot.BrakingOffset((float)BrakeDeltaV.magnitude, VSL, out BrakeDuration);
                    BrakeStartPoint = Path.PointAtUT(Math.Max(BrakeEndPoint.UT - offset, StartUT));
                }
                else
                {
                    SetBrakeEndUT(AerobrakeStartUT);
                    BrakeStartPoint = BrakeStartPoint;
                    BrakeDuration = 0;
                }
            }
            BrakeOffset = (float)Utils.ClampL(BrakeEndPoint.UT - BrakeStartPoint.UT, 0);
            //compute vessel coordinates at maneuver start
            if(VSL.LandedOrSplashed)
            {
                VslStartLat = Utils.ClampAngle(VSL.vessel.latitude);
                VslStartLon = Utils.ClampAngle(VSL.vessel.longitude);
            }
            else
            {
                var start_pos = (TrajectoryCalculator.BodyRotationAtdT(Body, -TimeToStart) * StartPos).xzy + Body.position;
                VslStartLat = Utils.ClampAngle(Body.GetLatitude(start_pos));
                VslStartLon = Utils.ClampAngle(Body.GetLongitude(start_pos));
            }
            //compute distance to target
            DistanceToTarget = Target.AngleTo(SurfacePoint) * Body.Radius;
            SurfacePoint.Name += string.Format("\n{0} from target", Utils.formatBigValue((float)DistanceToTarget, "m"));
            //compute distance in lat-lon coordinates
            DeltaLat = Utils.AngleDelta(SurfacePoint.Pos.Lat, Target.Pos.Lat) *
            Math.Sign(Utils.AngleDelta(approach.Lat, SurfacePoint.Pos.Lat));
            DeltaLon = Utils.AngleDelta(SurfacePoint.Pos.Lon, Target.Pos.Lon) *
            Math.Sign(Utils.AngleDelta(approach.Lon, SurfacePoint.Pos.Lon));
            //compute distance in radial coordinates
            DeltaFi = 90 - Utils.Angle2(Orbit.GetOrbitNormal(),
                                        TrajectoryCalculator.BodyRotationAtdT(Body, TimeToTarget) * Target.OrbPos(Body));
            DeltaR = Utils.RadDelta(SurfacePoint.AngleTo(VslStartLat, VslStartLon), Target.AngleTo(VslStartLat, VslStartLon)) * Mathf.Rad2Deg;
//            Utils.Log("{}", this);//debug
        }

        public Vector3d GetOrbitVelocityAtSurface()
        {
            return Orbit.getOrbitalVelocityAtUT(AtTargetUT);
        }

        public override void UpdateOrbit(Orbit current)
        {
            base.UpdateOrbit(current);
            update(false, false);
        }

        public void UpdateOrbit(Orbit current, bool with_brake, bool brake_at_fly_above)
        {
            base.UpdateOrbit(current);
            update(with_brake, brake_at_fly_above);
        }

        public override string ToString()
        {
            return base.ToString() +
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

                            "Path:\n{}\n\n" +

                            "AfterBrakePath:\n{}\n",
                             SurfacePoint,
                             DeltaR, DeltaLat, DeltaLon, FlyAbovePoint.UT,
                             BrakeStartPoint.UT, BrakeEndPoint.UT, BrakeOffset, BrakeStartPoint.UT - VSL.Physics.UT, 
                             BrakeEndPointDeltaAlt, ActualLandingAngle, LandingSteepness,
                             WillOverheat, MaxShipTemperature, MaxDynamicPressure,
                             Path, AfterBrakePath);
        }
    }

    public class LandingPath
    {
        public struct Point
        {
            public VesselWrapper VSL;
            public Orbit Orbit;
            public LandingPath Path;

            public double UT;
            public double Duration;

            public Vector3d vel, srf_vel, pos, rel_pos;

            public double Altitude;
            public bool Atmosphere;
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

            public bool Ascending { get { return Vector3d.Dot(vel, pos) > 0; } }

            public void Update(double UT)
            {
                this.UT = UT;
                pos = Orbit.getRelativePositionAtUT(UT);
                vel = Orbit.getOrbitalVelocityAtUT(UT);
                Update();
            }

            public void Update()
            {
                Altitude = pos.magnitude - Body.Radius;
                Atmosphere = Body.atmosphere && Altitude < Body.atmosphereDepth;
                rel_pos = Body.BodyFrame.WorldToLocal(TrajectoryCalculator.BodyRotationAtdT(Body, Path.UT0 - UT) * pos);
                srf_vel = vel + Vector3d.Cross(Body.zUpAngularVelocity, pos);
                SrfSpeed = srf_vel.magnitude;

                if(Atmosphere)
                {
                    Pressure = Body.GetPressure(Altitude);
                    AtmosphereTemperature = Body.GetTemperature(Altitude);
                    Density = Body.GetDensity(Pressure, AtmosphereTemperature);
                    Mach1 = Body.GetSpeedOfSound(Pressure, Density);

                    var Rho_v = Density * SrfSpeed;
                    DynamicPressure = Rho_v * SrfSpeed;
                    Mach = SrfSpeed / Mach1;
                    this.SpecificDrag = AtmoSim.Cd * DynamicPressure *
                    PhysicsGlobals.DragCurveMultiplier.Evaluate((float)Mach) *
                    PhysicsGlobals.DragCurvePseudoReynolds.Evaluate((float)(Rho_v));

                    var convectiveMachLerp = Math.Pow(UtilMath.Clamp01((Mach - PhysicsGlobals.NewtonianMachTempLerpStartMach) /
                                             (PhysicsGlobals.NewtonianMachTempLerpEndMach - PhysicsGlobals.NewtonianMachTempLerpStartMach)), 
                                                      PhysicsGlobals.NewtonianMachTempLerpExponent);
                    ShockTemperature = SrfSpeed * PhysicsGlobals.NewtonianTemperatureFactor;
                    if(convectiveMachLerp > 0.0)
                    {
                        double b = PhysicsGlobals.MachTemperatureScalar * Math.Pow(SrfSpeed, PhysicsGlobals.MachTemperatureVelocityExponent);
                        ShockTemperature = UtilMath.LerpUnclamped(ShockTemperature, b, convectiveMachLerp);
                    }
                    ShockTemperature *= (double)HighLogic.CurrentGame.Parameters.Difficulty.ReentryHeatScale * Body.shockTemperatureMultiplier;
                    ShockTemperature = Math.Max(AtmosphereTemperature, ShockTemperature);
                    //calculate convective coefficient for speed > Mach1; lower speed is not a concern
                    ConvectiveCoefficient = 1E-10 * PhysicsGlobals.MachConvectionFactor;
                    ConvectiveCoefficient *= Density > 1 ? Density : Math.Pow(Density, PhysicsGlobals.MachConvectionDensityExponent);
                    ConvectiveCoefficient *= Math.Pow(SrfSpeed, PhysicsGlobals.MachConvectionVelocityExponent) * Body.convectionMultiplier;
                }
            }

            public double UpdateShipTemp(double startT)
            {
                if(Atmosphere)
                {
                    var K = VSL.Physics.MMT_ThermalMass > ConvectiveCoefficient ? 
                            -ConvectiveCoefficient / VSL.Physics.MMT_ThermalMass : -1;
                    ShipTemperature = ShockTemperature + (startT - ShockTemperature) * Math.Exp(K * Globals.Instance.LTRJ.HeatingCoefficient * Duration);
                }
                return ShipTemperature;
            }

            public Orbit OrbitFromHere()
            {
                return TrajectoryCalculator.NewOrbit(Body, pos, vel, UT);
            }

            public Vector3 CBRelativePosInWorldFrame()
            {
                return (Vector3)(Orbit.referenceBody.BodyFrame.LocalToWorld(rel_pos).xzy +
                                 Orbit.referenceBody.position);
            }

            public override string ToString()
            {
                return Utils.Format("Altitude {} m\n" +
                                    "Density {}, Pressure {} kPa, Atm.T {} K\n" +
                                    "SrfSpeed {} m/s, Dyn.Pressure {} kPa, Shock.T {} K\n" +
                                    "ConvectiveCoefficient {}, Ship.T {} K\n" +
                                    "UT {}, Duration {} s\n" +
                                    "pos {}\n" +
                                    "vel {}\n",
                                    Altitude, Density, Pressure, AtmosphereTemperature, 
                                    SrfSpeed, DynamicPressure / 1000, ShockTemperature, 
                                    ConvectiveCoefficient, ShipTemperature, UT, Duration,
                                    pos, vel);
            }
        }

        public readonly VesselWrapper VSL;
        public readonly Orbit Orbit;
        public readonly double TargetAltitude;

        public List<Point> Points = new List<Point>();

        public Point LastPoint { get; private set; }

        public bool HavePoints { get; private set; }

        public bool Atmosphere { get; private set; }

        public double UT0 = -1;
        public double StartUT = -1;
        public double FirstPointUT = -1;
        public double AtmoStartUT = -1;
        public double AtmoStopUT = -1;
        public double EndUT = -1;
        public double MaxShipTemperature = -1;
        public double MaxDynamicPressure = -1;

        Point newP(double UT)
        { 
            var p = new Point{ VSL = VSL, Orbit = Orbit, Path=this }; 
            p.Update(UT); 
            return p; 
        }

        public LandingPath(VesselWrapper vsl, Orbit orb, double target_altitude, double startUT, double dt, 
                           double start_mass, double fuel = 0, double brake_vel = 0)
        {
            VSL = vsl;
            Orbit = orb;
            TargetAltitude = target_altitude;
            StartUT = startUT;
            UT0 = VSL.Physics.UT;
            if(Orbit.referenceBody.atmosphere || brake_vel > 0 && fuel > 0)
            {
                EndUT = startUT + orb.timeToPe;
                HavePoints = brake_vel > 0 && fuel > 0;
                var p = newP(StartUT);
                var m = start_mass;
                var s = Math.Max(VSL.Geometry.MinArea, VSL.Geometry.AreaWithBrakes);
                var ascending = p.Ascending;
                p.Duration = dt;
                while((ascending || p.Altitude > TargetAltitude) && p.UT < EndUT)
                {
                    if(!ascending && dt > 0.01)
                    {
                        var dAlt = p.Altitude - TargetAltitude;
                        if(dAlt / p.SrfSpeed < dt)
                        {
                            dt = dAlt / p.SrfSpeed * 0.9;
                            p.Duration = dt;
                        }
                    }
                    var drag_dv = p.SpecificDrag * s / m * dt;
                    var prev_alt = p.Altitude;
                    Atmosphere |= drag_dv > 0;
                    HavePoints |= Atmosphere;
                    if(HavePoints)
                    {
                        if(Points.Count == 0)
                        {
                            FirstPointUT = p.UT;
                            if(Atmosphere)
                                AtmoStartUT = p.UT;
                        }
                        Points.Add(p);
                        //                    VSL.Log("drag dV {}, m {}, fm {}, brake_vel {}, p {}",
                        //                            drag_dv, m, fuel, brake_vel, p);//debug
                        var r = p.pos.magnitude;
                        if(brake_vel > 0 && fuel > 0)
                        {
                            //compute thrust direction
                            var vV = Utils.ClampL(Vector3d.Dot(p.vel, p.pos / r), 1e-5);
                            var b_dir = LandingTrajectoryAutopilot.CorrectedBrakeVelocity(VSL, p.vel, p.pos, 
                                                                                          p.DynamicPressure / 1000 / Globals.Instance.LTRJ.MinDPressure, 
                                                                                          (p.Altitude - TargetAltitude) / vV).normalized;
                            //compute thrust and mass flow
                            float mflow;
                            var thrust = VSL.Engines.ThrustAtAlt((float)p.SrfSpeed, (float)p.Altitude, out mflow);
                            var Ve = thrust / mflow;
                            var dm = mflow * dt;
                            if(dm > fuel)
                                dm = fuel;
                            var bV = (double)VSL.Engines.DeltaV(Ve, (float)dm);
                            if(bV > brake_vel)
                            {
                                bV = brake_vel;
                                dm = EnginesProps.FuelNeeded((float)bV, Ve, (float)m);
                            }
                            p.vel -= b_dir * bV;
                            brake_vel -= bV;
                            //                        VSL.Log("vV {}, vB {}, thrust {}, mflow {}, Ve {}, dm {}\nb_dir {}\nvel {}\npos {}",
                            //                                vV, bV, thrust, mflow, Ve, dm, b_dir, p.vel, p.pos);//debug
                            //spend fuel
                            fuel -= dm;
                            m -= dm;
                        }
                        if(Atmosphere)
                            p.vel -= p.srf_vel / p.SrfSpeed * Math.Min(drag_dv, p.SrfSpeed);
                        p.vel -= p.pos * p.Body.gMagnitudeAtCenter / r / r / r * dt;
                        p.pos += p.vel * dt;
                        p.UT += dt;
                        p.Update();
                        if(Atmosphere && AtmoStopUT < 0 && p.SrfSpeed < 10)
                            AtmoStopUT = p.UT;
                    }
                    else
                        p.Update(p.UT + dt);
                    ascending &= p.Altitude > prev_alt;
                }
                if(HavePoints)
                {
                    Points.Add(p);
                    if(Atmosphere && AtmoStopUT < 0)
                        AtmoStopUT = p.UT;
                }
                EndUT = p.UT;
                LastPoint = p;
            }
            else
            {
                EndUT = TrajectoryCalculator.NearestRadiusUT(Orbit, Orbit.referenceBody.Radius + TargetAltitude, StartUT);
                LastPoint = newP(EndUT);
            }
        }

        public void UpdateOverheatInto(double startT)
        {
            if(!HavePoints)
                return;
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
            if(!HavePoints || altitude > Points[0].Altitude)
                return TrajectoryCalculator.NearestRadiusUT(Orbit, altitude + Orbit.referenceBody.Radius, StartUT);
            for(int i = 1, count = Points.Count; i < count; i++)
            {
                var p1 = Points[i];
                if(p1.Altitude > altitude)
                    continue;
                var p0 = Points[i - 1];
                return p0.UT + p0.Duration * (altitude - p0.Altitude) / (p1.Altitude - p0.Altitude);
            }
            return EndUT;
        }

        public double UT2Altitude(double UT)
        {
            if(!HavePoints || UT < FirstPointUT)
                return Orbit.RadiusAtTrueAnomaly(Orbit.TrueAnomalyAtUT(UT)) - Orbit.referenceBody.Radius;
            for(int i = 1, count = Points.Count; i < count; i++)
            {
                var p1 = Points[i];
                if(p1.UT < UT)
                    continue;
                var p0 = Points[i - 1];
                var t = (UT - p0.UT) / (p0.Duration);
                return p0.Altitude + (p1.Altitude - p0.Altitude) * t;
            }
            return TargetAltitude;
        }

        public Point PointAtShipTemp(double T)
        {
            if(!Atmosphere)
                return newP(StartUT);
            for(int i = 1, count = Points.Count; i < count; i++)
            {
                var p1 = Points[i];
                if(p1.ShipTemperature < T)
                    continue;
                var p0 = Points[i - 1];
                var p = p0;
                var t = (T - p0.ShipTemperature) / (p1.ShipTemperature - p0.ShipTemperature);
                p.UT = p0.UT + p0.Duration * t;
                p.Duration = p1.UT - p.UT;
                p.pos = Vector3d.Lerp(p0.pos, p1.pos, t);
                p.ShipTemperature = T;
                p.Update();
                return p;
            }
            return LastPoint;
        }

        public Point PointAtUT(double UT)
        {
            if(!HavePoints || UT < FirstPointUT)
                return newP(UT);
            for(int i = 1, count = Points.Count; i < count; i++)
            {
                var p1 = Points[i];
                if(p1.UT < UT)
                    continue;
                var p0 = Points[i - 1];
                var p = p0;
                var t = (UT - p0.UT) / (p0.Duration);
                p.UT = UT;
                p.Duration = p1.UT - UT;
                p.pos = Vector3d.Lerp(p0.pos, p1.pos, t);
                p.Update();
                return p;
            }
            return LastPoint;
        }

        public Point PointAtAltitude(double altitude)
        {
            if(!HavePoints || altitude > Points[0].Altitude)
                return newP(TrajectoryCalculator.NearestRadiusUT(Orbit, altitude + Orbit.referenceBody.Radius, StartUT));
            for(int i = 1, count = Points.Count; i < count; i++)
            {
                var p1 = Points[i];
                if(p1.Altitude > altitude)
                    continue;
                var p0 = Points[i - 1];
                var p = p0;
                var t = (p0.Altitude - altitude) / (p0.Altitude - p1.Altitude);
                p.UT += p0.Duration * t;
                p.Duration = p1.UT - p.UT;
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
            if(!HavePoints ||
               Utils.ProjectionAngle(Orbit.getRelativePositionAtUT(AtmoStartUT), 
                                     TrajectoryCalculator.BodyRotationAtdT(Orbit.referenceBody, AtmoStartUT - UT0) * pos, 
                                     Orbit.getOrbitalVelocityAtUT(AtmoStartUT)) < 0)
                return newP(TrajectoryCalculator.FlyAboveUT(Orbit, pos, StartUT));
            var p0 = Points[0];
            var angle0 = Utils.ProjectionAngle(p0.pos, 
                                               TrajectoryCalculator
                                               .BodyRotationAtdT(Orbit.referenceBody, p0.UT - UT0) * pos, 
                                               p0.vel);
            for(int i = 1, count = Points.Count; i < count; i++)
            {
                var p1 = Points[i];
                var angle1 = Utils.ProjectionAngle(p1.pos, 
                                                   TrajectoryCalculator
                                                   .BodyRotationAtdT(Orbit.referenceBody, p1.UT - UT0) * pos, 
                                                   p1.vel);
                if(angle1 > 0)
                {
                    angle0 = angle1;
                    continue;
                }
                p0 = Points[i - 1];
                var p = p0;
                var t = angle0 / (angle0 - angle1);
                p.UT += p0.Duration * t;
                p.Duration = p1.UT - p.UT;
                p.pos = Vector3d.Lerp(p0.pos, p1.pos, t);
                p.Update();
                return p;
            }
            return LastPoint;
        }

        public Vector3[] CBRelativePathInWorldFrame()
        {
            return Points.Select(p => p.CBRelativePosInWorldFrame()).ToArray();
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
                                MaxShipTemperature, MaxDynamicPressure / 1000,
                                Points);
        }
    }
}

