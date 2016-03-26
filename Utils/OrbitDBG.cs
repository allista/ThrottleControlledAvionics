//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2016 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.
//
#if DEBUG
using System;
using UnityEngine;

namespace ThrottleControlledAvionics
{
	public class OrbitDBG
	{
		public enum ObjectType
		{
			VESSEL,
			SPACE_DEBRIS,
			CELESTIAL_BODIES,
			UNKNOWN_MISC,
			KERBAL
		}
		public enum EncounterSolutionLevel
		{
			NONE,
			ESCAPE,
			ORBIT_INTERSECT,
			SOI_INTERSECT_2,
			SOI_INTERSECT_1
		}
		public enum PatchTransitionType
		{
			INITIAL,
			FINAL,
			ENCOUNTER,
			ESCAPE,
			MANEUVER,
			IMPACT
		}
		public const double Rad2Deg = 57.295779513082323;
		public double inclination;
		public double eccentricity;
		public double semiMajorAxis;
		public double LAN;
		public double argumentOfPeriapsis;
		public double epoch;
		public CelestialBody referenceBody;
		public Vector3d pos;
		public Vector3d vel;
		public double orbitalEnergy;
		public double meanAnomaly;
		public double trueAnomaly;
		public double eccentricAnomaly;
		public double radius;
		public double altitude;
		public double orbitalSpeed;
		public double orbitPercent;
		public double ObT;
		public double ObTAtEpoch;
		public double timeToPe;
		public double timeToAp;
		[Obsolete("Use VesselType or CelestialBodyType instead")]
		public Orbit.ObjectType objectType = Orbit.ObjectType.UNKNOWN_MISC;
		public Vector3d h;
		public Vector3d eccVec;
		public Vector3d an;
		public double meanAnomalyAtEpoch;
		public double period;
		public Vector3 debugPos;
		public Vector3 debugVel;
		public Vector3 debugH;
		public Vector3 debugAN;
		public Vector3 debugEccVec;
		public double mag;
		double drawResolution = 15.0;
		static double Deg2Rad = 0.0174533;
		public double FEVp;
		public double FEVs;
		public double SEVp;
		public double SEVs;
		public double UTappr;
		public double UTsoi;
		public double ClAppr;
		public double CrAppr;
		public double ClEctr1;
		public double ClEctr2;
		public double timeToTransition1;
		public double timeToTransition2;
		public double nearestTT;
		public double nextTT;
		public Vector3d secondaryPosAtTransition1;
		public Vector3d secondaryPosAtTransition2;
		public double closestTgtApprUT;
		public double StartUT;
		public double EndUT;
		public bool activePatch;
		public Orbit closestEncounterPatch;
		public CelestialBody closestEncounterBody;
		public Orbit.EncounterSolutionLevel closestEncounterLevel;
		public Orbit.PatchTransitionType patchStartTransition;
		public Orbit.PatchTransitionType patchEndTransition;
		public Orbit nextPatch;
		public Orbit previousPatch;
		public double fromE;
		public double toE;
		public double sampleInterval;
		public double E;
		public double V;
		public double fromV;
		public double toV;
		public bool debug_returnFullEllipseTrajectory;
		public double semiMinorAxis
		{
			get
			{
				double arg_70_0;
				if (eccentricity < 1.0)
				{
					arg_70_0 = semiMajorAxis * Math.Sqrt(1.0 - eccentricity * eccentricity);
				}
				else
				{
					arg_70_0 = semiMajorAxis * Math.Sqrt(eccentricity * eccentricity - 1.0);
				}
				return arg_70_0;
			}
		}
		public double semiLatusRectum
		{
			get
			{
				return h.sqrMagnitude / referenceBody.gravParameter;
			}
		}
		public double PeR
		{
			get
			{
				return (1.0 - eccentricity) * semiMajorAxis;
			}
		}
		public double ApR
		{
			get
			{
				return (1.0 + eccentricity) * semiMajorAxis;
			}
		}
		public double PeA
		{
			get
			{
				return PeR - referenceBody.Radius;
			}
		}
		public double ApA
		{
			get
			{
				return ApR - referenceBody.Radius;
			}
		}
		public OrbitDBG()
		{
		}
		public OrbitDBG(double inc, double e, double sma, double lan, double w, double mEp, double t, CelestialBody body)
		{
			inclination = inc;
			eccentricity = e;
			semiMajorAxis = sma;
			LAN = lan;
			argumentOfPeriapsis = w;
			meanAnomalyAtEpoch = mEp;
			epoch = t;
			referenceBody = body;
			Init();
		}
		public void Init()
		{
			period = 6.2831853071795862 * Math.Sqrt(Math.Pow(Math.Abs(semiMajorAxis), 3.0) / referenceBody.gravParameter);
			if (eccentricity < 1.0)
			{
				meanAnomaly = meanAnomalyAtEpoch;
				orbitPercent = meanAnomaly / 6.2831853071795862;
				ObTAtEpoch = orbitPercent * period;
			}
			else
			{
				meanAnomaly = meanAnomalyAtEpoch;
				ObT = Math.Pow(Math.Pow(Math.Abs(semiMajorAxis), 3.0) / referenceBody.gravParameter, 0.5) * meanAnomaly;
				ObTAtEpoch = ObT;
			}
		}
		public void UpdateFromOrbitAtUT(Orbit orbit, double UT, CelestialBody toBody)
		{
			pos = (orbit.getTruePositionAtUT(UT) - toBody.getTruePositionAtUT(UT)).xzy;
			vel = orbit.getOrbitalVelocityAtUT(UT) + orbit.referenceBody.GetFrameVelAtUT(UT) - toBody.GetFrameVelAtUT(UT);
			UpdateFromStateVectors(pos, vel, toBody, UT);
		}
		public void UpdateFromStateVectors(Vector3d pos, Vector3d vel, CelestialBody refBody, double UT)
		{
			referenceBody = refBody;
			h = Vector3d.Cross(pos, vel);
			inclination = Math.Acos(h.z / h.magnitude) * 57.295779513082323;
			if (inclination == 0.0)
			{
				vel += Vector3d.forward * 1E-10;
				h = Vector3d.Cross(pos, vel);
				inclination = Math.Acos(h.z / h.magnitude) * 57.295779513082323;
			}
			eccVec = Vector3d.Cross(vel, h) / refBody.gravParameter - pos / pos.magnitude;
			eccentricity = eccVec.magnitude;
			orbitalEnergy = vel.sqrMagnitude / 2.0 - refBody.gravParameter / pos.magnitude;
			double arg_199_1;
			if (eccentricity < 1.0)
			{
				arg_199_1 = -refBody.gravParameter / (2.0 * orbitalEnergy);
			}
			else
			{
				arg_199_1 = -semiLatusRectum / (eccVec.sqrMagnitude - 1.0);
			}
			semiMajorAxis = arg_199_1;
			an = Vector3d.Cross(Vector3d.forward, h);
			double arg_22E_0;
			if (an.y >= 0.0)
			{
				arg_22E_0 = Math.Acos(an.x / an.magnitude);
			}
			else
			{
				arg_22E_0 = 6.2831853071795862 - Math.Acos(an.x / an.magnitude);
			}
			LAN = arg_22E_0 * 57.295779513082323;
			argumentOfPeriapsis = Math.Acos(Vector3d.Dot(an, eccVec) / (an.magnitude * eccentricity));
			if (eccVec.z < 0.0)
			{
				argumentOfPeriapsis = 6.2831853071795862 - argumentOfPeriapsis;
			}
			if (an == Vector3d.zero)
			{
				LAN = 0.0;
				argumentOfPeriapsis = Math.Acos(eccVec.x / eccentricity);
			}
			LAN = (LAN + Planetarium.InverseRotAngle) % 360.0;
			argumentOfPeriapsis *= 57.295779513082323;
			period = 6.2831853071795862 * Math.Sqrt(Math.Pow(Math.Abs(semiMajorAxis), 3.0) / refBody.gravParameter);
			trueAnomaly = Math.Acos(Vector3d.Dot(eccVec, pos) / (eccentricity * pos.magnitude));
			if (Vector3d.Dot(pos, vel) < 0.0)
			{
				trueAnomaly = 6.2831853071795862 - trueAnomaly;
			}
			if (double.IsNaN(trueAnomaly))
			{
				trueAnomaly = 3.1415926535897931;
			}
			eccentricAnomaly = GetEccentricAnomaly(trueAnomaly);
			meanAnomaly = GetMeanAnomaly(eccentricAnomaly, trueAnomaly);
			meanAnomalyAtEpoch = meanAnomaly;
			if (eccentricity < 1.0)
			{
				orbitPercent = meanAnomaly / 6.2831853071795862;
				ObT = orbitPercent * period;
				timeToPe = period - ObT;
				timeToAp = timeToPe - period / 2.0;
				if (timeToAp < 0.0)
				{
					timeToAp += period;
				}
				ObTAtEpoch = ObT;
			}
			else
			{
				ObT = Math.Pow(Math.Pow(-semiMajorAxis, 3.0) / refBody.gravParameter, 0.5) * meanAnomaly;
				timeToPe = -ObT;
				ObTAtEpoch = ObT;
			}
			trueAnomaly *= 57.295779513082323;
			radius = pos.magnitude;
			altitude = radius - refBody.Radius;
			epoch = UT;
			this.pos = pos;
			this.vel = vel;
			debugPos = pos;
			debugVel = vel;
			debugH = h;
			debugAN = an;
			debugEccVec = eccVec;
		}
		public void UpdateFromUT(double UT)
		{
			ObT = getObtAtUT(UT);
			an = QuaternionD.AngleAxis(LAN, Planetarium.Zup.Z) * Planetarium.Zup.X;
			if (!Planetarium.Pause)
			{
				mag = Vector3d.Cross(pos, vel).magnitude;
				Vector3d expr_A9 = QuaternionD.AngleAxis(inclination, an) * Planetarium.Zup.Z;
				double arg_E5_1;
				if (double.IsNaN(mag))
				{
					arg_E5_1 = 1.0;
				}
				else
				{
					arg_E5_1 = Math.Max(mag, 1.0);
				}
				h = expr_A9 * arg_E5_1;
			}
			eccVec = QuaternionD.AngleAxis(argumentOfPeriapsis, h) * an * eccentricity;
			if (eccentricity < 1.0)
			{
				meanAnomaly = ObT / period * 2.0 * 3.1415926535897931;
				double arg_1B9_1;
				if (eccentricity < 0.9)
				{
					arg_1B9_1 = solveEccentricAnomalyStd(meanAnomaly, eccentricity, 1E-07);
				}
				else
				{
					arg_1B9_1 = solveEccentricAnomalyExtremeEcc(meanAnomaly, eccentricity, 8);
				}
				eccentricAnomaly = arg_1B9_1;
				trueAnomaly = Math.Acos((Math.Cos(eccentricAnomaly) - eccentricity) / (1.0 - eccentricity * Math.Cos(eccentricAnomaly)));
				if (ObT > period / 2.0)
				{
					trueAnomaly = 6.2831853071795862 - trueAnomaly;
				}
				radius = semiMajorAxis * (1.0 - eccentricity * eccentricity) / (1.0 + eccentricity * Math.Cos(trueAnomaly));
			}
			else
			{
				if (eccentricity == 1.0)
				{
					eccentricity += 1E-10;
				}
				meanAnomaly = 6.2831853071795862 * Math.Abs(ObT) / period;
				if (ObT < 0.0)
				{
					meanAnomaly *= -1.0;
				}
				eccentricAnomaly = solveEccentricAnomalyHyp(Math.Abs(meanAnomaly), eccentricity, 1E-07);
				trueAnomaly = Math.Atan2(Math.Sqrt(eccentricity * eccentricity - 1.0) * Math.Sinh(eccentricAnomaly), eccentricity - Math.Cosh(eccentricAnomaly));
				if (ObT < 0.0)
				{
					trueAnomaly = 6.2831853071795862 - trueAnomaly;
				}
				radius = -semiMajorAxis * (eccentricity * eccentricity - 1.0) / (1.0 + eccentricity * Math.Cos(trueAnomaly));
			}
			orbitPercent = meanAnomaly / 6.2831853071795862;
			trueAnomaly *= 57.295779513082323;
			pos = QuaternionD.AngleAxis(argumentOfPeriapsis + trueAnomaly, h) * an * radius;
			if (eccentricity > 1E-05)
			{
				if (eccentricity < 1.0)
				{
					Vector3d relativePositionAtT = getRelativePositionAtT(ObT + (double)TimeWarp.deltaTime);
					double magnitude = relativePositionAtT.magnitude;
					orbitalSpeed = Math.Sqrt(referenceBody.gravParameter * (2.0 / magnitude - 1.0 / semiMajorAxis));
					double num = magnitude / semiMajorAxis;
					double num2 = Math.Acos((2.0 - 2.0 * (eccentricity * eccentricity)) / (num * (2.0 - num)) - 1.0);
					double num3 = (3.1415926535897931 - num2) / 2.0;
					if (ObT > period / 2.0)
					{
						num3 = num2 + num3;
					}
					vel = QuaternionD.AngleAxis(num3 * 57.295779513082323, h) * relativePositionAtT.normalized * orbitalSpeed;
					goto IL_63A;
				}
			}
			Vector3d relativePositionAtT2 = getRelativePositionAtT(ObT + (double)TimeWarp.deltaTime);
			orbitalSpeed = Math.Sqrt(referenceBody.gravParameter * (2.0 / pos.magnitude - 1.0 / semiMajorAxis));
			vel = (relativePositionAtT2 - pos).normalized * orbitalSpeed;
			IL_63A:
			orbitalEnergy = orbitalSpeed * orbitalSpeed / 2.0 - referenceBody.gravParameter / pos.magnitude;
			altitude = radius - referenceBody.Radius;
			if (eccentricity < 1.0)
			{
				timeToPe = period - ObT;
				timeToAp = timeToPe - period / 2.0;
				if (timeToAp < 0.0)
				{
					timeToAp += period;
				}
			}
			else
			{
				timeToPe = -ObT;
				timeToAp = 0.0;
			}
			debugPos = pos;
			debugVel = vel;
			debugH = h;
			debugAN = an;
			debugEccVec = eccVec;
		}
		public double GetDTforTrueAnomaly(double tA, double wrapAfterSeconds)
		{
			double num = GetEccentricAnomaly(tA);
			double num2 = GetMeanAnomaly(num, tA);
			double arg_82_0;
			if (eccentricity < 1.0)
			{
				arg_82_0 = num2 / 6.2831853071795862 * period;
			}
			else
			{
				arg_82_0 = Math.Pow(Math.Pow(-semiMajorAxis, 3.0) / referenceBody.gravParameter, 0.5) * num2;
			}
			double num3 = arg_82_0;
			double num4;
			if (eccentricity < 1.0)
			{
				if (tA < 0.0)
				{
					num4 = -ObT - num3;
				}
				else
				{
					num4 = num3 - ObT;
				}
				if (num4 < -Math.Abs(wrapAfterSeconds))
				{
					num4 += period;
				}
			}
			else if (tA < 0.0)
			{
				num4 = -ObT - num3;
			}
			else
			{
				num4 = num3 - ObT;
			}
			if (double.IsNaN(num4))
			{
				Debug.Log(string.Concat(new object[]
				{
					"dT is NaN! tA:",
					tA,
					", E:",
					num,
					", M:",
					num2,
					", T:",
					num3
				}));
			}
			return num4;
		}
		public double GetUTforTrueAnomaly(double tA, double wrapAfterSeconds)
		{
			return Planetarium.GetUniversalTime() + GetDTforTrueAnomaly(tA, wrapAfterSeconds);
		}
		public Vector3d getPositionAtUT(double UT)
		{
			return getPositionAtT(getObtAtUT(UT));
		}
		public Vector3d getTruePositionAtUT(double UT)
		{
			return getRelativePositionAtUT(UT).xzy + referenceBody.getTruePositionAtUT(UT);
		}
		public Vector3d getRelativePositionAtUT(double UT)
		{
			return getRelativePositionAtT(getObtAtUT(UT));
		}
		public Vector3d getOrbitalVelocityAtUT(double UT)
		{
			return getOrbitalVelocityAtObT(getObtAtUT(UT));
		}
		public double getObtAtUT(double UT)
		{
			double num;
			if (eccentricity < 1.0)
			{
				num = (UT - epoch + ObTAtEpoch) % period;
			}
			else
			{
				num = ObTAtEpoch + (UT - epoch);
			}
			if (num < 0.0)
			{
				if (eccentricity < 1.0)
				{
					num += period;
				}
			}
			if (double.IsNaN(num))
			{
				Debug.Log("getObtAtUT result is NaN! UT:" + UT.ToString());
			}
			return num;
		}
		public double getObTAtMeanAnomaly(double M)
		{
			if (eccentricity < 1.0)
			{
				return meanAnomaly / 6.2831853071795862 * period;
			}
			return Math.Pow(Math.Pow(-semiMajorAxis, 3.0) / referenceBody.gravParameter, 0.5) * M;
		}
		public Vector3d GetOrbitNormal()
		{
			if (Planetarium.FrameIsRotating())
			{
				Vector3d axis = QuaternionD.AngleAxis(LAN, Planetarium.Zup.Z) * Planetarium.Zup.X;
				return QuaternionD.AngleAxis(inclination, axis) * Planetarium.Zup.Z;
			}
			return h;
		}
		public Vector3d GetEccVector()
		{
			if (Planetarium.FrameIsRotating())
			{
				Vector3d vector3d = QuaternionD.AngleAxis(LAN, Planetarium.Zup.Z) * Planetarium.Zup.X;
				Vector3d axis = QuaternionD.AngleAxis(inclination, vector3d) * Planetarium.Zup.Z;
				return QuaternionD.AngleAxis(argumentOfPeriapsis, axis) * vector3d;
			}
			return eccVec;
		}
		public Vector3d GetANVector()
		{
			if (Planetarium.FrameIsRotating())
			{
				return QuaternionD.AngleAxis(LAN, Planetarium.Zup.Z) * Planetarium.Zup.X;
			}
			return an;
		}
		public Vector3d GetVel()
		{
			Vector3d expr_06 = GetFrameVel();
			Vector3d arg_4E_1;
			if (FlightGlobals.ActiveVessel)
			{
				arg_4E_1 = FlightGlobals.ActiveVessel.orbitDriver.referenceBody.GetFrameVel();
			}
			else
			{
				arg_4E_1 = Vector3d.zero;
			}
			Vector3d vector3d = expr_06 - arg_4E_1;
			return new Vector3d(vector3d.x, vector3d.z, vector3d.y);
		}
		public Vector3d GetRelativeVel()
		{
			return vel.xzy;
		}
		public Vector3d GetRotFrameVel(CelestialBody refBody)
		{
			if (refBody.rotates)
			{
				if (refBody.inverseRotation)
				{
					return Vector3d.Cross(refBody.zUpAngularVelocity, -pos);
				}
			}
			return Vector3d.zero;
		}
		public Vector3d GetFrameVel()
		{
			return vel + referenceBody.GetFrameVel();
		}
		public Vector3d GetFrameVelAtUT(double UT)
		{
			return getOrbitalVelocityAtUT(UT) + referenceBody.GetFrameVelAtUT(UT);
		}
		public Vector3d GetWorldSpaceVel()
		{
			Vector3d expr_06 = GetVel();
			Vector3d arg_58_1;
			if (referenceBody.inverseRotation)
			{
				arg_58_1 = referenceBody.getRFrmVel(pos + referenceBody.position);
			}
			else
			{
				arg_58_1 = Vector3d.zero;
			}
			return expr_06 - arg_58_1;
		}
		public double GetEccentricAnomaly(double tA)
		{
			double num;
			if (eccentricity < 1.0)
			{
				num = Math.Acos((eccentricity + Math.Cos(tA)) / (1.0 + eccentricity * Math.Cos(tA)));
				if (tA > 3.1415926535897931)
				{
					num = 6.2831853071795862 - num;
				}
			}
			else
			{
				num = UtilMath.ACosh((eccentricity + Math.Cos(tA)) / (1.0 + eccentricity * Math.Cos(tA)));
				if (double.IsNaN(num))
				{
					num = 3.1415926535897931;
				}
				if (double.IsInfinity(num))
				{
					Debug.Log(string.Concat(new object[]
					{
						"E is Infinity! tA:",
						tA,
						", e =",
						eccentricity
					}));
				}
			}
			return num;
		}
		public double GetMeanAnomaly(double E, double tA)
		{
			if (eccentricity < 1.0)
			{
				return E - eccentricity * Math.Sin(E);
			}
			double num = eccentricity * Math.Sinh(E) - E;
			double arg_73_0 = num;
			double arg_73_1;
			if (tA < 3.1415926535897931)
			{
				arg_73_1 = 1.0;
			}
			else
			{
				arg_73_1 = -1.0;
			}
			return arg_73_0 * arg_73_1;
		}
		public double RadiusAtTrueAnomaly(double tA)
		{
			tA = tA * 3.1415926535897931 / 180.0;
			double arg_A4_0;
			if (eccentricity < 1.0)
			{
				arg_A4_0 = semiLatusRectum * (1.0 / (1.0 + eccentricity * Math.Cos(tA)));
			}
			else
			{
				arg_A4_0 = -semiMajorAxis * (eccentricity * eccentricity - 1.0) / (1.0 + eccentricity * Math.Cos(tA));
			}
			return arg_A4_0;
		}
		public double TrueAnomalyAtRadius(double R)
		{
			return trueAnomalyAtRadiusExtreme(R);
		}
		double trueAnomalyAtRadiusExtreme(double R)
		{
			double num = Vector3d.Cross(getRelativePositionFromEccAnomaly(eccentricAnomaly), getOrbitalVelocityAtObT(ObT)).sqrMagnitude / referenceBody.gravParameter;
			double result;
			if (eccentricity < 1.0)
			{
				R = Math.Min(Math.Max(PeR, R), ApR);
				result = Math.Acos(num / (eccentricity * R) - 1.0 / eccentricity);
			}
			else
			{
				R = Math.Max(PeR, R);
				result = 3.1415926535897931 - Math.Acos(semiMajorAxis * eccentricity / R - semiMajorAxis / (eccentricity * R) + 1.0 / eccentricity);
			}
			return result;
		}
		public double TrueAnomalyAtUT(double UT)
		{
			return TrueAnomalyAtT(getObtAtUT(UT));
		}
		public double TrueAnomalyAtT(double T)
		{
			double num3;
			if (eccentricity < 1.0)
			{
				double num = T / period * 2.0 * 3.1415926535897931;
				double arg_89_0;
				if (eccentricity < 0.9)
				{
					arg_89_0 = solveEccentricAnomalyStd(num, eccentricity, 1E-07);
				}
				else
				{
					arg_89_0 = solveEccentricAnomalyExtremeEcc(num, eccentricity, 8);
				}
				double num2 = arg_89_0;
				num3 = Math.Acos((Math.Cos(num2) - eccentricity) / (1.0 - eccentricity * Math.Cos(num2)));
				if (T > period / 2.0)
				{
					num3 = 6.2831853071795862 - num3;
				}
			}
			else
			{
				double num = 6.2831853071795862 * Math.Abs(T) / period;
				if (T < 0.0)
				{
					num *= -1.0;
				}
				double num2 = solveEccentricAnomalyHyp(Math.Abs(num), eccentricity, 1E-07);
				num3 = Math.Atan2(Math.Sqrt(eccentricity * eccentricity - 1.0) * Math.Sinh(num2), eccentricity - Math.Cosh(num2));
				if (T < 0.0)
				{
					num3 = 6.2831853071795862 - num3;
				}
			}
			return num3;
		}
		public double solveEccentricAnomaly(double M, double ecc, double maxError, int maxIterations)
		{
			double arg_74_0;
			if (eccentricity < 1.0)
			{
				if (eccentricity < 0.8)
				{
					arg_74_0 = solveEccentricAnomalyStd(M, eccentricity, maxError);
				}
				else
				{
					arg_74_0 = solveEccentricAnomalyExtremeEcc(M, eccentricity, maxIterations);
				}
			}
			else
			{
				arg_74_0 = solveEccentricAnomalyHyp(M, eccentricity, maxError);
			}
			return arg_74_0;
		}
		double solveEccentricAnomalyStd(double M, double ecc, double maxError = 1E-07)
		{
			double num = 1.0;
			double num2 = M + ecc * Math.Sin(M) + 0.5 * ecc * ecc * Math.Sin(2.0 * M);
			while (Math.Abs(num) > maxError)
			{
				double num3 = num2 - ecc * Math.Sin(num2);
				num = (M - num3) / (1.0 - ecc * Math.Cos(num2));
				num2 += num;
			}
			return num2;
		}
		double solveEccentricAnomalyExtremeEcc(double M, double ecc, int iterations = 8)
		{
			double num = M + 0.85 * eccentricity * (double)Math.Sign(Math.Sin(M));
			for (int i = 0; i < iterations; i++)
			{
				double num2 = ecc * Math.Sin(num);
				double num3 = ecc * Math.Cos(num);
				double num4 = num - num2 - M;
				double num5 = 1.0 - num3;
				double num6 = num2;
				num += -5.0 * num4 / (num5 + (double)Math.Sign(num5) * Math.Sqrt(Math.Abs(16.0 * num5 * num5 - 20.0 * num4 * num6)));
			}
			return num;
		}
		double solveEccentricAnomalyHyp(double M, double ecc, double maxError = 1E-07)
		{
			double num = 1.0;
			double num2 = Math.Log(2.0 * M / ecc + 1.8);
			while (Math.Abs(num) > maxError)
			{
				num = (eccentricity * Math.Sinh(num2) - num2 - M) / (eccentricity * Math.Cosh(num2) - 1.0);
				num2 -= num;
			}
			return num2;
		}
		public double getTrueAnomaly(double E)
		{
			double num;
			if (eccentricity < 1.0)
			{
				num = Math.Acos((Math.Cos(E) - eccentricity) / (1.0 - eccentricity * Math.Cos(E)));
				if (E > 3.1415926535897931)
				{
					num = 6.2831853071795862 - num;
				}
				if (E < 0.0)
				{
					num *= -1.0;
				}
			}
			else
			{
				num = Math.Atan2(Math.Sqrt(eccentricity * eccentricity - 1.0) * Math.Sinh(E), eccentricity - Math.Cosh(E));
			}
			return num;
		}
		public double GetTrueAnomalyOfZupVector(Vector3d vector)
		{
			Vector3d vector3d;
			if (eccVec != Vector3d.zero)
			{
				vector3d = Quaternion.Inverse(Quaternion.LookRotation(-GetEccVector().xzy, GetOrbitNormal().xzy)) * vector.xzy;
			}
			else
			{
				vector3d = Quaternion.Inverse(Quaternion.LookRotation(-getPositionFromTrueAnomaly(0.0).normalized, GetOrbitNormal().xzy)) * vector.xzy;
			}
			double num = 3.1415926535897931 - Math.Atan2(vector3d.x, vector3d.z);
			if (num < 0.0)
			{
				num = 6.2831853071795862 - num;
			}
			return num;
		}
		public Vector3d getPositionAtT(double T)
		{
			return referenceBody.position + getRelativePositionAtT(T).xzy;
		}
		public Vector3d getRelativePositionAtT(double T)
		{
			double num3;
			double d;
			if (eccentricity < 1.0)
			{
				double num = T / period * 2.0 * 3.1415926535897931;
				double arg_89_0;
				if (eccentricity < 0.9)
				{
					arg_89_0 = solveEccentricAnomalyStd(num, eccentricity, 1E-07);
				}
				else
				{
					arg_89_0 = solveEccentricAnomalyExtremeEcc(num, eccentricity, 8);
				}
				double num2 = arg_89_0;
				num3 = Math.Acos((Math.Cos(num2) - eccentricity) / (1.0 - eccentricity * Math.Cos(num2)));
				if (T > period / 2.0)
				{
					num3 = 6.2831853071795862 - num3;
				}
				d = semiMajorAxis * (1.0 - eccentricity * eccentricity) / (1.0 + eccentricity * Math.Cos(num3));
			}
			else
			{
				double num = 6.2831853071795862 * Math.Abs(T) / period;
				if (T < 0.0)
				{
					num *= -1.0;
				}
				double num2 = solveEccentricAnomalyHyp(Math.Abs(num), eccentricity, 1E-07);
				num3 = Math.Atan2(Math.Sqrt(eccentricity * eccentricity - 1.0) * Math.Sinh(num2), eccentricity - Math.Cosh(num2));
				if (T < 0.0)
				{
					num3 = 6.2831853071795862 - num3;
				}
				d = -semiMajorAxis * (eccentricity * eccentricity - 1.0) / (1.0 + eccentricity * Math.Cos(num3));
			}
			num3 *= 57.295779513082323;
			Vector3d vector3d = QuaternionD.AngleAxis(LAN, Planetarium.Zup.Z) * Planetarium.Zup.X;
			Vector3d axis = QuaternionD.AngleAxis(inclination, vector3d) * Planetarium.Zup.Z;
			return QuaternionD.AngleAxis(argumentOfPeriapsis + num3, axis) * vector3d * d;
		}
		public Vector3d getPositionFromMeanAnomaly(double M)
		{
			return referenceBody.position + getRelativePositionFromMeanAnomaly(M).xzy;
		}
		public Vector3d getRelativePositionFromMeanAnomaly(double M)
		{
			double e = solveEccentricAnomaly(M, eccentricity, 1E-05, 8);
			return getRelativePositionFromEccAnomaly(e);
		}
		public Vector3d getPositionFromEccAnomaly(double E)
		{
			return referenceBody.position + getRelativePositionFromEccAnomaly(E).xzy;
		}
		public Vector3d getRelativePositionFromEccAnomaly(double E)
		{
			E *= -1.0;
			double x;
			double y;
			if (eccentricity < 1.0)
			{
				x = semiMajorAxis * (Math.Cos(E) - eccentricity);
				y = semiMajorAxis * Math.Sqrt(1.0 - eccentricity * eccentricity) * -Math.Sin(E);
			}
			else if (eccentricity > 1.0)
			{
				x = -semiMajorAxis * (eccentricity - Math.Cosh(E));
				y = -semiMajorAxis * Math.Sqrt(eccentricity * eccentricity - 1.0) * -Math.Sinh(E);
			}
			else
			{
				x = 0.0;
				y = 0.0;
			}
			Vector3d vector3d = new Vector3d(x, y, 0.0);
			Vector3d axis = QuaternionD.AngleAxis(LAN, Planetarium.Zup.Z) * Planetarium.Zup.X;
			Vector3d axis2 = QuaternionD.AngleAxis(inclination, axis) * Planetarium.Zup.Z;
			QuaternionD rotation = QuaternionD.AngleAxis(argumentOfPeriapsis, axis2) * QuaternionD.AngleAxis(inclination, axis) * QuaternionD.AngleAxis(LAN - Planetarium.InverseRotAngle, Planetarium.Zup.Z);
			vector3d = rotation * vector3d;
			return vector3d;
		}
		public Vector3d getPositionFromTrueAnomaly(double tA)
		{
			return referenceBody.position + getRelativePositionFromTrueAnomaly(tA).xzy;
		}
		public Vector3d getRelativePositionFromTrueAnomaly(double tA)
		{
			double arg_8B_0;
			if (eccentricity < 1.0)
			{
				arg_8B_0 = semiLatusRectum * (1.0 / (1.0 + eccentricity * Math.Cos(tA)));
			}
			else
			{
				arg_8B_0 = -semiMajorAxis * (eccentricity * eccentricity - 1.0) / (1.0 + eccentricity * Math.Cos(tA));
			}
			double d = arg_8B_0;
			Vector3d vector3d = QuaternionD.AngleAxis(LAN, Planetarium.Zup.Z) * Planetarium.Zup.X;
			Vector3d axis = QuaternionD.AngleAxis(inclination, vector3d) * Planetarium.Zup.Z;
			return QuaternionD.AngleAxis(argumentOfPeriapsis + tA * 57.295779513082323, axis) * vector3d * d;
		}
		public double getOrbitalSpeedAt(double time)
		{
			return getOrbitalSpeedAtDistance(getRelativePositionAtT(time).magnitude);
		}
		public double getOrbitalSpeedAtRelativePos(Vector3d relPos)
		{
			return getOrbitalSpeedAtDistance(relPos.magnitude);
		}
		public double getOrbitalSpeedAtPos(Vector3d pos)
		{
			return getOrbitalSpeedAtDistance((referenceBody.position - pos).magnitude);
		}
		public double getOrbitalSpeedAtDistance(double d)
		{
			return Math.Sqrt(referenceBody.gravParameter * (2.0 / d - 1.0 / semiMajorAxis));
		}
		public Vector3d getOrbitalVelocityAtObT(double ObT)
		{
			if (eccentricity > 1E-05)
			{
				if (eccentricity < 1.0)
				{
					Vector3d relativePositionAtT = getRelativePositionAtT(ObT + (double)Time.fixedDeltaTime);
					double magnitude = relativePositionAtT.magnitude;
					double d = Math.Sqrt(referenceBody.gravParameter * (2.0 / magnitude - 1.0 / semiMajorAxis));
					double num = magnitude / semiMajorAxis;
					double num2 = Math.Acos((2.0 - 2.0 * (eccentricity * eccentricity)) / (num * (2.0 - num)) - 1.0);
					double num3 = (3.1415926535897931 - num2) / 2.0;
					if (ObT > period / 2.0)
					{
						num3 = num2 + num3;
					}
					Vector3d axis = QuaternionD.AngleAxis(LAN, Planetarium.Zup.Z) * Planetarium.Zup.X;
					Vector3d axis2 = QuaternionD.AngleAxis(inclination, axis) * Planetarium.Zup.Z;
					return QuaternionD.AngleAxis(num3 * 57.295779513082323, axis2) * relativePositionAtT.normalized * d;
				}
			}
			Vector3d relativePositionAtT2 = getRelativePositionAtT(ObT);
			Vector3d relativePositionAtT3 = getRelativePositionAtT(ObT - (double)Time.fixedDeltaTime);
			double d2 = Math.Sqrt(referenceBody.gravParameter * (2.0 / relativePositionAtT2.magnitude - 1.0 / semiMajorAxis));
			Vector3d result = (relativePositionAtT2 - relativePositionAtT3).normalized * d2;
			if (double.IsNaN(result.x))
			{
				Debug.Log(string.Concat(new string[]
				{
					"problem!",
					relativePositionAtT2.ToString(),
					"-",
					relativePositionAtT3.ToString(),
					"-",
					d2.ToString(),
					"-",
					result.ToString(),
					"-",
					ObT.ToString()
				}));
			}
			return result;
		}
		public void DrawOrbit()
		{
			if (eccentricity < 1.0)
			{
				for (double num = 0.0; num < 6.2831853071795862; num += drawResolution * 3.1415926535897931 / 180.0)
				{
					Vector3 v = getPositionFromTrueAnomaly(num % 6.2831853071795862);
					Vector3 v2 = getPositionFromTrueAnomaly((num + drawResolution * 3.1415926535897931 / 180.0) % 6.2831853071795862);
					Debug.DrawLine(ScaledSpace.LocalToScaledSpace(v), ScaledSpace.LocalToScaledSpace(v2), Color.Lerp(Color.yellow, Color.green, Mathf.InverseLerp((float)getOrbitalSpeedAtDistance(PeR), (float)getOrbitalSpeedAtDistance(ApR), (float)getOrbitalSpeedAtPos(v))));
				}
			}
			else
			{
				for (double num2 = -Math.Acos(-(1.0 / eccentricity)) + drawResolution * 0.01745329238474369; num2 < Math.Acos(-(1.0 / eccentricity)) - drawResolution * 0.01745329238474369; num2 += drawResolution * 0.01745329238474369)
				{
					Debug.DrawLine(ScaledSpace.LocalToScaledSpace(getPositionFromTrueAnomaly(num2)), ScaledSpace.LocalToScaledSpace(getPositionFromTrueAnomaly(Math.Min(Math.Acos(-(1.0 / eccentricity)), num2 + drawResolution * 0.01745329238474369))), Color.green);
				}
			}
			Debug.DrawLine(ScaledSpace.LocalToScaledSpace(getPositionAtT(ObT)), ScaledSpace.LocalToScaledSpace(referenceBody.position), Color.green);
			Debug.DrawRay(ScaledSpace.LocalToScaledSpace(getPositionAtT(ObT)), new Vector3d(vel.x, vel.z, vel.y) * 0.0099999997764825821, Color.white);
			Debug.DrawLine(ScaledSpace.LocalToScaledSpace(referenceBody.position), ScaledSpace.LocalToScaledSpace(referenceBody.position + an.xzy * radius), Color.cyan);
			Debug.DrawLine(ScaledSpace.LocalToScaledSpace(referenceBody.position), ScaledSpace.LocalToScaledSpace(getPositionAtT(0.0)), Color.magenta);
			Debug.DrawRay(ScaledSpace.LocalToScaledSpace(referenceBody.position), ScaledSpace.LocalToScaledSpace(h.xzy), Color.blue);
		}
		public static bool PeApIntersects(Orbit primary, Orbit secondary, double threshold)
		{
			if (primary.eccentricity >= 1.0)
			{
				return primary.PeR < secondary.ApR + threshold;
			}
			if (secondary.eccentricity >= 1.0)
			{
				return secondary.PeR < primary.ApR + threshold;
			}
			return Math.Max(primary.PeR, secondary.PeR) - Math.Min(primary.ApR, secondary.ApR) <= threshold;
		}
		public static void FindClosestPoints(Orbit p, Orbit s, ref double CD, ref double CCD, ref double FFp, ref double FFs, ref double SFp, ref double SFs, double epsilon, int maxIterations, ref int iterationCount)
		{
			double num = p.inclination * OrbitDBG.Deg2Rad;
			double num2 = s.inclination * OrbitDBG.Deg2Rad;
			double num3 = num - num2;
			Vector3d vector3d = Vector3d.Cross(s.h, p.h);
			Debug.DrawRay(ScaledSpace.LocalToScaledSpace(p.referenceBody.position), vector3d.xzy * 1000.0, Color.white);
			double x = 1.0 / Math.Sin(num3) * (Math.Sin(num) * Math.Cos(num2) - Math.Sin(num2) * Math.Cos(num) * Math.Cos(p.LAN * OrbitDBG.Deg2Rad - s.LAN * OrbitDBG.Deg2Rad));
			double y = 1.0 / Math.Sin(num3) * (Math.Sin(num2) * Math.Sin(p.LAN * OrbitDBG.Deg2Rad - s.LAN * OrbitDBG.Deg2Rad));
			double num4 = Math.Atan2(y, x);
			double x2 = 1.0 / Math.Sin(num3) * (Math.Sin(num) * Math.Cos(num2) * Math.Cos(p.LAN * OrbitDBG.Deg2Rad - s.LAN * OrbitDBG.Deg2Rad) - Math.Sin(num2) * Math.Cos(num));
			double y2 = 1.0 / Math.Sin(num3) * (Math.Sin(num) * Math.Sin(p.LAN * OrbitDBG.Deg2Rad - s.LAN * OrbitDBG.Deg2Rad));
			double num5 = Math.Atan2(y2, x2);
			FFp = num4 - p.argumentOfPeriapsis * OrbitDBG.Deg2Rad;
			FFs = num5 - s.argumentOfPeriapsis * OrbitDBG.Deg2Rad;
			if (p.eccentricity == 0.0)
			{
				if (s.eccentricity == 0.0)
				{
					CD = Vector3d.Distance(p.getPositionFromTrueAnomaly(FFp), s.getPositionFromTrueAnomaly(FFs));
					CCD = CD;
				}
			}
			CD = OrbitDBG.SolveClosestBSP(ref FFp, ref FFs, num3, 3.1415926535897931, p, s, 0.0001, maxIterations, ref iterationCount);
			Debug.DrawLine(ScaledSpace.LocalToScaledSpace(p.referenceBody.position), ScaledSpace.LocalToScaledSpace(p.getPositionFromTrueAnomaly(FFp)), Color.green);
			Debug.DrawLine(ScaledSpace.LocalToScaledSpace(s.referenceBody.position), ScaledSpace.LocalToScaledSpace(s.getPositionFromTrueAnomaly(FFs)), Color.grey);
			SFp = FFp + 3.1415926535897931;
			SFs = FFs + 3.1415926535897931;
			CCD = OrbitDBG.SolveClosestBSP(ref SFp, ref SFs, num3, 1.5707963267948966, p, s, 0.0001, maxIterations, ref iterationCount);
			Debug.DrawLine(ScaledSpace.LocalToScaledSpace(p.referenceBody.position), ScaledSpace.LocalToScaledSpace(p.getPositionFromTrueAnomaly(SFp)), Color.cyan);
			Debug.DrawLine(ScaledSpace.LocalToScaledSpace(s.referenceBody.position), ScaledSpace.LocalToScaledSpace(s.getPositionFromTrueAnomaly(SFs)), Color.magenta);
			CD = Math.Sqrt(CD);
			CCD = Math.Sqrt(CCD);
		}
		static double SolveClosestBSP(ref double Fp, ref double Fs, double Ir, double dF, Orbit p, Orbit s, double epsilon, int maxIterations, ref int iterationCount)
		{
			double num = dF;
			double num2 = dF;
			if (Math.Abs(Ir) % 3.1415926535897931 * 2.0 > 1.5707963267948966)
			{
				num2 *= -1.0;
			}
			iterationCount = 0;
			double num3 = (p.getRelativePositionFromTrueAnomaly(Fp) - s.getRelativePositionFromTrueAnomaly(Fs)).sqrMagnitude;
			while (num > 0.0001)
			{
				if (iterationCount >= maxIterations)
				{
					return num3;
				}
				else
				{
					double sqrMagnitude = (p.getRelativePositionFromTrueAnomaly(Fp + num) - s.getRelativePositionFromTrueAnomaly(Fs + num2)).sqrMagnitude;
					double sqrMagnitude2 = (p.getRelativePositionFromTrueAnomaly(Fp - num) - s.getRelativePositionFromTrueAnomaly(Fs - num2)).sqrMagnitude;
					num3 = Math.Min(num3, Math.Min(sqrMagnitude, sqrMagnitude2));
					if (num3 == sqrMagnitude)
					{
						Fp += num;
						Fs += num2;
					}
					else if (num3 == sqrMagnitude2)
					{
						Fp -= num;
						Fs -= num2;
					}
					num *= 0.5;
					num2 *= 0.5;
					iterationCount++;
				}
			}
			return num3;
		}
		public static double SolveClosestApproach(Orbit p, Orbit s, ref double UT, double dT, double threshold, double MinUT, double MaxUT, double epsilon, int maxIterations, ref int iterationCount)
		{
			if (UT < MinUT)
			{
				return -1.0;
			}
			if (UT > MaxUT)
			{
				return -1.0;
			}
			iterationCount = 0;
			double num = Math.Abs((p.getPositionAtUT(UT) - s.getPositionAtUT(UT)).sqrMagnitude);
			while (dT > epsilon)
			{
				if (iterationCount >= maxIterations)
				{
					goto IL_1C2;
				}
				else
				{
					double num2 = (p.getPositionAtUT(UT + dT) - s.getPositionAtUT(UT + dT)).sqrMagnitude;
					double num3 = (p.getPositionAtUT(UT - dT) - s.getPositionAtUT(UT - dT)).sqrMagnitude;
					if (UT - dT < MinUT)
					{
						num3 = 1.7976931348623157E+308;
					}
					if (UT + dT > MaxUT)
					{
						num2 = 1.7976931348623157E+308;
					}
					num = Math.Min(num, Math.Min(num2, num3));
					if (num == num3)
					{
						UT -= dT;
					}
					else if (num == num2)
					{
						UT += dT;
					}
					dT /= 2.0;
					iterationCount++;
					Debug.DrawLine(ScaledSpace.LocalToScaledSpace(p.referenceBody.position), ScaledSpace.LocalToScaledSpace(p.getPositionAtUT(UT)), XKCDColors.Lime * 0.5f);
				}
			}
			IL_1C2:
			return Math.Sqrt(num);
		}
		public static bool SolveSOI_BSP(Orbit p, Orbit s, ref double UT, double dT, double Rsoi, double MinUT, double MaxUT, double epsilon, int maxIterations, ref int iterationCount)
		{
			if (UT < MinUT)
			{
				return false;
			}
			if (UT > MaxUT)
			{
				return false;
			}
			iterationCount = 0;
			bool result = false;
			double num = Rsoi * Rsoi;
			double num2 = Math.Abs((p.getPositionAtUT(UT) - s.getPositionAtUT(UT)).sqrMagnitude - num);
			while (dT > epsilon)
			{
				if (iterationCount < maxIterations)
				{
					double num3 = (p.getPositionAtUT(UT + dT) - s.getPositionAtUT(UT + dT)).sqrMagnitude - num;
					double num4 = (p.getPositionAtUT(UT - dT) - s.getPositionAtUT(UT - dT)).sqrMagnitude - num;
					if (UT - dT < MinUT)
					{
						num4 = 1.7976931348623157E+308;
					}
					if (UT + dT > MaxUT)
					{
						num3 = 1.7976931348623157E+308;
					}
					if (num2 < 0.0)
					{
						goto IL_146;
					}
					if (num3 < 0.0)
					{
						goto IL_146;
					}
					if (num4 < 0.0)
					{
						goto IL_146;
					}
					IL_148:
					num3 = Math.Abs(num3);
					num4 = Math.Abs(num4);
					num2 = Math.Min(num2, Math.Min(num3, num4));
					if (num2 == num4)
					{
						UT -= dT;
					}
					else if (num2 == num3)
					{
						UT += dT;
					}
					dT /= 2.0;
					iterationCount++;
					Debug.DrawLine(ScaledSpace.LocalToScaledSpace(p.referenceBody.position), ScaledSpace.LocalToScaledSpace(p.getPositionAtUT(UT)), XKCDColors.LightMagenta * 0.5f);
					continue;
					IL_146:
					result = true;
					goto IL_148;
				}
				return result;
			}
			return result;
		}
		public Trajectory GetPatchTrajectory(int sampleCount)
		{
			Vector3d[] array = new Vector3d[sampleCount];
			double[] array2 = new double[sampleCount];
			float[] array3 = new float[sampleCount];
			if (eccentricity < 1.0)
			{
				if (patchEndTransition != Orbit.PatchTransitionType.FINAL)
				{
					if (debug_returnFullEllipseTrajectory)
					{
					}
					else
					{
						fromV = TrueAnomalyAtUT(StartUT);
						toV = TrueAnomalyAtUT(EndUT);
						fromE = GetEccentricAnomaly(fromV);
						toE = GetEccentricAnomaly(toV);
						if (fromV > toV)
						{
							fromE = -(6.2831853071795862 - fromE);
						}
						sampleInterval = (toE - fromE) / (double)(sampleCount - 5);
						fromE -= sampleInterval * 2.0;
						fromV = getTrueAnomaly(fromE);
						double dTforTrueAnomaly = GetDTforTrueAnomaly(fromV, 0.0);
						for (int i = 0; i < sampleCount; i++)
						{
							E = fromE + sampleInterval * (double)i;
							V = getTrueAnomaly(E);
							array2[i] = V;
							array3[i] = (float)(StartUT + GetDTforTrueAnomaly(V, dTforTrueAnomaly));
							array[i] = getRelativePositionFromEccAnomaly(E).xzy;
						}
						goto IL_277;
					}
				}
				sampleInterval = 6.2831853071795862 / (double)sampleCount;
				for (int j = 0; j < sampleCount; j++)
				{
					E = (double)j * sampleInterval;
					V = getTrueAnomaly(E);
					array3[j] = (float)(StartUT + GetDTforTrueAnomaly(V, 1.7976931348623157E+308));
					array[j] = getRelativePositionFromEccAnomaly(E).xzy;
				}
				IL_277:;
			}
			else
			{
				fromV = TrueAnomalyAtUT(StartUT);
				toV = TrueAnomalyAtUT(EndUT);
				fromE = GetEccentricAnomaly(fromV);
				toE = GetEccentricAnomaly(toV);
				if (fromV > 3.1415926535897931)
				{
					fromE = -fromE;
				}
				sampleInterval = (toE - fromE) / (double)(sampleCount - 1);
				for (int k = 0; k < sampleCount; k++)
				{
					E = fromE + sampleInterval * (double)k;
					V = getTrueAnomaly(E);
					array3[k] = (float)(StartUT + GetDTforTrueAnomaly(V, 1.7976931348623157E+308));
					array[k] = getRelativePositionFromEccAnomaly(E).xzy;
				}
			}
			Vector3d pe;
			Vector3d ap;
			if (eccentricity < 1.0)
			{
				pe = getRelativePositionAtT(0.0).xzy;
				ap = getRelativePositionAtT(period * 0.5).xzy;
			}
			else
			{
				pe = GetEccVector().xzy.normalized * (-semiMajorAxis * (eccentricity - 1.0));
				ap = Vector3d.zero;
			}
			Vector3d xzy = getRelativePositionAtUT(StartUT).xzy;
			Vector3d xzy2 = getRelativePositionAtUT(EndUT).xzy;
			return new Trajectory(array, array3, array2, pe, ap, xzy, xzy2, Vector3d.zero, 
			                      new Orbit(inclination,
			                                eccentricity,
			                                semiMajorAxis,
			                                LAN,
			                                argumentOfPeriapsis,
			                                meanAnomalyAtEpoch,
			                                epoch,
			                                referenceBody));
		}
		public static Orbit CreateRandomOrbitAround(CelestialBody body)
		{
			Orbit orbit = new Orbit();
			orbit.referenceBody = body;
			orbit.eccentricity = (double)UnityEngine.Random.Range(0.0001f, 0.01f);
			orbit.semiMajorAxis = (double)UnityEngine.Random.Range((float)body.atmosphereDepth, (float)body.sphereOfInfluence);
			orbit.inclination = (double)UnityEngine.Random.Range(-0.001f, 0.001f);
			orbit.LAN = (double)UnityEngine.Random.Range(0.999f, 1.001f);
			orbit.argumentOfPeriapsis = (double)UnityEngine.Random.Range(0.999f, 1.001f);
			orbit.meanAnomalyAtEpoch = (double)UnityEngine.Random.Range(0.999f, 1.001f);
			orbit.epoch = (double)UnityEngine.Random.Range(0.999f, 1.001f);
			orbit.Init();
			return orbit;
		}
		public static Orbit CreateRandomOrbitAround(CelestialBody body, double minAltitude, double maxAltitude)
		{
			Orbit orbit = new Orbit();
			orbit.referenceBody = body;
			orbit.eccentricity = (double)UnityEngine.Random.Range(0.0001f, 0.01f);
			orbit.semiMajorAxis = (double)UnityEngine.Random.Range((float)minAltitude, (float)maxAltitude);
			orbit.inclination = (double)UnityEngine.Random.Range(-0.001f, 0.001f);
			orbit.LAN = (double)UnityEngine.Random.Range(0.999f, 1.001f);
			orbit.argumentOfPeriapsis = (double)UnityEngine.Random.Range(0.999f, 1.001f);
			orbit.meanAnomalyAtEpoch = (double)UnityEngine.Random.Range(0.999f, 1.001f);
			orbit.epoch = (double)UnityEngine.Random.Range(0.999f, 1.001f);
			orbit.Init();
			return orbit;
		}
		public static Orbit CreateRandomOrbitNearby(Orbit baseOrbit)
		{
			Orbit orbit = new Orbit();
			orbit.eccentricity = baseOrbit.eccentricity + (double)UnityEngine.Random.Range(0.0001f, 0.01f);
			orbit.semiMajorAxis = baseOrbit.semiMajorAxis * (double)UnityEngine.Random.Range(0.999f, 1.001f);
			orbit.inclination = baseOrbit.inclination + (double)UnityEngine.Random.Range(-0.001f, 0.001f);
			orbit.LAN = baseOrbit.LAN * (double)UnityEngine.Random.Range(0.999f, 1.001f);
			orbit.argumentOfPeriapsis = baseOrbit.argumentOfPeriapsis * (double)UnityEngine.Random.Range(0.999f, 1.001f);
			orbit.meanAnomalyAtEpoch = baseOrbit.meanAnomalyAtEpoch * (double)UnityEngine.Random.Range(0.999f, 1.001f);
			orbit.epoch = baseOrbit.epoch;
			orbit.referenceBody = baseOrbit.referenceBody;
			orbit.Init();
			return orbit;
		}
		public static Orbit CreateRandomOrbitFlyBy(CelestialBody tgtBody, double daysToClosestApproach)
		{
			double periapsis = Math.Max(tgtBody.Radius * 3.0, tgtBody.sphereOfInfluence * (double)UnityEngine.Random.Range(0f, 1.1f));
			double deltaVatPeriapsis = (double)UnityEngine.Random.Range(100f, 500f);
			return Orbit.CreateRandomOrbitFlyBy(tgtBody.orbit, daysToClosestApproach * 24.0 * 60.0 * 60.0, periapsis, deltaVatPeriapsis);
		}
		public static Orbit CreateRandomOrbitFlyBy(Orbit targetOrbit, double timeToPeriapsis, double periapsis, double deltaVatPeriapsis)
		{
			double universalTime = Planetarium.GetUniversalTime();
			Vector3d relativePositionAtUT = targetOrbit.getRelativePositionAtUT(universalTime + timeToPeriapsis);
			Vector3d orbitalVelocityAtUT = targetOrbit.getOrbitalVelocityAtUT(universalTime + timeToPeriapsis);
			Orbit orbit = new Orbit();
			Vector3d vector3d = relativePositionAtUT + (Vector3d)UnityEngine.Random.onUnitSphere * periapsis;
			Vector3d vector3d2 = orbitalVelocityAtUT + (orbitalVelocityAtUT.normalized + UnityEngine.Random.onUnitSphere) * deltaVatPeriapsis;
			orbit.UpdateFromStateVectors(vector3d, vector3d2, targetOrbit.referenceBody, universalTime + timeToPeriapsis);
			return orbit;
		}
	}
}
#endif