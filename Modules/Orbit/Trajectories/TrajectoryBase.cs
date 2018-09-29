//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2016 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

using System;
using AT_Utils;

namespace ThrottleControlledAvionics
{
    public abstract class BaseTrajectory : ConfigNodeObject
    {
        internal static Globals GLB { get { return Globals.Instance; } }

        public VesselWrapper VSL;

        [Persistent] public Orbit OrigOrbit;
        public Orbit StartOrbit => TrajectoryCalculator.NextOrbit(OrigOrbit, StartUT);

        [Persistent] public Orbit  Orbit;
        public CelestialBody Body => Orbit.referenceBody;

        [Persistent] public double StartUT;
        public double TimeToStart => StartUT-VSL.Physics.UT;

        [Persistent] public Vector3d ManeuverDeltaV;
        [Persistent] public Vector3d NodeDeltaV;
        [Persistent] public double ManeuverDuration;
        [Persistent] public float  ManeuverFuel;
        [Persistent] public bool   FullManeuver;

        [Persistent] public Vector3d StartPos;
        [Persistent] public Vector3d StartVel;
        [Persistent] public Vector3d AfterStartVel;

        public virtual double GetTotalDeltaV() { return ManeuverDeltaV.magnitude; }
        public virtual float GetTotalFuel() { return ManeuverFuel; }

        public virtual void AttachVSL(VesselWrapper vsl) =>
        VSL = vsl;

        protected BaseTrajectory(VesselWrapper vsl, Vector3d dV, double startUT)
        {
            VSL = vsl;
            var dVm = (float)dV.magnitude;
            if(dVm > 0) 
            {
                ManeuverFuel = VSL.Engines.FuelNeeded(dVm);
                FullManeuver = ManeuverFuel < VSL.Engines.AvailableFuelMass;
                ManeuverDuration = VSL.Engines.TTB_Precise(dVm);
            }
            else 
            {
                ManeuverFuel = 0;
                ManeuverDuration = 0;
                FullManeuver = true;
            }
            StartUT = startUT;
            OrigOrbit = VSL.vessel.orbitDriver.orbit;
            var obt = StartOrbit;
            if(dVm > 0)
            {
                ManeuverDeltaV = dV;
                NodeDeltaV = ManeuverAutopilot.Orbital2NodeDeltaV(obt, ManeuverDeltaV, StartUT);
                try 
                { 
                    Orbit = TrajectoryCalculator.NewOrbit(obt, ManeuverDeltaV, StartUT);
                    var prev = Orbit;
                    for(int i = 0; i < 3; i++)
                    {
                        if(!PatchedConics.CalculatePatch(prev, prev.nextPatch ?? new Orbit(), prev.epoch, new PatchedConics.SolverParameters(), null)) break;
                        prev = prev.nextPatch;
                    }
//                    if(Orbit.patchEndTransition != Orbit.PatchTransitionType.FINAL)//debug
//                    {
//                        Utils.Log("**************************************************************************************************");//debug
//                        RendezvousAutopilot.log_patches(Orbit, "Orbit");//deubg
//                    }
                }
                catch(ArithmeticException) 
                { 
                    Orbit = OrigOrbit;
                    StartUT = VSL.Physics.UT;
                    ManeuverFuel = 0;
                    ManeuverDuration = 0;
                    ManeuverDeltaV = Vector3d.zero;
                    FullManeuver = true;
                }
            }
            else Orbit = StartOrbit;
            StartPos = obt.getRelativePositionAtUT(StartUT);
            StartVel = obt.getOrbitalVelocityAtUT(StartUT);
            AfterStartVel = Orbit.getOrbitalVelocityAtUT(StartUT);
        }

        public virtual void UpdateOrbit(Orbit current)
        {
            StartUT = VSL.Physics.UT;
            Orbit = current;
            ManeuverDeltaV = Vector3d.zero;
            ManeuverDuration = 0;
            FullManeuver = true;
            StartPos = Orbit.pos;
            StartVel = Orbit.vel;
            AfterStartVel = Orbit.vel;
        }

        public override string ToString()
        {
            return Utils.Format("[{}]\n" +
                                "{}\n" +
                                "{}\n" +
                                "StartUT: {} s, TimeToStart: {} s, ManeuverDuration: {} s\n" +
                                "ManeuverDeltaV: {}\n" +
                                "ManeuverFuel: {}, Enough: {}", 
                                GetType().Name, 
                                Utils.formatPatches(OrigOrbit, "OrigOrbit"), 
                                Utils.formatPatches(Orbit, "NewOrbit"), 
                                StartUT, TimeToStart, 
                                ManeuverDuration, ManeuverDeltaV, 
                                ManeuverFuel, FullManeuver);
        }
    }

    public abstract class TargetedTrajectoryBase : BaseTrajectory
    {
        [Persistent] public double AtTargetUT;
        [Persistent] public double TransferTime;
        [Persistent] public Vector3d AtTargetPos;
        [Persistent] public Vector3d AtTargetVel;
        [Persistent] public double DistanceToTarget = double.MaxValue;
        [Persistent] public double DeltaFi = double.MaxValue;

        public double TimeToTarget => AtTargetUT-VSL.Physics.UT;
        public double RelDistanceToTarget => DistanceToTarget/Orbit.semiMajorAxis;

        public Orbit EndOrbit => TrajectoryCalculator.NextOrbit(Orbit, AtTargetUT);

        [Persistent] public Vector3d BrakeDeltaV;
        [Persistent] public float BrakeFuel;
        [Persistent] public float BrakeDuration;
        [Persistent] public bool FullBrake;

        protected TargetedTrajectoryBase(VesselWrapper vsl, Vector3d dV, double startUT) 
            : base(vsl, dV, startUT) {}

        public override double GetTotalDeltaV() { return base.GetTotalDeltaV()+BrakeDeltaV.magnitude; }
        public override float GetTotalFuel() { return base.GetTotalFuel()+BrakeFuel; }

        /// <summary>
        /// The numeric "quality" of the trajectory. The less, the better, i.e. closer to the target for less dV.
        /// </summary>
        public double Quality => DistanceToTarget + ManeuverDeltaV.magnitude+BrakeDeltaV.magnitude;

        public override string ToString()
        { 
            return base.ToString()+
                Utils.Format("\nDistance To Target: {} m\n" +
                             "Transfer Time:  {} s\n" +
                             "Time To Target: {} s\n" +
                             "AtTargetUT {}\n\n" +

                             "Brake DeltaV:   {} m/s\n" +
                             "Brake Fuel: {}, Full Brake: {}\n" +
                             "Brake Duration: {} s\n" +
                             "DeltaFi: {} deg\n",
                             DistanceToTarget, TransferTime, TimeToTarget, AtTargetUT,
                             BrakeDeltaV, BrakeFuel, FullBrake, BrakeDuration,
                             DeltaFi);
        }
    }

    public abstract class TargetedTrajectory : TargetedTrajectoryBase
    {
        [Persistent] public WayPoint Target;

        protected TargetedTrajectory(VesselWrapper vsl, Vector3d dV, double startUT, WayPoint target) 
            : base(vsl, dV, startUT) { Target = target; }

        public override string ToString()
        { 
            return base.ToString()+
                string.Format("\nTarget: {0}", Target); 
        }
    }
}

