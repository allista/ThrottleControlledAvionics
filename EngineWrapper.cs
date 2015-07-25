//------------------------------------------------------------------------------
// This EngineWrapper class came from HoneyFox's EngineIgnitor mod:
// Edited by Willem van Vliet 2014, by Allis Tauri 2015
// More info: https://github.com/HoneyFox/EngineIgnitor
//------------------------------------------------------------------------------

using System;
using UnityEngine;

namespace ThrottleControlledAvionics
{
	public class EngineWrapper
	{
		public readonly ModuleEngines engine;
		public readonly TCAEngineInfo einfo;

		public static readonly PI_Dummy ThrustPI = new PI_Dummy();
		PIf_Controller thrustController = new PIf_Controller();
		public Vector3 specificTorque   = Vector3.zero;
		public Vector3 currentTorque    = Vector3.zero;
		public Vector3 thrustDirection  = Vector3.zero;
		public float   limit, best_limit, limit_tmp;
		public float   currentTorque_m;
		public bool    throttleLocked;
		public float   throttle;
		public float   thrustMod;
		public float   VSF; //vertical speed factor
		public bool    isVSC; //vertical speed controller
		public TCARole Role;
		public CenterOfThrustQuery thrustInfo;

		readonly float zeroISP;

		public EngineWrapper(ModuleEngines engine) 
		{
			thrustController.setMaster(ThrustPI);
			einfo = engine.part.GetModule<TCAEngineInfo>();
			zeroISP = engine.atmosphereCurve.Evaluate(0f);
			this.engine = engine;
		}

		#region methods
		public void InitLimits()
		{
			isVSC = false;
			switch(Role)
			{
			case TCARole.MAIN:
			case TCARole.BALANCE:
				limit = best_limit = 1f;
				isVSC = true;
				break;
			case TCARole.MANEUVER:
				limit = best_limit = 0f;
				break;
			case TCARole.MANUAL:
				limit = best_limit = thrustPercentage/100;
				break;
			}
		}

		public void InitState()
		{
			//update thrust info
			thrustInfo = new CenterOfThrustQuery();
			engine.OnCenterOfThrustQuery(thrustInfo);
			thrustInfo.dir.Normalize();
			//compute velocity and atmosphere thrust modifier
			thrustMod = engine.atmosphereCurve.Evaluate((float)(engine.vessel.staticPressurekPa * PhysicsGlobals.KpaToAtmospheres))/zeroISP;
			if(engine.atmChangeFlow)
			{
				thrustMod = (float)(part.atmDensity / 1.225);
				if(engine.useAtmCurve)
					thrustMod = engine.atmCurve.Evaluate(thrustMod);
			}
			if(engine.useVelCurve)
				thrustMod *= engine.velCurve.Evaluate((float)part.machNumber);
			//update Role
			throttleLocked =  engine.throttleLocked;
			if(einfo != null)
			{
				if(throttleLocked && einfo.Role != TCARole.MANUAL) 
					einfo.SetRole(TCARole.MANUAL);
				Role = einfo.Role;
			} else Role = TCARole.MAIN;
			InitLimits();
		}

		public Vector3 Torque(float throttle)
		{
			return specificTorque * 
				(Role != TCARole.MANUAL? 
				 nominalCurrentThrust(throttle) :
				 finalThrust);
		}

		/// <summary>
		/// If the wrapper is still points to the valid ModuleEngines(FX)
		/// </summary>
		public bool Valid 
		{ get { return engine.part != null && engine.vessel != null; } }
		#endregion

		#region Accessors
		public Vessel vessel
		{ get { return engine.vessel; } }

		public Part part
		{ get { return engine.part; } }

		public float nominalCurrentThrust(float throttle)
		{ return thrustMod * (throttleLocked ? maxThrust : Mathf.Lerp(minThrust, maxThrust, throttle)); }

		public float finalThrust
		{ get { return engine.finalThrust; } }

		public float maxThrust
		{ get { return engine.maxThrust; } }

		public float minThrust
		{ get { return engine.minThrust; } }

		public float thrustPercentage
		{
			get { return engine.thrustPercentage; }
			set
			{
				thrustController.Update(value-thrustPercentage);
				engine.thrustPercentage = Mathf.Clamp(engine.thrustPercentage+thrustController, 0, 100);
			}
		}

		public void forceThrustPercentage(float value) { engine.thrustPercentage = value; }

		public bool isOperational { get { return engine.isOperational; } }

		public string name
		{
			get 
			{
				var r = "";
				r += engine.part.name;
				if(engine.engineID.Length > 0)
					r += " (" + engine.engineID + ")";
				return r;
			}
		}
		#endregion
	}
}
