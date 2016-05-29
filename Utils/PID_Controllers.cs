//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

using System;
using UnityEngine;

namespace ThrottleControlledAvionics
{
	#region PI Controllers
	public class PI_Controller : ConfigNodeObject
	{
		new public const string NODE_NAME = "PICONTROLLER";

		//buggy: need to be public to be persistent
		[Persistent] public float p = 0.5f, i = 0.5f; //some default values
		protected PI_Controller master;

		public float P { get { return master == null? p : master.P; } set { p = value; } }
		public float I { get { return master == null? i : master.I; } set { i = value; } }

		public PI_Controller() {}
		public PI_Controller(float P, float I) { p = P; i = I; }

		public void setPI(PI_Controller other) { p = other.P; i = other.I; }
		public void setMaster(PI_Controller master) { this.master = master; }

		public virtual void DrawControls(string name)
		{
			GUILayout.BeginHorizontal();
			GUILayout.Label(name, GUILayout.ExpandWidth(false));
			p = Utils.FloatSlider(" P", P, 0, TCAScenario.Globals.ENG.MaxP, "F2");
			i = Utils.FloatSlider(" I", I, 0, TCAScenario.Globals.ENG.MaxI, "F2");
			GUILayout.EndHorizontal();
		}

		public override string ToString() { return string.Format("[P={0}, I={1}]", P, I); }
	}

	public abstract class PI_Controller<T> : PI_Controller
	{
		protected T action = default(T);
		protected T integral_error = default(T);

		public abstract void Update(T error);

		public void Reset() 
		{ action = default(T); integral_error = default(T); }

		//access
		public T Action { get { return action; } }
		public static implicit operator T(PI_Controller<T> c) { return c.action; }
	}

	public class PIf_Controller : PI_Controller<float>
	{
		public override void Update(float error)
		{
			integral_error += error * TimeWarp.fixedDeltaTime;
			action = error * P + integral_error * I;
		}
	}
	#endregion

	#region PID Controllers
	public class PID_Controller : ConfigNodeObject
	{
		new public const string NODE_NAME = "PIDCONTROLLER";

		[Persistent] public float Min = -1, Max = 1;
		[Persistent] public float P = 0.9f, I = 0.1f, D = 0.02f;

		public PID_Controller() {}
		public PID_Controller(float p, float i, float d, float min, float max)
		{ P = p; I = i; D = d; Min = min; Max = max; }

		public void setPID(PID_Controller c)
		{ P = c.P; I = c.I; D = c.D; Min = c.Min; Max = c.Max; }

		public virtual void Reset() {}

		public override string ToString()
		{ return string.Format("[P={0}, I={1}, D={2}, Min={3}, Max={4}]", P, I, D, Min, Max); }
	}

	public class PID_Controller<T> : PID_Controller
	{
		protected T action;
		protected T last_error;
		protected T integral_error;

		public override void Reset() 
		{ action = default(T); integral_error = default(T); }

		//access
		public T Action { get { return action; } }
		public static implicit operator T(PID_Controller<T> c) { return c.action; }

		public override string ToString()
		{
			return base.ToString()+
				string.Format("\nLast Error:     {0}" +
				              "\nIntegral Error: {1}" +
				              "\nAction:         {2}\n",
				              last_error, 
				              integral_error, 
				              action);
		}
	}

	//separate implementation of the strange PID controller from MechJeb2
	public class PIDv_Controller2 : PID_Controller<Vector3>
	{
		public PIDv_Controller2() {}
		public PIDv_Controller2(float p, float i, float d, float min, float max)
		{ P = p; I = i; D = d; Min = min; Max = max; }

		public void Update(Vector3 error, Vector3 omega)
		{
			var derivative   = D * omega/TimeWarp.fixedDeltaTime;
			integral_error.x = (Mathf.Abs(derivative.x) < 0.6f * Max) ? integral_error.x + (error.x * I * TimeWarp.fixedDeltaTime) : 0.9f * integral_error.x;
			integral_error.y = (Mathf.Abs(derivative.y) < 0.6f * Max) ? integral_error.y + (error.y * I * TimeWarp.fixedDeltaTime) : 0.9f * integral_error.y;
			integral_error.z = (Mathf.Abs(derivative.z) < 0.6f * Max) ? integral_error.z + (error.z * I * TimeWarp.fixedDeltaTime) : 0.9f * integral_error.z;
			Vector3.ClampMagnitude(integral_error, Max);
			var act = error * P + integral_error + derivative;
			action = new Vector3
				(
					float.IsNaN(act.x)? 0f : Mathf.Clamp(act.x, Min, Max),
					float.IsNaN(act.y)? 0f : Mathf.Clamp(act.y, Min, Max),
					float.IsNaN(act.z)? 0f : Mathf.Clamp(act.z, Min, Max)
				);
		}
	}

	public class PIDvd_Controller : PID_Controller<Vector3d>
	{
		public PIDvd_Controller() {}
		public PIDvd_Controller(float p, float i, float d, float min, float max)
		{ P = p; I = i; D = d; Min = min; Max = max; }

		public void Update(Vector3d error)
		{
			if(last_error.IsZero()) last_error = error;
			var old_ierror = integral_error;
			integral_error += error*TimeWarp.fixedDeltaTime;
			var act = P*error + I*integral_error + D*(error-last_error)/TimeWarp.fixedDeltaTime;
			if(act.IsZero()) action = act;
			else
			{
				action = new Vector3d
					(
						double.IsNaN(act.x)? 0f : Utils.Clamp(act.x, Min, Max),
						double.IsNaN(act.y)? 0f : Utils.Clamp(act.y, Min, Max),
						double.IsNaN(act.z)? 0f : Utils.Clamp(act.z, Min, Max)
					);
				if(!act.Equals(action)) integral_error = old_ierror;
			}
			//			Utils.Log("{0}\nPe {1}; Ie {2}; De {3}; error {4}, action {5}", 
			//			          this, P*error, I*integral_error, D*(error-last_error)/TimeWarp.fixedDeltaTime, error, action);//debug
			last_error = error;
		}
	}

	public class PIDf_Controller : PID_Controller<float>
	{
		public PIDf_Controller() {}
		public PIDf_Controller(float p, float i, float d, float min, float max)
		{ P = p; I = i; D = d; Min = min; Max = max; }

		public void Update(float error
		                   #if DEBUG
		                   , bool debug = false
		                   #endif
		                  )
		{
			if(float.IsNaN(error)) return;
			if(last_error.Equals(0)) last_error = error;
			var old_ierror = integral_error;
			integral_error += error*TimeWarp.fixedDeltaTime;
			var act = P*error + I*integral_error + D*(error-last_error)/TimeWarp.fixedDeltaTime;
			action = Mathf.Clamp(act, Min, Max);
			if(!act.Equals(action)) integral_error = old_ierror;
			#if DEBUG
			if(debug)
				Utils.Log("{0}\nPe {1}; Ie {2}; De {3}; error {4}, action {5}", 
				          this, P*error, I*integral_error, D*(error-last_error)/TimeWarp.fixedDeltaTime, error, action);
			#endif
			last_error = error;
		}
	}

	public class PIDf_Controller2 : PID_Controller<float>
	{
		public PIDf_Controller2() {}
		public PIDf_Controller2(float p, float i, float d, float min, float max)
		{ P = p; I = i; D = d; Min = min; Max = max; }

		public void Update(float error, float speed = float.NaN)
		{
			if(float.IsNaN(error)) return;
			if(last_error.Equals(0)) last_error = error;
			var derivative = D * (float.IsNaN(speed)? (error-last_error)/TimeWarp.fixedDeltaTime : speed);
			integral_error = Mathf.Clamp((Math.Abs(derivative) < 0.6f * Max) ? integral_error + (error * I * TimeWarp.fixedDeltaTime) : 0.9f * integral_error, Min, Max);
			var act = error * P + integral_error + derivative;
			if(!float.IsNaN(act)) action = Mathf.Clamp(act, Min, Max);
//			Utils.LogF("error {}; Pe {}; Ie {}; De {}; action {}", error, P*error, integral_error, derivative, action);//debug
			last_error = error;
		}
	}
	#endregion
}

