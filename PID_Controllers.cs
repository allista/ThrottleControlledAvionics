/* Name: Throttle Controlled Avionics, Fork by Allis Tauri
 *
 * Authors: Quinten Feys & Willem van Vliet & Allis Tauri
 * License: BY: Attribution-ShareAlike 3.0 Unported (CC BY-SA 3.0): 
 * http://creativecommons.org/licenses/by-sa/3.0/
 * 
 */

using System;
using UnityEngine;

namespace ThrottleControlledAvionics
{
	public class PI_Controller : ConfigNodeObject
	{
		new public const string NODE_NAME = "PICONTROLLER";

		//buggy: need to be public to be persistent
		[Persistent] public float p = 0.5f, i = 0.5f; //some default values
		protected PI_Controller master;

		public float P { get { return master == null? p : master.P; } set { p = value; } }
		public float I { get { return master == null? i : master.I; } set { i = value; } }

		public void setPI(PI_Controller other) { p = other.P; i = other.I; }
		public void setMaster(PI_Controller master) { this.master = master; }

		public virtual void DrawControls(string name)
		{
			GUILayout.BeginHorizontal();
			GUILayout.Label(name, GUILayout.ExpandWidth(false));
			p = Utils.FloatSlider(" P", P, 0, TCAConfiguration.Globals.MaxP, "F2");
			i = Utils.FloatSlider(" I", I, 0, TCAConfiguration.Globals.MaxI, "F2");
			GUILayout.EndHorizontal();
		}
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

	public class PI_Dummy : PI_Controller
	{ 
		public PI_Dummy() {}
		public PI_Dummy(float P, float I) { p = P; i = I; }
	}

	public class PIv_Controller : PI_Controller<Vector3>
	{
		public override void Update(Vector3 error)
		{
			integral_error += error * TimeWarp.fixedDeltaTime;
			action = error * P + integral_error * I;
		}
	}

	//I hate strongly-typed languages! =(
	public class PIf_Controller : PI_Controller<float>
	{
		public override void Update(float error)
		{
			integral_error += error * TimeWarp.fixedDeltaTime;
			action = error * P + integral_error * I;
		}
	}

	//separate implementation of the strange PID controller from MechJeb2
	public class PIDv_Controller :  ConfigNodeObject
	{
		new public const string NODE_NAME = "PIDCONTROLLER";

		[Persistent] public float Min = -1, Max = 1;
		[Persistent] public float P = 0.9f, I = 0.1f, D = 0.02f;

		public PIDv_Controller() {}
		public PIDv_Controller(float p, float i, float d, float min, float max)
		{ P = p; I = i; D = d; Min = min; Max = max; }

		protected Vector3 action = Vector3.zero;
		protected Vector3 integral_error = Vector3.zero;

		public void Reset() 
		{ action = Vector3.zero; integral_error = Vector3.zero; }

		//access
		public Vector3 Action { get { return action; } }
		public static implicit operator Vector3(PIDv_Controller c) { return c.action; }

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

	public class PIDf_Controller :  ConfigNodeObject
	{
		new public const string NODE_NAME = "PIDCONTROLLER";

		[Persistent] public float Min = -1, Max = 1, eps = 0.01f;
		[Persistent] public float P = 0.9f, I = 0.1f, D = 0.02f;

		public PIDf_Controller() {}
		public PIDf_Controller(float p, float i, float d, float min, float max, float e = 0.01f)
		{ P = p; I = i; D = d; Min = min; Max = max; eps = e; }

		protected float action;
		protected float last_error;
		protected float integral_error;

		public void Reset() 
		{ action = 0f; integral_error = 0f; last_error = 0f; }

		public void setPID(PIDf_Controller c)
		{ P = c.P; I = c.I; D = c.D; Min = c.Min; Max = c.Max; eps = c.eps; }

		//access
		public float Action { get { return action; } }
		public static implicit operator float(PIDf_Controller c) { return c.action; }

		public void Update(float error)
		{
			var old_ierror = integral_error;
			integral_error += error*TimeWarp.fixedDeltaTime;
			var act = P*error + I*integral_error + D*(error-last_error)/TimeWarp.fixedDeltaTime;
			action = Mathf.Clamp(act, Min, Max);
			if(Mathf.Abs(action) < eps) action = 0f;
			if(!act.Equals(action)) integral_error = old_ierror;
			last_error = error;
		}
	}
}

