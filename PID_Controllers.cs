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
			p = Utils.FloatSlider(" P", P, 0, TCAConfiguration.Globals.ENG.MaxP, "F2");
			i = Utils.FloatSlider(" I", I, 0, TCAConfiguration.Globals.ENG.MaxI, "F2");
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
	}

	//separate implementation of the strange PID controller from MechJeb2
	public class PIDv_Controller : PID_Controller<Vector3>
	{
		public PIDv_Controller() {}
		public PIDv_Controller(float p, float i, float d, float min, float max)
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

	public class PIDf_Controller : PID_Controller<float>
	{
		public PIDf_Controller() {}
		public PIDf_Controller(float p, float i, float d, float min, float max)
		{ P = p; I = i; D = d; Min = min; Max = max; }

		public void Update(float error)
		{
			var old_ierror = integral_error;
			integral_error += error*TimeWarp.fixedDeltaTime;
			var act = P*error + I*integral_error + D*(error-last_error)/TimeWarp.fixedDeltaTime;
			action = Mathf.Clamp(act, Min, Max);
			if(!act.Equals(action)) integral_error = old_ierror;
			last_error = error;
		}
	}

	public class PIDf_Controller2 : PID_Controller<float>
	{
		public PIDf_Controller2() {}
		public PIDf_Controller2(float p, float i, float d, float min, float max)
		{ P = p; I = i; D = d; Min = min; Max = max; }

		public void Update(float error)
		{
			var derivative = D * (error - last_error)/TimeWarp.fixedDeltaTime;
			integral_error = (Math.Abs(derivative) < 0.6f * Max) ? integral_error + (error * I * TimeWarp.fixedDeltaTime) : 0.9f * integral_error;
			if(integral_error > Max) integral_error = Max;
			action = Mathf.Clamp(error * P + integral_error + derivative, Min, Max);
		}
	}
	#endregion
}

