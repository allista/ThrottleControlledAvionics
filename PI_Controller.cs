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

		[Persistent] public float p = 0.8f, i = 0.3f; //some default values
		protected PI_Controller master;

		public float P { get { return master == null? p : master.P; } set { p = value; } }
		public float I { get { return master == null? i : master.I; } set { i = value; } }

		public void setPI(float P, float I) { p = P; i = I; }
		public void setPI(PI_Controller other) { p = other.P; i = other.I; }
		public void setMaster(PI_Controller master) { this.master = master; }

		public void DrawPIControls(string name)
		{
			GUILayout.BeginHorizontal();
			GUILayout.Label(name+" P: ", GUILayout.ExpandWidth(false));
			GUILayout.Label(P.ToString("F2"), GUILayout.ExpandWidth(false));
			p = GUILayout.HorizontalSlider(P, 0f, 2f);
			GUILayout.Label("I: ", GUILayout.ExpandWidth(false));
			GUILayout.Label(I.ToString("F2"), GUILayout.ExpandWidth(false));
			i = GUILayout.HorizontalSlider(I, 0f, 2f);
			GUILayout.EndHorizontal();
		}
	}

	public abstract class PI_Controller<T> : PI_Controller
	{
		protected T value = default(T);
		protected T integral_error = default(T);

		public abstract void Update(T new_value);

		//access
		public T Value { get { return value; } }
		public static implicit operator T(PI_Controller<T> c) { return c.value; }
	}

	public class PI_Dummy : PI_Controller<int> 
	{ 
		public PI_Dummy() {}
		public PI_Dummy(float P, float I) { p = P; i = I; }
		public override void Update(int new_value) {} 
	}

	public class PIv_Controller : PI_Controller<Vector3>
	{
		public override void Update(Vector3 new_value)
		{
			var error = new_value - value;
			integral_error += error * TimeWarp.fixedDeltaTime;
			value = new_value * P + integral_error * I;
		}
	}

	//I hate strongly-typed languages! =(
	public class PIf_Controller : PI_Controller<float>
	{
		public override void Update(float new_value)
		{
			var error = new_value - value;
			integral_error += error * TimeWarp.fixedDeltaTime;
			value = new_value * P + integral_error * I;
		}
	}
}

