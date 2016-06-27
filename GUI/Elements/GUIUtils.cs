//  Author:
//       allis <>
//
//  Copyright (c) 2016 allis
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.
//
using System;
using UnityEngine;

namespace ThrottleControlledAvionics
{
	public static partial class Utils
	{
		public static float FloatSlider(string name, float value, float min, float max, string format="F1", int label_width = -1, string tooltip = "")
		{
			var label = name.Length > 0? string.Format("{0}: {1}", name, value.ToString(format)) : value.ToString(format);
			GUILayout.Label(new GUIContent(label, tooltip), label_width > 0? GUILayout.Width(label_width) : GUILayout.ExpandWidth(false));
			return GUILayout.HorizontalSlider(value, min, max, GUILayout.ExpandWidth(true));
		}

		public static int IntSelector(int value, int min, int max=int.MaxValue, string format="D", string tooltip = "")
		{
			if(GUILayout.Button("<", Styles.normal_button, GUILayout.Width(15)))
			{ if(value >= min) value--; }
			GUILayout.Label(new GUIContent(value < min? "Off" : value.ToString(format), tooltip), 
				GUILayout.Width(20));
			if(GUILayout.Button(">", Styles.normal_button, GUILayout.Width(15)))
			{ if(value <= max) value++; }
			return value;
		}

		public static bool ButtonSwitch(string name, bool current_value, string tooltip = "", params GUILayoutOption[] options)
		{
			return string.IsNullOrEmpty(tooltip)? 
				GUILayout.Button(name, current_value ? Styles.enabled_button : Styles.active_button, options) : 
				GUILayout.Button(new GUIContent(name, tooltip), current_value ? Styles.enabled_button : Styles.active_button, options);
		}

		public static bool ButtonSwitch(string name, ref bool current_value, string tooltip = "", params GUILayoutOption[] options)
		{
			var ret = ButtonSwitch(name, current_value, tooltip, options);
			if(ret) current_value = !current_value;
			return ret;
		}

		#region ControlLock
		//modified from Kerbal Alarm Clock mod
		public static void LockEditor(string LockName, bool Lock=true)
		{
			if(Lock && InputLockManager.GetControlLock(LockName) != ControlTypes.EDITOR_LOCK)
				InputLockManager.SetControlLock(ControlTypes.EDITOR_LOCK, LockName);
			else if(!Lock && InputLockManager.GetControlLock(LockName) == ControlTypes.EDITOR_LOCK) 
				InputLockManager.RemoveControlLock(LockName);
		}

		public static void LockIfMouseOver(string LockName, Rect WindowRect, bool Lock=true)
		{
			Lock &= WindowRect.Contains(Event.current.mousePosition);
			LockEditor(LockName, Lock);
		}
		#endregion

		public static void Message(string msg, params object[] args)
		{ ScreenMessages.PostScreenMessage(string.Format(msg, args), 5, ScreenMessageStyle.UPPER_CENTER); }
	}

	//adapted from MechJeb
	public static class GLUtils
	{
		static Material _material;
		static Material material
		{
			get
			{
				if(_material == null) _material = new Material(Shader.Find("Particles/Additive"));
				return _material;
			}
		}

		static Camera GLBeginWorld(out float far)
		{
			var camera = MapView.MapIsEnabled? PlanetariumCamera.Camera : FlightCamera.fetch.mainCamera;
			far = camera.farClipPlane;
			camera.farClipPlane = far*100;
			GL.PushMatrix();
			material.SetPass(0);
			GL.LoadProjectionMatrix(camera.projectionMatrix);
			GL.modelview = camera.worldToCameraMatrix;
			return camera;
		}

		public static void GLTriangleMap(Vector3d[] worldVertices, Color c)
		{
			float far;
			var camera = GLBeginWorld(out far);
			GL.Begin(GL.TRIANGLES);
			GL.Color(c);
			GL.Vertex(worldVertices[0]);
			GL.Vertex(worldVertices[1]);
			GL.Vertex(worldVertices[2]);
			GL.End();
			GL.PopMatrix();
			camera.farClipPlane = far;
		}

		public static void GLTriangleMap(Vector3[] worldVertices, Color c)
		{
			float far;
			var camera = GLBeginWorld(out far);
			GL.Begin(GL.TRIANGLES);
			GL.Color(c);
			GL.Vertex(worldVertices[0]);
			GL.Vertex(worldVertices[1]);
			GL.Vertex(worldVertices[2]);
			GL.End();
			GL.PopMatrix();
			camera.farClipPlane = far;
		}

		public static void GLLine(Vector3 ori, Vector3 end, Color c)
		{
			float far;
			var camera = GLBeginWorld(out far);
			if(MapView.MapIsEnabled)
			{
				ori = ScaledSpace.LocalToScaledSpace(ori);
				end = ScaledSpace.LocalToScaledSpace(end);
			}
			GL.Begin(GL.LINES);
			GL.Color(c);
			GL.Vertex(ori);
			GL.Vertex(end);
			GL.End();
			GL.PopMatrix();
			camera.farClipPlane = far;
		}

		public static void GLVec(Vector3 ori, Vector3 vec, Color c)
		{ GLLine(ori, ori+vec, c); }

		//		edges[0] = new Vector3(min.x, min.y, min.z); //left-bottom-back
		//		edges[1] = new Vector3(min.x, min.y, max.z); //left-bottom-front
		//		edges[2] = new Vector3(min.x, max.y, min.z); //left-top-back
		//		edges[3] = new Vector3(min.x, max.y, max.z); //left-top-front
		//		edges[4] = new Vector3(max.x, min.y, min.z); //right-bottom-back
		//		edges[5] = new Vector3(max.x, min.y, max.z); //right-bottom-front
		//		edges[6] = new Vector3(max.x, max.y, min.z); //right-top-back
		//		edges[7] = new Vector3(max.x, max.y, max.z); //right-top-front

		public static void GLBounds(Bounds b, Transform T, Color col)
		{
			var c = Utils.BoundCorners(b);
			for(int i = 0; i < 8; i++) c[i] = T.TransformPoint(c[i]);
			GLLine(c[0], c[1], col);
			GLLine(c[1], c[5], col);
			GLLine(c[5], c[4], col);
			GLLine(c[4], c[0], col);

			GLLine(c[2], c[3], col);
			GLLine(c[3], c[7], col);
			GLLine(c[7], c[6], col);
			GLLine(c[6], c[2], col);

			GLLine(c[2], c[0], col);
			GLLine(c[3], c[1], col);
			GLLine(c[7], c[5], col);
			GLLine(c[6], c[4], col);
		}
	}

	public class FloatField : ConfigNodeObject
	{
		string svalue;
		[Persistent] public string format;
		[Persistent] public float  fvalue;
		[Persistent] public float  Min;
		[Persistent] public float  Max;
		[Persistent] public bool   Circle;

		public float Range { get { return Max-Min; } }

		public float Value 
		{ 
			get { return fvalue; } 
			set 
			{
				fvalue = Circle? Utils.Circle(value, Min, Max) : Utils.Clamp(value, Min, Max);
				svalue = fvalue.ToString(format);	
			}
		}

		public void Invert()
		{
			Min = -Max; Max = -Min;
			Value = -Value;
		}

		public override void Load(ConfigNode node)
		{
			base.Load(node);
			Value = fvalue;
		}

		public static implicit operator float(FloatField ff) { return ff.fvalue; }
		public override string ToString () { return fvalue.ToString(format); }

		public FloatField(string format = "F1", float min = float.MinValue, float max = float.MaxValue, bool circle = false)
		{
			this.format = format;
			Circle = circle;
			Min = min; Max = max;
			Value = fvalue;
		}

		public bool UpdateValue()
		{
			float val;
			if(float.TryParse(svalue, out val)) 
			{ Value = val; return true; }
			return false;
		}

		public bool Draw(string suffix = "", bool show_set_button = true, float increment = 0)
		{
			bool updated = false;
			if(!increment.Equals(0)) 
			{
				if(GUILayout.Button(string.Format("-{0}", increment), Styles.normal_button, GUILayout.ExpandWidth(false)))
				{ Value = fvalue-increment; updated = true; }
				if(GUILayout.Button(string.Format("+{0}", increment), Styles.normal_button, GUILayout.ExpandWidth(false)))
				{ Value = fvalue+increment; updated = true; }
			}
			svalue = GUILayout.TextField(svalue, svalue.Equals(fvalue.ToString(format))? Styles.green : Styles.white,
			                             GUILayout.ExpandWidth(true), GUILayout.MinWidth(70));
			if(!string.IsNullOrEmpty(suffix)) GUILayout.Label(suffix, Styles.label, GUILayout.ExpandWidth(false));
			updated |= 
				show_set_button && 
				GUILayout.Button("Set", Styles.normal_button, GUILayout.ExpandWidth(false)) && 
				UpdateValue();
			return updated;
		}
	}
}

