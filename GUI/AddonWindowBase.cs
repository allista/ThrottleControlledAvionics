//   AddonWindowBase.cs
//
//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

using System.Reflection;
using UnityEngine;
using KSP.IO;

namespace ThrottleControlledAvionics
{
	abstract public class AddonWindowBase<T> : MonoBehaviour where T : AddonWindowBase<T>
	{
		protected static T instance;
		protected static PluginConfiguration GUI_CFG = PluginConfiguration.CreateForType<T>();
		protected static TCAGlobals GLB { get { return TCAScenario.Globals; } }

		protected static Rect drag_handle = new Rect(0,0, 10000, 20);
		protected static int  width = 550, height = 100;
		protected static Rect MainWindow = new Rect();
		static protected bool showHUD = true;

		static protected string TCATitle;


		void onShowUI() { showHUD = true; }
		void onHideUI() { showHUD = false; }

		public virtual void Awake()
		{
			instance = (T)this;
			LoadConfig();
			GameEvents.onHideUI.Add(onHideUI);
			GameEvents.onShowUI.Add(onShowUI);
			TCATitle = "Throttle Controlled Avionics - " + 
				Assembly.GetCallingAssembly().GetName().Version;
		}

		public virtual void OnDestroy()
		{
			SaveConfig();
			GameEvents.onHideUI.Remove(onHideUI);
			GameEvents.onShowUI.Remove(onShowUI);
			instance = null;
		}

		//settings
		protected static string mangleName(string name) { return typeof(T).Name+"-"+name; }

		protected static void SetConfigValue(string key, object value)
		{ GUI_CFG.SetValue(mangleName(key), value); }

		protected static V GetConfigValue<V>(string key, V _default)
		{ return GUI_CFG.GetValue<V>(mangleName(key), _default); }

		virtual public void LoadConfig()
		{
			GUI_CFG.load();
			MainWindow = GetConfigValue<Rect>(Utils.PropertyName(new {MainWindow}), 
				new Rect(100, 50, width, height));
		}

		virtual public void SaveConfig(ConfigNode node = null)
		{
			SetConfigValue(Utils.PropertyName(new {MainWindow}), MainWindow);
			GUI_CFG.save();
		}

		#region Tooltips
		//adapted from blizzy's Toolbar
		protected static string tooltip = "";

		protected static void GetToolTip()
		{
			if(Event.current.type == EventType.repaint)
				tooltip = GUI.tooltip.Trim();
		}

		protected static void DrawToolTip(Rect window) 
		{
			if(tooltip.Length == 0) return;
			var mousePos = Utils.GetMousePosition(window);
			var size = Styles.tooltip.CalcSize(new GUIContent(tooltip));
			var rect = new Rect(mousePos.x, mousePos.y + 20, size.x, size.y);
			Rect orig = rect;
			rect = rect.clampToWindow(window);
			//clamping moved the tooltip up -> reposition above mouse cursor
			if(rect.y < orig.y) 
			{
				rect.y = mousePos.y - size.y - 5;
				rect = rect.clampToScreen();
			}
			//clamping moved the tooltip left -> reposition lefto of the mouse cursor
			if(rect.x < orig.x)
			{
				rect.x = mousePos.x - size.x - 5;
				rect = rect.clampToScreen();
			}
			GUI.Label(rect, tooltip, Styles.tooltip);
		}
		#endregion

		/// <summary>
		/// Draws the main window. Should be called last in child class overrides.
		/// </summary>
		/// <param name="windowID">Window ID</param>
		protected virtual void DrawMainWindow(int windowID)
		{ DrawWindow(windowID, MainWindow); }

		protected virtual void DrawWindow(int windowID, Rect window_rect)
		{
			GetToolTip();
			DrawToolTip(window_rect);
			GUI.DragWindow(drag_handle);
		}
	}
}

