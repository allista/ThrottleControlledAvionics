//   Styles.cs
//
//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

using UnityEngine;

namespace ThrottleControlledAvionics
{
	public static class Styles 
	{
		//This code is based on Styles class from Extraplanetary Launchpad plugin.
		public static GUISkin skin;

		public static GUIStyle normal_button;
		public static GUIStyle red_button;
		public static GUIStyle dark_red_button;
		public static GUIStyle green_button;
		public static GUIStyle dark_green_button;
		public static GUIStyle yellow_button;
		public static GUIStyle dark_yellow_button;
		public static GUIStyle cyan_button;
		public static GUIStyle magenta_button;
		public static GUIStyle white;
		public static GUIStyle grey;
		public static GUIStyle red;
		public static GUIStyle yellow;
		public static GUIStyle green;
		public static GUIStyle blue;
		public static GUIStyle label;
		public static GUIStyle slider;
		public static GUIStyle slider_text;

		public static GUIStyle list_item;
		public static GUIStyle list_box;

		static bool initialized;

		public static void InitSkin()
		{
			if(skin != null) return;
			GUI.skin = null;
			skin = (GUISkin)UnityEngine.Object.Instantiate(GUI.skin);
		}

		public static void InitGUI()
		{
			if (initialized) return;
			initialized = true;

			normal_button = new GUIStyle(GUI.skin.button);
			normal_button.normal.textColor = normal_button.focused.textColor = Color.white;
			normal_button.hover.textColor = normal_button.active.textColor = Color.yellow;
			normal_button.onNormal.textColor = normal_button.onFocused.textColor = normal_button.onHover.textColor = normal_button.onActive.textColor = Color.yellow;
			normal_button.padding = new RectOffset (4, 4, 4, 4);

			red_button = new GUIStyle(normal_button);
			red_button.normal.textColor = red_button.focused.textColor = Color.red;

			dark_red_button = new GUIStyle(normal_button);
			dark_red_button.normal.textColor = dark_red_button.focused.textColor = new Color(0.6f, 0, 0, 1);

			green_button = new GUIStyle(normal_button);
			green_button.normal.textColor = green_button.focused.textColor = Color.green;

			dark_green_button = new GUIStyle(normal_button);
			dark_green_button.normal.textColor = dark_green_button.focused.textColor = new Color(0, 0.6f, 0, 1);

			yellow_button = new GUIStyle(normal_button);
			yellow_button.normal.textColor = yellow_button.focused.textColor = Color.yellow;
			yellow_button.hover.textColor = yellow_button.active.textColor = Color.green;
			yellow_button.onNormal.textColor = yellow_button.onFocused.textColor = yellow_button.onHover.textColor = yellow_button.onActive.textColor = Color.green;

			dark_yellow_button = new GUIStyle(yellow_button);
			dark_yellow_button.normal.textColor = dark_yellow_button.focused.textColor = new Color(0.6f, 0.6f, 0, 1);

			cyan_button = new GUIStyle (normal_button);
			cyan_button.normal.textColor = cyan_button.focused.textColor = Color.cyan;

			magenta_button = new GUIStyle (normal_button);
			magenta_button.normal.textColor = magenta_button.focused.textColor = Color.magenta;

			white = new GUIStyle(GUI.skin.box);
			white.padding = new RectOffset (4, 4, 4, 4);
			white.normal.textColor = white.focused.textColor = Color.white;

			grey = new GUIStyle(white);
			grey.normal.textColor = grey.focused.textColor = Color.grey;

			red = new GUIStyle(white);
			red.normal.textColor = red.focused.textColor = Color.red;

			yellow = new GUIStyle(white);
			yellow.normal.textColor = yellow.focused.textColor = Color.yellow;

			green = new GUIStyle(white);
			green.normal.textColor = green.focused.textColor = Color.green;

			blue = new GUIStyle(white);
			blue.normal.textColor = blue.focused.textColor = new Color(0.6f, 0.6f, 1f, 1f);

			label = new GUIStyle (GUI.skin.label);
			label.normal.textColor = label.focused.textColor = Color.white;
			label.alignment = TextAnchor.MiddleCenter;

			slider = new GUIStyle (GUI.skin.horizontalSlider);
			slider.margin = new RectOffset (0, 0, 0, 0);

			slider_text = new GUIStyle (GUI.skin.label);
			slider_text.alignment = TextAnchor.MiddleCenter;
			slider_text.margin = new RectOffset (0, 0, 0, 0);

			list_item = new GUIStyle(GUI.skin.box);
			var texInit = new Texture2D(1, 1);
			texInit.SetPixel(0, 0, new Color(0.05f, 0.05f, 0.05f, 1f));
			texInit.Apply();
			list_item.normal.background = list_item.onNormal.background = list_item.hover.background = list_item.onHover.background = texInit;
			list_item.normal.textColor = list_item.focused.textColor = Color.white;
			list_item.hover.textColor = list_item.active.textColor = Color.yellow;
			list_item.onNormal.textColor = list_item.onFocused.textColor = list_item.onHover.textColor = list_item.onActive.textColor = Color.yellow;
			list_item.padding = new RectOffset(4, 4, 4, 4);

			list_box = new GUIStyle(GUI.skin.button);
			list_box.normal.textColor = list_box.focused.textColor = Color.yellow;
			list_box.hover.textColor = list_box.active.textColor = Color.green;
			list_box.onNormal.textColor = list_box.onFocused.textColor = list_box.onHover.textColor = list_box.onActive.textColor = Color.green;
			list_box.padding = new RectOffset (4, 4, 4, 4);
		}

		public static void Init()
		{
			Styles.InitSkin();
			GUI.skin = Styles.skin;
			Styles.InitGUI();
		}

		public static GUIStyle fracStyle(float frac)
		{
			if(frac < 0.1) return Styles.red;
			if(frac < 0.5) return Styles.yellow;
			if(frac < 0.8) return Styles.white;
			return Styles.green;
		}
	}
}

