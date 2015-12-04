//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

using UnityEngine;

namespace ThrottleControlledAvionics
{
	public static class Styles 
	{
		//This code is based on Styles class from Extraplanetary Launchpad plugin.
		public static GUISkin skin;

		public static GUIStyle normal_button;
		public static GUIStyle grey_button;
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
		public static GUIStyle cyan;
		public static GUIStyle label;
		public static GUIStyle rich_label;
		public static GUIStyle tooltip;
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

		static GUIStyle OtherColor(this GUIStyle style, Color normal)
		{
			var s = new GUIStyle(style);
			s.normal.textColor = s.focused.textColor = normal;
			return s;
		}

		static GUIStyle OtherColor(this GUIStyle style, Color normal, Color hover)
		{
			var s = style.OtherColor(normal);
			s.hover.textColor = s.active.textColor = hover;
			s.onNormal.textColor = s.onFocused.textColor = s.onHover.textColor = s.onActive.textColor = hover;
			return s;
		}

		public static void InitGUI()
		{
			if (initialized) return;
			initialized = true;

			var b_texture = new Texture2D(1, 1);
			b_texture.SetPixel(0, 0, new Color(0.05f, 0.05f, 0.05f, 1f));
			b_texture.Apply();

			//buttons
			normal_button = GUI.skin.button.OtherColor(Color.white, Color.yellow);
			normal_button.padding = new RectOffset (4, 4, 4, 4);

			grey_button        = normal_button.OtherColor(Color.grey, Color.white);
			red_button         = normal_button.OtherColor(Color.red, Color.white);
			dark_red_button    = red_button.OtherColor(new Color(0.6f, 0, 0, 1));
			green_button       = red_button.OtherColor(Color.green);
			dark_green_button  = red_button.OtherColor(new Color(0, 0.6f, 0, 1));
			yellow_button      = red_button.OtherColor(Color.yellow);
			dark_yellow_button = red_button.OtherColor(new Color(0.6f, 0.6f, 0, 1));
			cyan_button        = red_button.OtherColor(Color.cyan);
			magenta_button     = red_button.OtherColor(Color.magenta);

			//boxes
			white = GUI.skin.box.OtherColor(Color.white);
			white.padding = new RectOffset (4, 4, 4, 4);

			grey   = white.OtherColor(Color.grey);
			red    = white.OtherColor(Color.red);
			yellow = white.OtherColor(Color.yellow);
			green  = white.OtherColor(Color.green);
			blue   = white.OtherColor(new Color(0.6f, 0.6f, 1f, 1f));
			cyan   = white.OtherColor(Color.cyan);

			//tooltip
			tooltip  = white.OtherColor(Color.white);
			tooltip.alignment = TextAnchor.MiddleCenter;
			tooltip.normal.background = tooltip.onNormal.background = tooltip.hover.background = tooltip.onHover.background = b_texture;

			//lable
			label  = GUI.skin.label.OtherColor(Color.white);
			label.alignment = TextAnchor.MiddleCenter;

			rich_label = GUI.skin.label.OtherColor(Color.white);
			rich_label.richText = true;

			//slider
			slider = new GUIStyle(GUI.skin.horizontalSlider);
			slider.margin = new RectOffset (0, 0, 0, 0);

			slider_text = new GUIStyle(GUI.skin.label);
			slider_text.alignment = TextAnchor.MiddleCenter;
			slider_text.margin = new RectOffset (0, 0, 0, 0);

			//list box
			list_item = new GUIStyle(GUI.skin.box);
			list_item.normal.background = list_item.onNormal.background = list_item.hover.background = list_item.onHover.background = b_texture;
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

