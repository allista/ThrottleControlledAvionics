//   ControlTab.cs
//
//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2017 Allis Tauri

using System;
using System.IO;
using System.Linq;
using System.Collections.Generic;
using System.Reflection;
using UnityEngine;
using AT_Utils;

namespace ThrottleControlledAvionics
{
	/// <summary>
	/// Tab info defines the display options of a ControlTab
	/// </summary>
	public class TabInfo : Attribute
	{
		public int Index = -1;
		public string Icon = "";
		public string Title = "";

		public TabInfo(string title, int index)
		{
			Title = title;
			Index = index;
		}

		public GUIContent GetButtonContent()
		{
			Texture2D Image = null;
			if(!string.IsNullOrEmpty(Icon))
			{
				//using direct texture loading instead of GameDatabase,
				//because the latter resamples textures with huge qualit loss
				//Image = GameDatabase.Instance.GetTexture(Icon, false);
				var path = CustomConfig.GameDataFolder(Icon.Replace('/', Path.DirectorySeparatorChar));
				if(File.Exists(path))
				{
					Image = new Texture2D(32, 32, TextureFormat.RGBA32, false);
					Image.LoadImage(File.ReadAllBytes(path));
				}
				else Utils.Log("No such file: {}", path);
			}
			return Image != null? new GUIContent(Image, Title) :
				new GUIContent(Title);
		}

		public override string ToString()
		{
			return Utils.Format("{}: index {}, icon {}", Title, Index, Icon);
		}
	}

	/// <summary>
	/// Internal module is used to flag TCAModules that are required for a particular ControlTab, 
	/// but do not provide independent functionality. Such modules are not considered when the 
	/// ControlTab.Valid value is calculated.
	/// </summary>
	public class InternalModule : Attribute {}

	/// <summary>
	/// ControlTab is an assembly of controls of several related TCAModules
	/// with additional framework for integration the tab into TCAGui.
	/// </summary>
	public abstract class ControlTab : DrawableComponent
	{
		GUIContent TabButtonContent = new GUIContent();
		public int Index = -1;

		protected List<TCAModule> ThisModules = new List<TCAModule>();

		public bool Enabled { get; protected set; }

        public override bool Valid { get { return base.Valid && ThisModules.Count > 0; } }

		protected ControlTab(ModuleTCA tca) : base(tca) {}

		protected void follow_me() { if(SQD != null) SQD.FollowMe(); }

		public void SetupTab(TabInfo info)
		{
			Index = info.Index;
			TabButtonContent = info.GetButtonContent();
		}

		public virtual void Update() {}

		public virtual void Init() 
		{
			InitModuleFields();
			ThisModules = GetType().GetFields(BindingFlags.NonPublic|BindingFlags.Public|BindingFlags.Instance|BindingFlags.DeclaredOnly)
				.Where(fi => fi.FieldType.IsSubclassOf(typeof(TCAModule)) && fi.GetCustomAttributes(typeof(InternalModule), true).Length == 0)
				.Select(fi => fi.GetValue(this) as TCAModule)
				.Where(m => m != null)
				.ToList();
		}

		public virtual void Reset() 
		{
			ThisModules.Clear();
		}

		public virtual bool DrawTabButton(bool active)
		{
			if(!Valid) return false;
			if(active) 
			{
				GUILayout.Label(TabButtonContent, Styles.white, GUILayout.ExpandWidth(false));
				return false;
			}
			return GUILayout.Button(TabButtonContent, Styles.active_button, GUILayout.ExpandWidth(false));
		}

		public static List<FieldInfo> GetTabFields(Type T)
		{
			return T.GetFields(BindingFlags.Instance|BindingFlags.Public|BindingFlags.DeclaredOnly)
				.Where(fi => fi.FieldType.IsSubclassOf(typeof(ControlTab))).ToList();
		}
	}
}
