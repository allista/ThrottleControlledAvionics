//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2017 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

using System;
using System.Linq;
using System.Collections.Generic;
using System.Reflection;
using UnityEngine;
using AT_Utils;

namespace ThrottleControlledAvionics
{
	[KSPAddon(KSPAddon.Startup.Flight, false)]
	public class TCAGui : AddonWindowBase<TCAGui>
	{
		Vessel vessel;
        public ModuleTCA ActiveVesselTCA { get; private set; }
		public ModuleTCA TCA { get; private set; }
		public VesselWrapper VSL { get { return TCA.VSL; } }
		internal static Globals GLB { get { return Globals.Instance; } }
		public VesselConfig CFG { get { return TCA.CFG; } }
        public bool HaveRemoteControl { get; private set; }
        public bool RemoteControl { get; private set; }

		#region GUI Parameters
		public const int ControlsWidth = 450, ControlsHeight = 180, LineHeight = 35;
		public const int ControlsHeightHalf = ControlsHeight/2;

		[ConfigOption] 
		public KeyCode TCA_Key = KeyCode.Y;

        [ConfigOption]
        public bool Collapsed;

        [ConfigOption]
        Rect collapsed_rect = new Rect();

        [ConfigOption]
        public bool ShowOnHover = true;

        bool draw_main_window;

		public static string StatusMessage;
		public static DateTime StatusEndTime;

		public static Blinker EnabledBlinker = new Blinker(0.5);
		#endregion

		#pragma warning disable 169
		#region modules
		TimeWarpControl WRP;
		AltitudeControl ALT;
		VerticalSpeedControl VSC;
		ThrottleControl THR;
		SquadControl SQD;
		#endregion

		#region ControlWindows
		VFlightWindow VFlight_Window;
		ManeuverWindow Maneuver_Window;
		AttitudeControlWindow Attitude_Window;
		List<ControlWindow> AllWindows = new List<ControlWindow>();
		#endregion

		#region ControlTabs
		[TabInfo("Navigation", 1, Icon = "ThrottleControlledAvionics/Icons/NavigationTab.png")]
		public NavigationTab NAV;

		[TabInfo("Orbital Autopilots", 2, Icon = "ThrottleControlledAvionics/Icons/OrbitalTab.png")]
		public OrbitalTab ORB;

		[TabInfo("Engines Control", 3, Icon = "ThrottleControlledAvionics/Icons/EnginesTab.png")]
		public EnginesTab ENG;

		[TabInfo("Advanced Settings", 4, Icon = "ThrottleControlledAvionics/Icons/AdvancedTab.png")]
		public AdvancedTab ADV;

		[TabInfo("Macros", 5, Icon = "ThrottleControlledAvionics/Icons/MacrosTab.png")]
		public MacrosTab MCR;

		public ControlTab ActiveTab = null;
		List<ControlTab> AllTabs = new List<ControlTab>();
		List<FieldInfo> AllTabFields = new List<FieldInfo>();
		Vector2 tabs_scroll = Vector2.zero;
		#endregion

        //other subwindows
        public TCAPartsEditor ModulesGraph;
		#pragma warning restore 169

		#region Initialization
		void save_config(ConfigNode node) { SaveConfig(); }

		public override void Awake()
		{
			base.Awake();
			AllTabFields = ControlTab.GetTabFields(GetType());
			AllWindows = subwindows.Where(sw => sw is ControlWindow).Cast<ControlWindow>().ToList();
			GameEvents.onGameStateSave.Add(save_config);
			GameEvents.onVesselChange.Add(onVesselChange);
            GameEvents.onVesselDestroy.Add(onVesselDestroy);
			NavigationTab.OnAwake();
		}

		public override void OnDestroy()
		{
			base.OnDestroy();
			clear_fields();
			TCAToolbarManager.AttachTCA(null);
			GameEvents.onGameStateSave.Remove(save_config);
			GameEvents.onVesselChange.Remove(onVesselChange);
            GameEvents.onVesselDestroy.Remove(onVesselDestroy);
		}

        void onVesselDestroy(Vessel vsl)
        {
            if(vsl == vessel && vsl != FlightGlobals.ActiveVessel)
                onVesselChange(FlightGlobals.ActiveVessel);
        }

		void onVesselChange(Vessel vsl)
		{
            vessel = vsl;
            StatusMessage = "";
            StatusEndTime = DateTime.MinValue;
            if(vsl != null && vsl.parts != null) 
                StartCoroutine(init_on_load());
            else clear_fields();
		}

        void switch_vessel(Func<Vessel,Vessel> get_next)
        {
            if(ActiveVesselTCA == null) return;
            var next_vessel = vessel;
            ModuleTCA next = null;
            while(next == null || 
                  !SquadControl.IsCommReachable(ActiveVesselTCA, next))
            {
                next_vessel = get_next(next_vessel);
                if(next_vessel.loaded)
                    next = ModuleTCA.AvailableTCA(next_vessel);
            }
            if(next != TCA)
            {
                SquadControl.UnpackVessel(ActiveVesselTCA.vessel, next_vessel);
                onVesselChange(next_vessel);
            }
        }

		public override void Show(bool show)
		{
			if(TCA == null || CFG == null) return;
			CFG.GUIVisible = show;
			base.Show(show);
		}

		public static void Reinitialize(ModuleTCA tca) 
		{ 
			if(Instance == null || tca.vessel != Instance.vessel) return;
			Instance.StartCoroutine(Instance.init_on_load()); 
		}

		IEnumerator<YieldInstruction> init_on_load()
		{
			do {
				yield return null;
				if(vessel == null) yield break;
			} while(!vessel.loaded || TimeWarp.CurrentRateIndex == 0 && vessel.packed);	
			init();
		}

		void create_fields()
		{
			TCA.InitModuleFields(this);
			AllWindows.ForEach(w => w.Init(TCA));
			foreach(var fi in AllTabFields)
			{
				var tab = TCA.CreateComponent(fi.FieldType) as ControlTab;
				if(tab == null) continue;
				tab.Init();
				if(!tab.Valid) continue;
				var info = fi.GetCustomAttributes(typeof(TabInfo), false).FirstOrDefault() as TabInfo;
				if(info != null) tab.SetupTab(info);
				fi.SetValue(this, tab);
				AllTabs.Add(tab);
			}
			AllTabs.Sort((a,b) => a.Index.CompareTo(b.Index));
			if(CFG != null && CFG.ActiveTab < AllTabs.Count-1)
				ActiveTab = AllTabs[CFG.ActiveTab];
			else ActiveTab = AllTabs[0];
		}

		void clear_fields()
		{
            ModulesGraph.Show(false);
			AllTabs.ForEach(t => t.Reset());
			AllWindows.ForEach(w => w.Reset());
			AllTabFields.ForEach(fi => fi.SetValue(this, null));
			ModuleTCA.ResetModuleFields(this);
			AllTabs.Clear();
            ActiveVesselTCA = null;
            TCA = null;
		}

		bool init()
		{
            clear_fields();
            ClearStatus();
			TCAToolbarManager.AttachTCA(null);
			TCA = ModuleTCA.AvailableTCA(vessel);
			if(TCA == null || CFG == null) return false;
            ActiveVesselTCA = ModuleTCA.AvailableTCA(FlightGlobals.ActiveVessel);
            HaveRemoteControl = ActiveVesselTCA != null && ActiveVesselTCA.GetModule<SquadControl>() != null;
            RemoteControl = ActiveVesselTCA != TCA;
            ShowInstance(CFG.GUIVisible);
            ModulesGraph.SetCFG(CFG);
			TCAToolbarManager.AttachTCA(TCA);
			create_fields();
			if(ADV != null)
				ADV.UpdateNamedConfigs();
			return true;
		}
		#endregion

		#region Status
		public static void ClearStatus() { TCAGui.StatusMessage = ""; StatusEndTime = DateTime.MinValue; }

		public static void Status(double seconds, string msg, params object[] args)
		{
			TCAGui.StatusMessage = string.Format(msg, args);
			TCAGui.StatusEndTime = seconds > 0? DateTime.Now.AddSeconds(seconds) : DateTime.MinValue;
		}

		public static void Status(string msg, params object[] args) { Status(-1, msg, args); }

		public static void Status(double seconds, string color, string msg, params object[] args)
		{ Status(seconds, string.Format("<color={0}>{1}</color>", color, msg), args); }

		public static void Status(string color, string msg, params object[] args) 
		{ Status(-1, color, msg, args); }

		void DrawStatusMessage()
		{
			if(!string.IsNullOrEmpty(StatusMessage))
			{ 
				if(GUILayout.Button(new GUIContent(StatusMessage, "Click to dismiss"), 
				                       Styles.boxed_label, GUILayout.ExpandWidth(true)) ||
				      StatusEndTime > DateTime.MinValue && DateTime.Now > StatusEndTime)
					StatusMessage = "";
			}
		}

		string StatusString()
		{
			if(TCA.IsStateSet(TCAState.Enabled))
			{
				if(TCA.IsStateSet(TCAState.ObstacleAhead))
                    return "<color=red>Obstacle On Course</color>";
				else if(TCA.IsStateSet(TCAState.GroundCollision))
                    return "<color=red>Ground Collision Possible</color>";
				else if(TCA.IsStateSet(TCAState.LoosingAltitude))
                    return "<color=red>Loosing Altitude</color>";
				else if(!VSL.Controls.HaveControlAuthority)
                    return "<color=red>Low Control Authority</color>";
                else if(TCA.IsStateSet(TCAState.Unoptimized))
                    return "<color=yellow>Engines Unoptimized</color>";
				else if(TCA.IsStateSet(TCAState.Ascending))
                    return "<color=yellow>Ascending</color>";
				else if(TCA.IsStateSet(TCAState.VTOLAssist))
                    return "<color=yellow>VTOL Assist On</color>";
				else if(TCA.IsStateSet(TCAState.StabilizeFlight))
                    return "<color=yellow>Stabilizing Flight</color>";
				else if(TCA.IsStateSet(TCAState.AltitudeControl))
                    return "<color=lime>Altitude Control</color>";
				else if(TCA.IsStateSet(TCAState.VerticalSpeedControl))
                    return "<color=lime>Vertical Speed Control</color>";
				else if(TCA.State == TCAState.Nominal)
                    return "<color=lime>Systems Nominal</color>";
				else if(TCA.State == TCAState.NoActiveEngines)
                    return "<color=yellow>No Active Engines</color>";
				else if(TCA.State == TCAState.NoEC)
                    return "<color=red>No Electric Charge</color>";
				else //this should never happen
                    return "<color=magenta>Unknown State</color>";
			}
            return "<color=grey>Disabled</color>";
		}

        void StatusLabel()
        {
            GUILayout.Label(StatusString(), Styles.boxed_label, GUILayout.ExpandWidth(false));
        }
		#endregion

        void update_collapsed_rect()
        {
            if(Collapsed)
                collapsed_rect = new Rect(WindowPos.x, WindowPos.y, 
                                          ShowOnHover? WindowPos.width : 40, 23);
        }

        static GUIContent collapse_button = new GUIContent("▲", "Collapse Main Window");
        static GUIContent uncollapse_button = new GUIContent("▼", "Restore Main Window");
        static GUIContent prev_vessel_button = new GUIContent("◀", "Switch to previous vessel");
        static GUIContent next_vessel_button = new GUIContent("▶", "Switch to next vessel");
        static GUIContent active_vessel_button = new GUIContent("◇", "Back to active vessel");
        static GUIContent switch_vessel_button = new GUIContent("◆", "Switch to current vessel");
		void DrawMainWindow(int windowID)
		{
			//help button
            if(GUI.Button(new Rect(0, 0f, 20f, 18f), 
                          Collapsed? uncollapse_button : collapse_button, Styles.label)) 
            {
                Collapsed = !Collapsed;
                update_collapsed_rect();
            }
			if(GUI.Button(new Rect(WindowPos.width - 20f, 0f, 20f, 18f), 
			              new GUIContent("?", "Help"), Styles.label)) 
                TCAManual.ToggleInstance();
            //vessel switching
            if(HaveRemoteControl)
            {
                if(GUI.Button(new Rect(22, 0f, 20f, 18f), prev_vessel_button, Styles.label)) 
                    switch_vessel(FlightGlobals.Vessels.Next);
                if(RemoteControl && 
                   GUI.Button(new Rect(44, 0f, 20f, 18f), active_vessel_button, Styles.label)) 
                    onVesselChange(ActiveVesselTCA.vessel);
                if(RemoteControl &&
                   GUI.Button(new Rect(WindowPos.width - 64f, 0f, 20f, 18f), switch_vessel_button, Styles.label))
                    FlightGlobals.SetActiveVessel(vessel);
                if(GUI.Button(new Rect(WindowPos.width - 42f, 0f, 20f, 18f), next_vessel_button, Styles.label))
                    switch_vessel(FlightGlobals.Vessels.Prev);
            }
			if(TCA.IsControllable)
			{
				GUILayout.BeginVertical();
				GUILayout.BeginHorizontal();
				//tca toggle
				var enabled_style = Styles.inactive_button;
				if(CFG.Enabled) enabled_style = Styles.enabled_button;
				else if(!VSL.LandedOrSplashed) 
				{
					if(EnabledBlinker.On) enabled_style = Styles.danger_button;
					Status(0.1, "red", "<b>TCA is disabled</b>");
				}
				if(GUILayout.Button("Enabled", enabled_style, GUILayout.Width(70)))
					TCA.ToggleTCA();
				#if DEBUG
				if(GUILayout.Button("ReGlobals", Styles.active_button, GUILayout.ExpandWidth(false))) 
				{
					Globals.Load();
					Styles.ConfigureButtons();
					TCA.OnReloadGlobals();
				}
				#endif
				//squad mode switch
				if(SQD != null) SQD.Draw();
				GUILayout.FlexibleSpace();
                StatusLabel();
				GUILayout.EndHorizontal();
				GUILayout.BeginHorizontal();
				GUILayout.BeginVertical(Styles.white, GUILayout.MinHeight(ControlsHeight), GUILayout.ExpandWidth(false), GUILayout.ExpandHeight(true));
				if(ActiveTab != null) ActiveTab.Draw();
				GUILayout.EndVertical();
				tabs_scroll = GUILayout.BeginScrollView(tabs_scroll, Styles.white, GUILayout.ExpandHeight(true), GUILayout.Width(55));
				for(int i = 0, AllTabsCount = AllTabs.Count; i < AllTabsCount; i++)
				{
					var t = AllTabs[i];
					if(t.DrawTabButton(t == ActiveTab))
					{
						ActiveTab = t;
						CFG.ActiveTab = i;
					}
				}
				GUILayout.EndScrollView();
				GUILayout.EndHorizontal();
				DrawStatusMessage();
				GUILayout.EndVertical();
			}
			else 
			{
				GUILayout.BeginVertical();
				GUILayout.BeginHorizontal();
				VSL.Info.Draw();
                GUILayout.FlexibleSpace();
                StatusLabel();
				GUILayout.EndHorizontal();
				GUILayout.Label("Vessel is Uncontrollable", Styles.label, GUILayout.ExpandWidth(true), GUILayout.ExpandHeight(true));
				DrawStatusMessage();
				GUILayout.EndVertical();
			}
			TooltipsAndDragWindow();
		}

		protected override bool can_draw()
		{ return TCA != null && VSL != null && vessel != null && CFG.GUIVisible && AllTabs.Count > 0; }

		#if DEBUG
		static Rect debug_rect = new Rect(Screen.width*0.75f, 0, 200, 25).clampToScreen();
		#endif
		protected override void draw_gui()
		{
            //handle collapsed state
            if(Collapsed)
            {
                if(ShowOnHover)
                {
                    if(Event.current.type == EventType.Repaint) 
                        draw_main_window = WindowPos.Contains(Event.current.mousePosition);
                    if(!draw_main_window)
                    {
                        UnlockControls();
                        var prefix = CFG.Enabled? 
                            "<b><color=lime>TCA: </color></b>" : 
                            (VSL.LandedOrSplashed? "<b>TCA: </b>" : "<b><color=red>TCA: </color></b>");
                        GUI.Label(collapsed_rect, prefix+StatusString(), Styles.boxed_label);
                    }
                }
                else
                {
                    UnlockControls();
                    GUI.Label(collapsed_rect, new GUIContent("TCA", "Push to show Main Window"), 
                          CFG.Enabled? Styles.green : (VSL.LandedOrSplashed? Styles.white : Styles.red));
                    if(Input.GetMouseButton(0) && collapsed_rect.Contains(Event.current.mousePosition))
                        Collapsed = false;
                    TooltipManager.GetTooltip();
                }
            }
            else draw_main_window = true;
            //draw main window if allowed
            if(draw_main_window)
            {
                LockControls();
    			WindowPos = 
    				GUILayout.Window(TCA.GetInstanceID(), 
    				                 WindowPos, 
    				                 DrawMainWindow, 
                                     RemoteControl? "RC: "+vessel.vesselName : vessel.vesselName,
    				                 GUILayout.Width(ControlsWidth),
    				                 GUILayout.Height(50)).clampToScreen();
                update_collapsed_rect();
            }
            //draw waypoints and all subwindows
            if(RemoteControl && Event.current.type == EventType.Repaint)
                Markers.DrawWorldMarker(TCA.vessel.transform.position, Color.green, 
                                        "Remotely Controlled Vessel", NavigationTab.PathNodeMarker, 8);
			if(NAV != null) NAV.DrawWaypoints();
			AllWindows.ForEach(w => w.Draw());
            ModulesGraph.Draw();

			#if DEBUG
			GUI.Label(debug_rect, 
			          string.Format("[{0}] {1:HH:mm:ss.fff}", 
			                        TCA != null && vessel != null? vessel.situation.ToString() : "", 
			                        DateTime.Now), 
			          Styles.boxed_label);
			#endif
		}

		public void Update()
		{
			if(TCA == null) return;
			if(!TCA.Available && !init()) return;
			if(!TCA.IsControllable) return;
			if(ADV != null && ADV.SelectingKey) ADV.Update();
			else if(!FlightDriver.Pause)
			{
				if(Input.GetKeyDown(TCA_Key)) TCA.ToggleTCA();
				if(CFG.Enabled)
				{
					if(CFG.BlockThrottle && THR != null)
					{
						if(CFG.VF[VFlight.AltitudeControl]) 
						{ if(ALT != null) ALT.ProcessKeys(); }
						else if(VSC != null) VSC.ProcessKeys();
					}
                    if(WRP != null) WRP.ProcessKeys();
				}
			}
		}

        public void OnRenderObject()
        {
            AllTabs.ForEach(t => t.OnRenderObject());
        }


        #if DEBUG
        static string DebugMessage;
        #pragma warning disable 169
        DebugMessageWindow debug_window;
        #pragma warning restore 169

        public static void AddDebugMessage(string msg, params object[] args)
        { DebugMessage += Utils.Format(msg, args)+"\n"; }

        public static void ClearDebugMessage()
        { DebugMessage = ""; }

        static Vector2 eInfoScroll;
        void EnginesInfo()
        {
            GUILayout.BeginVertical();
            GUILayout.BeginHorizontal();
            GUILayout.Label(string.Format("Steering: {0}", VSL.Controls.Steering), GUILayout.ExpandWidth(false));
            GUILayout.Label(string.Format("Angular Accel Error: {0:F3}rad/s2", TCA.ENG.TorqueError), GUILayout.ExpandWidth(false));
            GUILayout.Label(string.Format("Vertical Speed Factor: {0:P1}", VSL.OnPlanetParams.VSF), GUILayout.ExpandWidth(false));
            GUILayout.EndHorizontal();
            eInfoScroll = GUILayout.BeginScrollView(eInfoScroll, GUILayout.Height(ControlsHeight*2));
            GUILayout.BeginVertical();
            foreach(var e in VSL.Engines.Active)
            {
                if(!e.Valid(VSL)) continue;
                GUILayout.BeginHorizontal();
                GUILayout.Label(e.name + "\n" +
                                string.Format(
                                    "Torque: {0}\n" +
                                    "Attitude Modifier: {1:P1}\n" +
                                    "Thrust Limit:      {2:F1}%",
                                    e.currentTorque,
                                    e.limit, e.thrustLimit*100));
                GUILayout.EndHorizontal();
            }
            GUILayout.EndVertical();
            GUILayout.EndScrollView();
            GUILayout.EndVertical();
        }

        void DebugInfo()
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label(string.Format("vV: {0:0.0}m/s", VSL.VerticalSpeed.Absolute), GUILayout.Width(100));
            GUILayout.Label(string.Format("A: {0:0.0}m/s2", VSL.VerticalSpeed.Derivative), GUILayout.Width(80));
            GUILayout.Label(string.Format("ApA: {0:0.0}m", VSL.orbit.ApA), GUILayout.Width(120));
            GUILayout.Label(string.Format("hV: {0:0.0}m/s", VSL.HorizontalSpeed.Absolute), GUILayout.Width(100));
            GUILayout.Label(string.Format("Rho: {0:0.000}ASL", VSL.Body.atmosphere? VSL.vessel.atmDensity/VSL.Body.atmDensityASL : 0), GUILayout.Width(100));
            GUILayout.Label(string.Format("aV2: {0:0.0E0}", VSL.vessel.angularVelocity.sqrMagnitude), GUILayout.Width(100));
            GUILayout.Label(string.Format("inc: {0:0.000}", VSL.orbit.inclination), GUILayout.Width(100));
            GUILayout.EndHorizontal();
            GUILayout.BeginHorizontal();
            GUILayout.Label(string.Format("VSP: {0:0.0m/s}", CFG.VerticalCutoff), GUILayout.Width(100));
            GUILayout.Label(string.Format("TWR: {0:0.0}", VSL.OnPlanetParams.DTWR), GUILayout.Width(80));
            if(VSL.Altitude.Ahead.Equals(float.MinValue)) GUILayout.Label("Obst: N/A", GUILayout.Width(120));
            else GUILayout.Label(string.Format("Obst: {0:0.0}m", VSL.Altitude.Ahead), GUILayout.Width(120));
            GUILayout.Label(string.Format("Orb: {0:0.0}m/s", Math.Sqrt(VSL.Physics.StG*VSL.Physics.Radial.magnitude)), GUILayout.Width(100));
            GUILayout.Label(string.Format("dP: {0:0.000}kPa", VSL.vessel.dynamicPressurekPa), GUILayout.Width(100));
            GUILayout.Label(string.Format("Thr: {0:P1}", VSL.vessel.ctrlState.mainThrottle), GUILayout.Width(100));
            GUILayout.Label(string.Format("ecc: {0:0.000}", VSL.orbit.eccentricity), GUILayout.Width(100));
            GUILayout.EndHorizontal();
        }

        class DebugMessageWindow : ControlWindow
        {
            public DebugMessageWindow()
            {
                width = 600;
                height = 25;
            }

            protected override void DrawContent()
            {
    //          DebugInfo();
    //          EnginesInfo();
                if(!string.IsNullOrEmpty(DebugMessage))
                    GUILayout.Label(DebugMessage, Styles.boxed_label, GUILayout.Width(width));
            }
        }
        #endif
    }
}
