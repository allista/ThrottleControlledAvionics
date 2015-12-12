/* The GUI for ThrottleControlledAvionics.
 * Authors: Quinten Feys, Willem van Vliet, Allis Tauri
 * License: BY: Attribution-ShareAlike 3.0 Unported (CC BY-SA 3.0): http://creativecommons.org/licenses/by-sa/3.0/
 */

using System;
using System.Linq;
using System.Collections.Generic;
using UnityEngine;

namespace ThrottleControlledAvionics
{
	public partial class ThrottleControlledAvionics
	{
		#region GUI Parameters
		const string LockName = "TCAGui";

		static bool advOptions;
		//named configs
		static NamedConfig selected_config;
		static string config_name = string.Empty;
		static readonly DropDownList namedConfigsListBox = new DropDownList();
		//dimensions
		public const int controlsWidth = 600, controlsHeight = 100, lineHeight = 35;
		static Vector2 waypointsScroll;
		//keybindings
		public static KeyCode TCA_Key = KeyCode.Y;
		static bool selecting_key;
		//map view
		static bool selecting_target;
		static readonly ActionDamper AddTargetDamper = new ActionDamper();
		const string WPM_ICON = "ThrottleControlledAvionics/Icons/waypoint";
		const string PN_ICON  = "ThrottleControlledAvionics/Icons/path-node";
		const float  IconSize = 16;
		static Texture2D WayPointMarker, PathNodeMarker;
		//altitude edit
		static string s_altitude = null;
		static float altitude;
		#if DEBUG
		public static string DebugMessage;
		#endif
		#endregion

		#region Configs Selector
		static void updateConfigs()
		{ 
			var configs = TCAScenario.NamedConfigs.Keys.ToList();
			var first = namedConfigsListBox.Items.Count == 0;
			configs.Add(string.Empty); namedConfigsListBox.Items = configs; 
			if(first) namedConfigsListBox.SelectItem(configs.Count-1);
		}

		static void SelectConfig_start() 
		{ 
			namedConfigsListBox.styleListBox  = Styles.list_box;
			namedConfigsListBox.styleListItem = Styles.list_item;
			namedConfigsListBox.windowRect    = MainWindow;
			if(TCAScenario.NamedConfigs.Count > 0)
				namedConfigsListBox.DrawBlockingSelector(); 
		}

		static void SelectConfig()
		{
			if(TCAScenario.NamedConfigs.Count == 0)
				GUILayout.Label("[Nothing Saved]", GUILayout.ExpandWidth(true));
			else
			{
				namedConfigsListBox.DrawButton();
				var new_config = TCAScenario.GetConfig(namedConfigsListBox.SelectedIndex);
				if(new_config != selected_config)
				{
					selected_config = new_config;
					config_name = selected_config != null? selected_config.Name : string.Empty;
				}
			}
		}

		static void SelectConfig_end()
		{
			if(TCAScenario.NamedConfigs.Count == 0) return;
			namedConfigsListBox.DrawDropDown();
			namedConfigsListBox.CloseOnOutsideClick();
		}
		#endregion

		protected override void DrawMainWindow(int windowID)
		{
			//help button
			if(GUI.Button(new Rect(MainWindow.width - 23f, 2f, 20f, 18f), 
			                  new GUIContent("?", "Help"))) TCAManual.Toggle();
			if(TCA.Controllable)
			{
				//options button
				if(GUI.Button(new Rect(2f, 2f, 70f, 18f), 
				              new GUIContent("advanced", "Advanced configuration"), 
				              advOptions? Styles.green_button : Styles.normal_button)) 
					advOptions = !advOptions;
				GUILayout.BeginVertical();
				GUILayout.BeginHorizontal();
				//tca toggle
				if(GUILayout.Button("Enabled", 
				                    CFG.Enabled? Styles.green_button : Styles.grey_button,
				                    GUILayout.Width(70)))
					Apply(tca => tca.ToggleTCA());
				//squad mode switch
				Utils.ButtonSwitch("Squadron Mode", ref squad_mode, 
				                   "Control autopilot on all squadron vessels", 
				                   GUILayout.ExpandWidth(false));
				if(squad_mode) CFG.Squad = Utils.IntSelector(CFG.Squad, 1, tooltip: "Squad ID");
				GUILayout.FlexibleSpace();
				StatusString();
				GUILayout.EndHorizontal();
				SelectConfig_start();
				AdvancedOptions();
				AttitudeControls();
				InOrbitControl();
				AutopilotControls();
				MacroControls();
				WaypointList();
				EnginesControl();
				#if DEBUG
				if(!string.IsNullOrEmpty(DebugMessage))
					GUILayout.Label(DebugMessage, Styles.white, GUILayout.ExpandWidth(true));
	//			EnginesInfo();
				#endif
				SelectConfig_end();
				GUILayout.EndVertical();
			}
			else GUILayout.Label("Vessel is Uncontrollable", Styles.label, GUILayout.ExpandWidth(true), GUILayout.ExpandHeight(true));
			base.DrawMainWindow(windowID);
		}

		static void StatusString()
		{
			var state = "Disabled";
			var style = Styles.grey;
			if(TCA.IsStateSet(TCAState.Enabled))
			{
				if(TCA.IsStateSet(TCAState.ObstacleAhead))
				{ state = "Obstacle On Course"; style = Styles.red; }
				else if(TCA.IsStateSet(TCAState.GroundCollision))
				{ state = "Ground Collision"; style = Styles.red; }
				else if(TCA.IsStateSet(TCAState.LoosingAltitude))
				{ state = "Loosing Altitude"; style = Styles.red; }
                else if(TCA.IsStateSet(TCAState.Unoptimized))
				{ state = "Engines Unoptimized"; style = Styles.yellow; }
				else if(TCA.IsStateSet(TCAState.Ascending))
				{ state = "Ascending"; style = Styles.yellow; }
				else if(TCA.IsStateSet(TCAState.VTOLAssist))
				{ state = "VTOL Assist On"; style = Styles.yellow; }
				else if(TCA.IsStateSet(TCAState.StabilizeFlight))
				{ state = "Stabilizing Flight"; style = Styles.yellow; }
				else if(TCA.IsStateSet(TCAState.Landing))
				{ state = "Landing..."; style = Styles.green; }
				else if(TCA.IsStateSet(TCAState.CheckingSite))
				{ state = "Checking Landing Site"; style = Styles.yellow; }
				else if(TCA.IsStateSet(TCAState.Searching))
				{ state = "Searching For Landing Site"; style = Styles.yellow; }
				else if(TCA.IsStateSet(TCAState.Scanning))
				{ state = string.Format("Scanning Surface {0:P0}", TCA.LND.Progress); style = Styles.yellow; }
				else if(TCA.IsStateSet(TCAState.AltitudeControl))
				{ state = "Altitude Control"; style = Styles.green; }
				else if(TCA.IsStateSet(TCAState.VerticalSpeedControl))
				{ state = "Vertical Speed Control"; style = Styles.green; }
				else if(TCA.State == TCAState.Nominal)
				{ state = "Systems Nominal"; style = Styles.green; }
				else if(TCA.State == TCAState.NoActiveEngines)
				{ state = "No Active Engines"; style = Styles.yellow; }
				else if(TCA.State == TCAState.NoEC)
				{ state = "No Electric Charge"; style = Styles.red; }
				else //this should never happen
				{ state = "Unknown State"; style = Styles.magenta_button; }
			}
			GUILayout.Label(state, style, GUILayout.ExpandWidth(false));
		}

		static void AdvancedOptions()
		{
			if(!advOptions) return;
			GUILayout.BeginVertical(Styles.white);
			GUILayout.Label(TCATitle, Styles.label, GUILayout.ExpandWidth(true));
			if(GUILayout.Button("Reload TCA Settings", Styles.yellow_button, GUILayout.ExpandWidth(true))) 
			{
				TCAScenario.LoadGlobals();
				TCA.OnReloadGlobals();
			}
			//change key binding
			GUILayout.BeginHorizontal();
			GUILayout.Label("Press to change TCA hotkey:", GUILayout.ExpandWidth(false));
			if(GUILayout.Button(selecting_key? new GUIContent("?", "Choose new TCA hotkey") : 
			                    new GUIContent(TCA_Key.ToString(), "Select TCA Hotkey"), 
			                    selecting_key? Styles.yellow_button : Styles.green_button, 
			                    GUILayout.Width(40)))
			{ selecting_key = true; ScreenMessages.PostScreenMessage("Enter new key to toggle TCA", 5, ScreenMessageStyle.UPPER_CENTER); }
			GUILayout.EndHorizontal();
			CFG.VTOLAssistON = GUILayout.Toggle(CFG.VTOLAssistON, "Assist with vertical takeoff and landing", GUILayout.ExpandWidth(true));
			CFG.StabilizeFlight = GUILayout.Toggle(CFG.StabilizeFlight, "Try to stabilize flight if spinning uncontrollably", GUILayout.ExpandWidth(true));
			GUILayout.BeginHorizontal();
			CFG.VSControlSensitivity = Utils.FloatSlider("Sensitivity of throttle controls", CFG.VSControlSensitivity, 0.001f, 0.05f, "P2");
			GUILayout.EndHorizontal();
			CFG.AutoTune = GUILayout.Toggle(CFG.AutoTune, "Autotune engines' controller parameters", GUILayout.ExpandWidth(false));
			ControllerProperties();
			ConfigsGUI();
			GUILayout.EndVertical();
		}

		static void AttitudeControls()
		{
			GUILayout.BeginHorizontal();
			GUILayout.Label(new GUIContent("T-SAS", "Thrust attitude control"), 
			                CFG.AT? Styles.cyan : Styles.white, GUILayout.ExpandWidth(false));
			if(GUILayout.Button(new GUIContent("Kill", "Kill rotation"), CFG.AT[Attitude.KillRotation]? 
			                    Styles.green_button : Styles.yellow_button, GUILayout.ExpandWidth(false)))
				CFG.AT.XToggle(Attitude.KillRotation);
			if(GUILayout.Button(new GUIContent("Hold", "Hold current attitude"), CFG.AT[Attitude.HoldAttitude]? 
			                    Styles.green_button : Styles.yellow_button, GUILayout.ExpandWidth(false)))
				CFG.AT.XToggle(Attitude.HoldAttitude);
			if(GUILayout.Button(new GUIContent("Maneuver", "Maneuver node"), CFG.AT[Attitude.ManeuverNode]? 
			                    Styles.green_button : Styles.yellow_button, GUILayout.ExpandWidth(false)))
				CFG.AT.XToggle(Attitude.ManeuverNode);
			if(GUILayout.Button(new GUIContent("PG", "Prograde"), CFG.AT[Attitude.Prograde]? 
			                    Styles.green_button : Styles.yellow_button, GUILayout.ExpandWidth(false)))
				CFG.AT.XToggle(Attitude.Prograde);
			if(GUILayout.Button(new GUIContent("RG", "Retrograde"), CFG.AT[Attitude.Retrograde]? 
			                    Styles.green_button : Styles.yellow_button, GUILayout.ExpandWidth(false)))
				CFG.AT.XToggle(Attitude.Retrograde);
			if(GUILayout.Button(new GUIContent("R+", "Radial"), CFG.AT[Attitude.Radial]? 
			                    Styles.green_button : Styles.yellow_button, GUILayout.ExpandWidth(false)))
				CFG.AT.XToggle(Attitude.Radial);
			if(GUILayout.Button(new GUIContent("R-", "AntiRadial"), CFG.AT[Attitude.AntiRadial]? 
			                    Styles.green_button : Styles.yellow_button, GUILayout.ExpandWidth(false)))
				CFG.AT.XToggle(Attitude.AntiRadial);
			if(GUILayout.Button(new GUIContent("N+", "Normal"), CFG.AT[Attitude.Normal]? 
			                    Styles.green_button : Styles.yellow_button, GUILayout.ExpandWidth(false)))
				CFG.AT.XToggle(Attitude.Normal);
			if(GUILayout.Button(new GUIContent("N-", "AntiNormal"), CFG.AT[Attitude.AntiNormal]? 
			                    Styles.green_button : Styles.yellow_button, GUILayout.ExpandWidth(false)))
				CFG.AT.XToggle(Attitude.AntiNormal);
			if(GUILayout.Button(new GUIContent("T+", "Target"), CFG.AT[Attitude.Target]? 
			                    Styles.green_button : Styles.yellow_button, GUILayout.ExpandWidth(false)))
				CFG.AT.XToggle(Attitude.Target);
			if(GUILayout.Button(new GUIContent("T-", "AntiTarget"), CFG.AT[Attitude.AntiTarget]? 
			                    Styles.green_button : Styles.yellow_button, GUILayout.ExpandWidth(false)))
				CFG.AT.XToggle(Attitude.AntiTarget);
			if(Utils.ButtonSwitch("rV+", CFG.AT[Attitude.RelVel], "Relative Velocity", GUILayout.ExpandWidth(false)))
				CFG.AT.XToggle(Attitude.RelVel);
			if(Utils.ButtonSwitch("rV-", CFG.AT[Attitude.AntiRelVel], "Against Relative Velocity", GUILayout.ExpandWidth(false)))
				CFG.AT.XToggle(Attitude.AntiRelVel);
			if(GUILayout.Button("Auto", CFG.AT[Attitude.Custom]? Styles.green_button : Styles.grey, GUILayout.ExpandWidth(false)))
				CFG.AT.OffIfOn(Attitude.Custom);
			GUILayout.Label(CFG.AT? string.Format("Err: {0:F1}°", TCA.ATC.AttitudeError) : "Err: N/A", 
			                TCA.ATC.Aligned? Styles.green : Styles.white, GUILayout.ExpandWidth(true));
			GUILayout.EndHorizontal();
		}

		static void InOrbitControl()
		{
			if(!VSL.InOrbit) return;
			GUILayout.BeginHorizontal();
			if(VSL.Countdown >= 0 && Utils.ButtonSwitch("Warp", CFG.WarpToNode, "Warp to the burn", GUILayout.ExpandWidth(false)))
			{
				CFG.WarpToNode = !CFG.WarpToNode;
				if(!CFG.WarpToNode) TimeWarp.SetRate(0, false);
			}
			if(VSL.HasManeuverNode) 
			{
				if(GUILayout.Button(CFG.AP[Autopilot.Maneuver]? "Abort Maneuver" : "Execute Maneuver", 
				                    CFG.AP[Autopilot.Maneuver]? Styles.red_button : Styles.green_button, GUILayout.ExpandWidth(true)))
					CFG.AP.XToggle(Autopilot.Maneuver);
			}
			if(VSL.HasTarget && !CFG.AP[Autopilot.Maneuver])
			{
				if(Utils.ButtonSwitch("Match Velocity", CFG.AP[Autopilot.MatchVel], 
				                      "Match orbital velocity with the target", GUILayout.ExpandWidth(true)))
					CFG.AP.XToggle(Autopilot.MatchVel);
				if(Utils.ButtonSwitch("Brake Near Target", CFG.AP[Autopilot.MatchVelNear], 
				                      "Match orbital velocity with the target at nearest point", GUILayout.ExpandWidth(true)))
					CFG.AP.XToggle(Autopilot.MatchVelNear);
			}
			if(VSL.Countdown >= 0)
				GUILayout.Label(string.Format("Countdown: {0:F1}s", VSL.Countdown), Styles.white, GUILayout.ExpandWidth(true));
			if(VSL.TTB >= 0)
				GUILayout.Label(string.Format("Full Thrust: {0:F1}s", VSL.TTB), Styles.yellow, GUILayout.ExpandWidth(true));
			GUILayout.EndHorizontal();
		}

		static void ConfigsGUI()
		{
			GUILayout.BeginVertical();
			GUILayout.Label("Manage named configurations", Styles.label, GUILayout.ExpandWidth(true));
			GUILayout.BeginHorizontal();
			GUILayout.Label("Name:", GUILayout.Width(50));
			config_name = GUILayout.TextField(config_name, GUILayout.ExpandWidth(true), GUILayout.MinWidth(50));
			if(TCAScenario.NamedConfigs.ContainsKey(config_name))
			{
				if(GUILayout.Button(new GUIContent("Overwrite", "Overwrite selected configuration with the current one"), 
				                    Styles.red_button, GUILayout.Width(70)))
					TCAScenario.SaveNamedConfig(config_name, CFG, true);
			}
			else if(GUILayout.Button(new GUIContent("Add", "Save current configuration"), 
			                         Styles.green_button, GUILayout.Width(50)) && config_name != string.Empty) 
			{
				TCAScenario.SaveNamedConfig(config_name, CFG);
				updateConfigs();
				namedConfigsListBox.SelectItem(TCAScenario.NamedConfigs.IndexOfKey(config_name));
			}
			SelectConfig();
			if(GUILayout.Button(new GUIContent("Load", "Load selected configuration"),
			                    Styles.yellow_button, GUILayout.Width(50)) && selected_config != null) 
				CFG.CopyFrom(selected_config);
			if(GUILayout.Button(new GUIContent("Delete", "Delete selected configuration"), 
			                    Styles.red_button, GUILayout.Width(50)) && selected_config != null)
			{ 
				TCAScenario.NamedConfigs.Remove(selected_config.Name);
				namedConfigsListBox.SelectItem(namedConfigsListBox.SelectedIndex-1);
				updateConfigs();
				selected_config = null;
			}
			GUILayout.EndHorizontal();
			GUILayout.EndVertical();
		}

		static void ControllerProperties()
		{
			if(CFG.AutoTune) return;
			//steering modifiers
			GUILayout.BeginHorizontal();
			CFG.SteeringGain = Utils.FloatSlider("Steering Gain", CFG.SteeringGain, 0, 1, "P1");
			CFG.PitchYawLinked = GUILayout.Toggle(CFG.PitchYawLinked, "Link Pitch&Yaw", GUILayout.ExpandWidth(false));
			GUILayout.EndHorizontal();
			GUILayout.BeginHorizontal();
			if(CFG.PitchYawLinked && !CFG.AutoTune)
			{
				CFG.SteeringModifier.x = Utils.FloatSlider("Pitch&Yaw", CFG.SteeringModifier.x, 0, 1, "P1");
				CFG.SteeringModifier.z = CFG.SteeringModifier.x;
			}
			else
			{
				CFG.SteeringModifier.x = Utils.FloatSlider("Pitch", CFG.SteeringModifier.x, 0, 1, "P1");
				CFG.SteeringModifier.z = Utils.FloatSlider("Yaw", CFG.SteeringModifier.z, 0, 1, "P1");
			}
			CFG.SteeringModifier.y = Utils.FloatSlider("Roll", CFG.SteeringModifier.y, 0, 1, "P1");
			GUILayout.EndHorizontal();
			//engines
			CFG.Engines.DrawControls("Engines Controller");
		}

		static void AutopilotControls()
		{
			if(VSL.OnPlanet)
			{
				//vertical speed or altitude limit
				GUILayout.BeginHorizontal();
				if(CFG.VF[VFlight.AltitudeControl])
				{
					update_altitude();
					var above_ground = VSL.AltitudeAboveGround;
					var style = above_ground? Styles.green : Styles.red;
					GUILayout.Label(string.Format("Altitude: {0:F2}m {1:+0.0;-0.0;+0.0}m/s", 
					                              VSL.Altitude, VSL.VerticalSpeedDisp), 
					                GUILayout.Width(190));
					GUILayout.Label(new GUIContent("Set Point (m):", above_ground? 
					                               "Setpoint is above the ground" : "Warning! Setpoint is below the ground"), 
					                GUILayout.Width(90));
					s_altitude = GUILayout.TextField(s_altitude, style, GUILayout.ExpandWidth(true), GUILayout.MinWidth(60));
					if(GUILayout.Button("Set", Styles.normal_button, GUILayout.Width(50))) 
					{
						if(float.TryParse(s_altitude, out altitude)) set_altitude();
						else altitude = CFG.DesiredAltitude;
					}
					if(GUILayout.Button("-10m", Styles.normal_button, GUILayout.Width(50))) 
					{ altitude -= 10; set_altitude(); }
					if(GUILayout.Button("+10m", Styles.normal_button, GUILayout.Width(50))) 
					{ altitude += 10; set_altitude(); }
				}
				else
				{
					GUILayout.Label(string.Format("Vertical Speed: {0:0.00m/s}", VSL.VerticalSpeedDisp), GUILayout.Width(190));
					GUILayout.Label("Set Point: " + (CFG.VerticalCutoff < GLB.VSC.MaxSpeed? 
					                                 CFG.VerticalCutoff.ToString("0.0m/s") : "OFF"), 
					                GUILayout.ExpandWidth(false));
					var VSP = GUILayout.HorizontalSlider(CFG.VerticalCutoff, 
		                                                -GLB.VSC.MaxSpeed, 
		                                                GLB.VSC.MaxSpeed);
					if(Mathf.Abs(VSP-CFG.VerticalCutoff) > 1e-5) set_vspeed(VSP);
				}
				TCA.THR.BlockThrottle(GUILayout.Toggle(CFG.BlockThrottle, 
					                                   new GUIContent("AutoThrottle",
					                                                  CFG.VF[VFlight.AltitudeControl]?
					                                                  "Change altitude with throttle controls" :
					                                                  "Set vertical speed with throttle controls"), 
					                                   GUILayout.ExpandWidth(false)));
				GUILayout.EndHorizontal();
				#if DEBUG
				GUILayout.BeginHorizontal();
				GUILayout.Label(string.Format("vV: {0:0.0}m/s", VSL.AbsVerticalSpeed), GUILayout.Width(100));
				GUILayout.Label(string.Format("A: {0:0.0}m/s2", VSL.VerticalAccel), GUILayout.Width(80));
				GUILayout.Label(string.Format("ApA: {0:0.0}m", VSL.orbit.ApA), GUILayout.Width(120));
				GUILayout.Label(string.Format("hV: {0:0.0}m/s", VSL.HorizontalSpeed), GUILayout.Width(100));
				GUILayout.EndHorizontal();
				GUILayout.BeginHorizontal();
				GUILayout.Label(string.Format("VSP: {0:0.0m/s}", CFG.VerticalCutoff), GUILayout.Width(100));
				GUILayout.Label(string.Format("TWR: {0:0.0}", VSL.DTWR), GUILayout.Width(80));
				if(TCA.RAD.AltitudeAhead.Equals(float.MinValue)) GUILayout.Label("Obst: N/A", GUILayout.Width(120));
				else GUILayout.Label(string.Format("Obst: {0:0.0}m", TCA.RAD.AltitudeAhead), GUILayout.Width(120));
				GUILayout.Label(string.Format("Orb: {0:0.0}m/s", Math.Sqrt(VSL.StG*(VSL.wCoM-VSL.mainBody.position).magnitude)), GUILayout.Width(100));
				GUILayout.EndHorizontal();
				#endif
				//autopilot toggles
				GUILayout.BeginHorizontal();
				if(GUILayout.Button(new GUIContent("Stop", "Kill horizontal velocity"), 
				                    CFG.HF[HFlight.Stop]? Styles.green_button : Styles.yellow_button,
				                    GUILayout.Width(50)))
				{
					if(CFG.HF[HFlight.Stop]) apply_cfg(cfg => cfg.HF.OffIfOn(HFlight.Stop));
					else apply_cfg(cfg => { cfg.HF.XOn(HFlight.Stop); cfg.StopMacro(); });
				}
				if(GUILayout.Button(new GUIContent("Anchor", "Hold current position"), 
				                    CFG.Nav.Any(Navigation.AnchorHere, Navigation.Anchor)? 
				                    Styles.green_button : Styles.yellow_button,
				                    GUILayout.Width(60)))
					apply_cfg(cfg => cfg.Nav.XToggle(Navigation.AnchorHere));
				if(GUILayout.Button(new GUIContent("Level", "Point thrust vertically"), 
				                    CFG.HF[HFlight.Level]? 
				                    Styles.green_button : Styles.yellow_button,
				                    GUILayout.Width(50)))
					apply_cfg(cfg => cfg.HF.XToggle(HFlight.Level));
				if(GUILayout.Button(new GUIContent("Land", "Try to land on a nearest flat surface"), 
				                    CFG.AP[Autopilot.Land]? Styles.green_button : Styles.yellow_button,
				                    GUILayout.Width(50)))
				{
					var state = !CFG.AP[Autopilot.Land];
					if(state) { follow_me(); CFG.AP.XOn(Autopilot.Land); }
					else apply_cfg(cfg => cfg.AP.XOffIfOn(Autopilot.Land));
				}
				if(GUILayout.Button(new GUIContent("Cruise", "Maintain course and speed"), 
				                    CFG.HF[HFlight.CruiseControl]? Styles.green_button : Styles.yellow_button,
				                    GUILayout.Width(60)))
				{
					CFG.HF.XToggle(HFlight.CruiseControl);
					if(CFG.HF[HFlight.CruiseControl]) follow_me();
				}
				if(GUILayout.Button(new GUIContent("Hover", "Maintain altitude"), 
				                    CFG.VF[VFlight.AltitudeControl]? Styles.green_button : Styles.yellow_button,
				                    GUILayout.Width(60)))
					apply_cfg(cfg => cfg.VF.XToggle(VFlight.AltitudeControl));
				var follow_terrain = GUILayout.Toggle(CFG.AltitudeAboveTerrain, 
				                                      new GUIContent("Follow Terrain", 
				                                                     "Keep altitude above the ground and avoid collisions"),
				                                      GUILayout.ExpandWidth(false));
				if(follow_terrain != CFG.AltitudeAboveTerrain)
					Apply(tca => tca.ALT.SetAltitudeAboveTerrain(follow_terrain));
				GUILayout.EndHorizontal();
				//navigator toggles
				GUILayout.BeginHorizontal();
				if(VSL.HasTarget && !CFG.Nav.Paused)
				{
					if(GUILayout.Button(new GUIContent("Go To", "Fly to current target"), 
					                    CFG.Nav[Navigation.GoToTarget]? Styles.green_button 
					                    : Styles.yellow_button,
					                    GUILayout.Width(50)))
					{
						CFG.Nav.XOn(Navigation.GoToTarget);
						if(CFG.Nav[Navigation.GoToTarget]) follow_me();
					}
					if(GUILayout.Button(new GUIContent("Follow", "Follow current target"), 
						CFG.Nav[Navigation.FollowTarget]? Styles.green_button 
						: Styles.yellow_button,
						GUILayout.Width(50)))
						Apply(tca => 
					{
						if(TCA.vessel.targetObject as Vessel == tca.vessel) return;
						tca.vessel.targetObject = TCA.vessel.targetObject;
						tca.CFG.Nav.XOn(Navigation.FollowTarget);
					});
				}
				else 
				{
					GUILayout.Label(new GUIContent("Go To", CFG.Nav.Paused? "Paused" : "No target selected"),  
					                Styles.grey, GUILayout.Width(50));
					GUILayout.Label(new GUIContent("Follow", CFG.Nav.Paused? "Paused" : "No target selected"),  
					                Styles.grey, GUILayout.Width(50));
				}
				if(squad_mode)
				{
					if(CFG.Nav.Paused)
						GUILayout.Label(new GUIContent("Follow Me", "Make the squadron follow"),  
						                Styles.grey, GUILayout.Width(75));
					else if(GUILayout.Button(new GUIContent("Follow Me", "Make the squadron follow"), 
					                         Styles.yellow_button, GUILayout.Width(75)))
						follow_me();
				}
				if(selecting_target)
				{
					if(GUILayout.Button("Cancel", Styles.red_button, GUILayout.Width(120)))
						selecting_target = false;
				}
				else if(VSL.HasTarget && 
				        !(VSL.Target is WayPoint) && 
				        (CFG.Waypoints.Count == 0 || VSL.Target != CFG.Waypoints.Peek().GetTarget()))
				{
					if(GUILayout.Button(new GUIContent("Add As Waypoint", "Add current target as a waypoint"), 
					                    Styles.yellow_button, GUILayout.Width(120)))
					{
						CFG.Waypoints.Enqueue(new WayPoint(VSL.Target));
						CFG.ShowWaypoints = true;
					}
				}
				else if(GUILayout.Button(new GUIContent("Add Waypoint", "Select a new waypoint"), 
				                         Styles.yellow_button, GUILayout.Width(120)))
				{
					selecting_target = true;
					CFG.ShowWaypoints = true;
				}
				if(CFG.Waypoints.Count > 0 && !CFG.Nav.Paused)
				{
					if(GUILayout.Button("Follow Route",
					                    CFG.Nav[Navigation.FollowPath]? Styles.green_button 
					                    : Styles.yellow_button,
					                    GUILayout.Width(90)))
					{
						CFG.Nav.XToggle(Navigation.FollowPath);
						if(CFG.Nav[Navigation.FollowPath])
							follow_me();
					}
				}
				else GUILayout.Label(new GUIContent("Follow Route", CFG.Nav.Paused? "Paused" : "Add some waypoints first"), 
				                     Styles.grey, GUILayout.Width(90));
				var max_nav_speed = Utils.FloatSlider("", CFG.MaxNavSpeed, GLB.PN.MinSpeed, GLB.PN.MaxSpeed, "0 m/s", 60, "Maximum horizontal speed on autopilot");
				if(Mathf.Abs(max_nav_speed-CFG.MaxNavSpeed) > 1e-5)
					apply_cfg(cfg => cfg.MaxNavSpeed = max_nav_speed);
				GUILayout.EndHorizontal();
			}
//			else GUILayout.Label("Autopilot Not Available In Orbit", Styles.grey, GUILayout.ExpandWidth(true));
		}

		static bool selecting_macro;
		static void MacroControls()
		{
			GUILayout.BeginHorizontal();
			if(CFG.SelectedMacro != null && CFG.MacroIsActive)
			{
				GUILayout.Label(new GUIContent(CFG.SelectedMacro.Title, "The macro is executing..."), 
				                Styles.yellow, GUILayout.ExpandWidth(true));
				CFG.MacroIsActive &= !GUILayout.Button("Pause", Styles.green_button, GUILayout.Width(70));
				if(GUILayout.Button("Stop", Styles.red_button, GUILayout.ExpandWidth(false))) 
					CFG.StopMacro();
				GUILayout.Label("Edit", Styles.grey, GUILayout.ExpandWidth(false));
			}
			else if(CFG.SelectedMacro != null)
			{
				if(GUILayout.Button(new GUIContent(CFG.SelectedMacro.Title, "Select a macro from databases"), 
				                    Styles.normal_button, GUILayout.ExpandWidth(true))) 
					selecting_macro = !selecting_macro;
				CFG.MacroIsActive |= GUILayout.Button(CFG.SelectedMacro.Active? "Resume" : "Execute", 
				                                      Styles.yellow_button, GUILayout.Width(70));
				if(GUILayout.Button("Stop", CFG.SelectedMacro.Active? 
				                    Styles.red_button : Styles.grey, GUILayout.ExpandWidth(false))) 
					CFG.SelectedMacro.Rewind();
				if(GUILayout.Button("Edit", Styles.yellow_button, GUILayout.ExpandWidth(false)))
					TCAMacroEditor.Edit(CFG);
			}
			else 
			{
				if(GUILayout.Button("Select Macro", Styles.normal_button, GUILayout.ExpandWidth(true))) 
					selecting_macro = !selecting_macro;
				if(GUILayout.Button("New Macro", Styles.green_button, GUILayout.ExpandWidth(false)))
					TCAMacroEditor.Edit(CFG);
			}
			GUILayout.EndHorizontal();
			if(selecting_macro)
			{
				TCAMacro macro = null;
				if(TCAMacroEditor.DrawMacroSelector(CFG, out macro)) 
				{
					if(macro != null) 
					{
						CFG.SelectedMacro = (TCAMacro)macro.GetCopy();
						CFG.MacroIsActive = false;
					}
					selecting_macro = false;
				}
			}
		}

		static void WaypointList()
		{
			if(CFG.Waypoints.Count == 0) return;
			GUILayout.BeginVertical();
			if(GUILayout.Button(CFG.ShowWaypoints? "Hide Waypoints" : "Show Waypoints", 
			                    Styles.yellow_button,
			                    GUILayout.ExpandWidth(true)))
				CFG.ShowWaypoints = !CFG.ShowWaypoints;
			if(CFG.ShowWaypoints)
			{
				GUILayout.BeginVertical(Styles.white);
				waypointsScroll = GUILayout.BeginScrollView(waypointsScroll, GUILayout.Height(Utils.ClampH(lineHeight*(CFG.Waypoints.Count+1), controlsHeight)));
				GUILayout.BeginVertical();
				int i = 0;
				var num = (float)(CFG.Waypoints.Count-1);
				var del = new HashSet<WayPoint>();
				var col = GUI.contentColor;
				foreach(var wp in CFG.Waypoints)
				{
					GUILayout.BeginHorizontal();
					GUI.contentColor = marker_color(i, num);
					var label = string.Format("{0}) {1}", 1+i, wp.GetName());
					if(CFG.Nav[Navigation.FollowPath] && i == 0)
					{
						var d = wp.DistanceTo(vessel);
						label += string.Format(" <= {0}", Utils.DistanceToStr(d)); 
						if(vessel.horizontalSrfSpeed > 0.1)
							label += string.Format(", ETA {0:c}", new TimeSpan(0,0,(int)(d/vessel.horizontalSrfSpeed)));
					}
					if(GUILayout.Button(label,GUILayout.ExpandWidth(true)))
						FlightGlobals.fetch.SetVesselTarget(wp.GetTarget());
					GUI.contentColor = col;
					GUILayout.FlexibleSpace();
					if(GUILayout.Button(new GUIContent("Land", "Land on arrival"), 
					                    wp.Land? Styles.green_button : Styles.yellow_button, 
					                    GUILayout.Width(50))) 
						wp.Land = !wp.Land;
					if(GUILayout.Button(new GUIContent("||", "Pause on arrival"), 
					                    wp.Pause? Styles.green_button : Styles.yellow_button, 
					                    GUILayout.Width(25))) 
						wp.Pause = !wp.Pause;
					if(GUILayout.Button(new GUIContent("X", "Delete waypoint"), 
					                    Styles.red_button, GUILayout.Width(25))) 
						del.Add(wp);
					GUILayout.EndHorizontal();
					i++;
				}
				GUI.contentColor = col;
				if(GUILayout.Button("Clear", Styles.red_button, GUILayout.ExpandWidth(true)))
					CFG.Waypoints.Clear();
				else if(del.Count > 0)
				{
					var edited = CFG.Waypoints.Where(wp => !del.Contains(wp)).ToList();
					CFG.Waypoints = new Queue<WayPoint>(edited);
				}
				if(CFG.Waypoints.Count == 0 && CFG.Nav) CFG.HF.XOn(HFlight.Stop);
				GUILayout.EndVertical();
				GUILayout.EndScrollView();
				GUILayout.EndVertical();
			}
			GUILayout.EndVertical();
		}

		static void EnginesControl()
		{
			GUILayout.BeginVertical();
			if(CFG.ActiveProfile.NumManual > 0)
			{
				if(GUILayout.Button(CFG.ShowManualLimits? "Hide Manual Engines" : "Show Manual Engines", 
				                    Styles.yellow_button,
				                    GUILayout.ExpandWidth(true)))
					CFG.ShowManualLimits = !CFG.ShowManualLimits;
				if(CFG.ShowManualLimits) CFG.EnginesProfiles.DrawManual(Utils.ClampH(lineHeight*CFG.ActiveProfile.NumManual, controlsHeight));
			}
			if(GUILayout.Button(CFG.ShowEnginesProfiles? "Hide Engines Profiles" : "Show Engines Profiles", 
			                    Styles.yellow_button,
			                    GUILayout.ExpandWidth(true)))
				CFG.ShowEnginesProfiles = !CFG.ShowEnginesProfiles;
			if(CFG.ShowEnginesProfiles) CFG.EnginesProfiles.Draw(controlsHeight*2);
			GUILayout.EndVertical();
		}

		#region Waypoints Overlay
		static Color marker_color(int i, float N)
		{ 
			if(N.Equals(0)) return Color.red;
			var t = i/N;
			return t < 0.5f ? 
				Color.Lerp(Color.red, Color.green, t*2).Normalized() : 
				Color.Lerp(Color.green, Color.cyan, (t-0.5f)*2).Normalized(); 
		}

		//adapted from MechJeb
		bool clicked;
		DateTime clicked_time;
		void WaypointOverlay()
		{
			if(TCA == null || !TCA.Available || !showHUD) return;
			if(selecting_target)
			{
				var coords = MapView.MapIsEnabled? 
					Utils.GetMouseCoordinates(vessel.mainBody) :
					Utils.GetMouseFlightCoordinates();
				if(coords != null)
				{
					var t = new WayPoint(coords);
					DrawGroundMarker(vessel.mainBody, coords.Lat, coords.Lon, new Color(1.0f, 0.56f, 0.0f));
					GUI.Label(new Rect(Input.mousePosition.x + 15, Screen.height - Input.mousePosition.y, 200, 50), 
					          string.Format("{0} {1}\n{2}", coords, Utils.DistanceToStr(t.DistanceTo(vessel)), 
					                        ScienceUtil.GetExperimentBiome(vessel.mainBody, coords.Lat, coords.Lon)));
					if(!clicked)
					{ 
						if(Input.GetMouseButtonDown(0)) clicked = true;
						else if(Input.GetMouseButtonDown(1))  
						{ clicked_time = DateTime.Now; clicked = true; }
					}
					else 
					{
						if(Input.GetMouseButtonUp(0))
						{ 
							AddTargetDamper.Run(() => CFG.Waypoints.Enqueue(t));
							CFG.ShowWaypoints = true;
							clicked = false;
						}
						if(Input.GetMouseButtonUp(1))
						{ 
							selecting_target &= (DateTime.Now - clicked_time).TotalSeconds >= 0.5;
							clicked = false; 
						}
					}
				}
			}
			if(CFG.ShowWaypoints)
			{
				var i = 0;
				var num = (float)(CFG.Waypoints.Count-1);
				WayPoint wp0 = null;
				foreach(var wp in CFG.Waypoints)
				{
					wp.UpdateCoordinates(vessel.mainBody);
					var c = marker_color(i, num);
					if(wp0 == null) DrawPath(vessel, wp, c);
					else DrawPath(vessel.mainBody, wp0, wp, c);
					DrawGroundMarker(vessel.mainBody, wp.Lat, wp.Lon, c);
					wp0 = wp; i++;
				}
			}
		}

		static Material _icon_material;
		static Material IconMaterial
		{
			get
			{
				if(_icon_material == null) 
					_icon_material = new Material(Shader.Find("Particles/Additive"));
				return _icon_material;
			}
		}

		static void DrawMarker(Vector3 icon_center, Color c, float r, Texture2D texture)
		{
			if(texture == null) texture = WayPointMarker;
			var icon_rect = new Rect(icon_center.x - r * 0.5f, (float)Screen.height - icon_center.y - r * 0.5f, r, r);
			Graphics.DrawTexture(icon_rect, texture, new Rect(0f, 0f, 1f, 1f), 0, 0, 0, 0, c, IconMaterial);
		}

		static void DrawGroundMarker(CelestialBody body, double lat, double lon, Color c, float r = IconSize, Texture2D texture = null)
		{
			Vector3d center;
			Camera camera;
			if(MapView.MapIsEnabled)
			{
				camera = PlanetariumCamera.Camera;
				//TODO: cache local center coordinates of the marker
				var up = body.GetSurfaceNVector(lat, lon);
				var h  = Utils.TerrainAltitude(body, lat, lon);
				if(h < body.Radius) h = body.Radius;
				center = body.position + h * up;
			}
			else
			{
				camera = FlightCamera.fetch.mainCamera;
				center = body.GetWorldSurfacePosition(lat, lon, Utils.TerrainAltitude(body, lat, lon)+GLB.WaypointHeight);
				if(Vector3d.Dot(center-camera.transform.position, 
				                camera.transform.forward) <= 0) return;
			}
			if(IsOccluded(center, body)) return;
			DrawMarker(camera.WorldToScreenPoint(MapView.MapIsEnabled? ScaledSpace.LocalToScaledSpace(center) : center), c, r, texture);
		}

		static void DrawPath(CelestialBody body, WayPoint wp0, WayPoint wp1, Color c)
		{
			var D = wp1.AngleTo(wp0);
			var N = (int)Mathf.Clamp((float)D*Mathf.Rad2Deg, 2, 5);
			var dD = D/N;
			for(int i = 1; i<N; i++)
			{
				var p = wp0.PointBetween(wp1, dD*i);
				DrawGroundMarker(body, p.Lat, p.Lon, c, IconSize/2, PathNodeMarker);
			}
		}

		static void DrawPath(Vessel v, WayPoint wp1, Color c)
		{
			var wp0 = new WayPoint();
			wp0.Lat = v.latitude; wp0.Lon = v.longitude;
			DrawPath(v.mainBody, wp0, wp1, c);
		}

		//Tests if byBody occludes worldPosition, from the perspective of the planetarium camera
		static bool IsOccluded(Vector3d worldPosition, CelestialBody byBody)
		{
			return Vector3d.Angle(ScaledSpace.ScaledToLocalSpace(PlanetariumCamera.Camera.transform.position) - 
			                      worldPosition, byBody.position - worldPosition) <= 90.0;
		}
		#endregion

		#if DEBUG
		static Vector2 eInfoScroll;
		static void EnginesInfo()
		{
			GUILayout.BeginVertical();
			GUILayout.BeginHorizontal();
			GUILayout.Label(string.Format("Steering: {0}", VSL.Steering), GUILayout.ExpandWidth(false));
			GUILayout.Label(string.Format("Angular Accel Error: {0:F3}rad/s2", TCA.ENG.TorqueError), GUILayout.ExpandWidth(false));
			GUILayout.Label(string.Format("Vertical Speed Factor: {0:P1}", VSL.VSF), GUILayout.ExpandWidth(false));
			GUILayout.EndHorizontal();
			eInfoScroll = GUILayout.BeginScrollView(eInfoScroll, GUILayout.Height(controlsHeight*4));
			GUILayout.BeginVertical();
			foreach(var e in VSL.ActiveEngines)
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
		#endif

		public void OnGUI()
		{
			if(TCA == null || !CFG.GUIVisible || !showHUD) 
			{
				Utils.LockIfMouseOver(LockName, MainWindow, false);
				return;
			}
			Styles.Init();
			Utils.LockIfMouseOver(LockName, MainWindow);
			MainWindow = 
				GUILayout.Window(TCA.GetInstanceID(), 
				                 MainWindow, 
								 DrawMainWindow, 
				                 VSL.vessel.vesselName,
				                 GUILayout.Width(controlsWidth),
				                 GUILayout.Height(controlsHeight));
			MainWindow.clampToScreen();
		}
	}
}