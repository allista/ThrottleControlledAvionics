//   TCAGui.cs
//
//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

using System;
using System.Linq;
using System.Collections.Generic;
using UnityEngine;

namespace ThrottleControlledAvionics
{
	public partial class ThrottleControlledAvionics
	{
		#region GUI Parameters
		static bool showHelp;
		//named configs
		static NamedConfig selected_config;
		static string config_name = string.Empty;
		static readonly DropDownList namedConfigsListBox = new DropDownList();
		//dimensions
		public const int controlsWidth = 550, controlsHeight = 100;
		public const int helpWidth = 550, helpHeight = 500;
		static Rect HelpWindow     = new Rect(Screen.width/2-helpWidth/2, 100, helpWidth, helpHeight);
		static Vector2 waypointsScroll, helpScroll;
		//keybindings
		public static KeyCode TCA_Key = KeyCode.Y;
		static bool selecting_key;
		//map view
		static bool selecting_map_target;
		static readonly ActionDamper AddTargetDamper = new ActionDamper();
		const string WPM_ICON = "ThrottleControlledAvionics/Icons/waypoint";
		const string PN_ICON  = "ThrottleControlledAvionics/Icons/path-node";
		const float  IconSize = 16;
		static Texture2D WayPointMarker, PathNodeMarker;
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
			if(TCAScenario.NamedConfigs.Count < 2) return;
			namedConfigsListBox.styleListBox  = Styles.list_box;
			namedConfigsListBox.styleListItem = Styles.list_item;
			namedConfigsListBox.windowRect    = MainWindow;
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
			if(TCAScenario.NamedConfigs.Count < 2) return;
			namedConfigsListBox.DrawDropDown();
			namedConfigsListBox.CloseOnOutsideClick();
		}
		#endregion

		protected override void DrawMainWindow(int windowID)
		{
			//help button
			if(GUI.Button(new Rect(MainWindow.width - 23f, 2f, 20f, 18f), 
			              new GUIContent("?", "Help"))) showHelp = !showHelp;
			GUILayout.BeginVertical();
			GUILayout.BeginHorizontal();
			//tca toggle
			if(GUILayout.Button(CFG.Enabled? "Disable" : "Enable", 
			                    CFG.Enabled? Styles.red_button : Styles.green_button,
			                    GUILayout.Width(70)))
				TCA.ToggleTCA();
			//change key binding
			if(GUILayout.Button(selecting_key? new GUIContent("?") : 
			                    new GUIContent(TCA_Key.ToString(), "Select TCA Hotkey"), 
			                    selecting_key? Styles.yellow_button : Styles.green_button, 
			                    GUILayout.Width(40)))
			{ selecting_key = true; ScreenMessages.PostScreenMessage("Enter new key to toggle TCA", 5, ScreenMessageStyle.UPPER_CENTER); }
			//autotune switch
			CFG.AutoTune = GUILayout.Toggle(CFG.AutoTune, "Autotune Parameters", GUILayout.ExpandWidth(true));
			#if DEBUG
			if(GUILayout.Button("Reload Globals", Styles.yellow_button, GUILayout.Width(120))) 
			{
				TCAScenario.LoadGlobals();
				TCA.OnReloadGlobals();
			}
			#endif
			StatusString();
			GUILayout.EndHorizontal();
			SelectConfig_start();
			ConfigsGUI();
			ControllerProperties();
			AutopilotControls();
			WaypointList();
			EnginesControl();
			#if DEBUG
			EnginesInfo();
			#endif
			SelectConfig_end();
			GUILayout.EndVertical();
			base.DrawMainWindow(windowID);
		}

		static void StatusString()
		{
			var state = "Disabled";
			var style = Styles.grey;
			if(TCA.IsStateSet(TCAState.Enabled))
			{
				if(TCA.IsStateSet(TCAState.ObstacleAhead))
				{ state = "Obstacle Ahead"; style = Styles.red; }
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
				else if(TCA.IsStateSet(TCAState.Landing))
				{ state = "Landing..."; style = Styles.green; }
				else if(TCA.IsStateSet(TCAState.CheckingSite))
				{ state = "Checking Landing Site"; style = Styles.yellow; }
				else if(TCA.IsStateSet(TCAState.Searching))
				{ state = "Searching For Landing Site"; style = Styles.yellow; }
				else if(TCA.IsStateSet(TCAState.Scanning))
				{ state = "Scanning Surface"; style = Styles.yellow; }
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

		static void ConfigsGUI()
		{
			GUILayout.BeginHorizontal();
			GUILayout.Label("Name:", GUILayout.Width(50));
			config_name = GUILayout.TextField(config_name, GUILayout.ExpandWidth(true), GUILayout.MinWidth(50));
			if(TCAScenario.NamedConfigs.ContainsKey(config_name))
			{
				if(GUILayout.Button("Overwrite", Styles.red_button, GUILayout.Width(70)))
					TCAScenario.SaveNamedConfig(config_name, CFG, true);
			}
			else if(GUILayout.Button("Add", Styles.green_button, GUILayout.Width(50)) && 
			        config_name != string.Empty) 
			{
				TCAScenario.SaveNamedConfig(config_name, CFG);
				updateConfigs();
				namedConfigsListBox.SelectItem(TCAScenario.NamedConfigs.IndexOfKey(config_name));
			}
			SelectConfig();
			if(GUILayout.Button("Load", Styles.yellow_button, GUILayout.Width(50)) && selected_config != null) 
				CFG.CopyFrom(selected_config);
			if(GUILayout.Button("Delete", Styles.red_button, GUILayout.Width(50)) && selected_config != null)
			{ 
				TCAScenario.NamedConfigs.Remove(selected_config.Name);
				namedConfigsListBox.SelectItem(namedConfigsListBox.SelectedIndex-1);
				updateConfigs();
				selected_config = null;
			}
			GUILayout.EndHorizontal();
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
					GUILayout.Label("Altitude: " + 
					                (VSL.Altitude.ToString("F2")+"m"), 
					                GUILayout.Width(120));
					GUILayout.Label("Set Point: " + (CFG.DesiredAltitude.ToString("F1") + "m"), 
					                GUILayout.Width(125));
					if(GUILayout.Button("-10m", Styles.normal_button, GUILayout.Width(50))) CFG.DesiredAltitude -= 10;
					if(GUILayout.Button("+10m", Styles.normal_button, GUILayout.Width(50))) CFG.DesiredAltitude += 10;
					GUILayout.Label("Vertical Speed: " + 
					                (TCA.IsStateSet(TCAState.VerticalSpeedControl)? VSL.VerticalSpeedDisp.ToString("F2")+"m/s" : "N/A"), 
					                GUILayout.Width(180));
					GUILayout.FlexibleSpace();
				}
				else
				{
					GUILayout.Label("Vertical Speed: " + 
					                (TCA.IsStateSet(TCAState.VerticalSpeedControl)? VSL.VerticalSpeedDisp.ToString("F2")+"m/s" : "N/A"), 
					                GUILayout.Width(180));
					GUILayout.Label("Set Point: " + (CFG.VerticalCutoff < GLB.VSC.MaxSpeed? 
					                                 CFG.VerticalCutoff.ToString("F1") + "m/s" : "OFF"), 
					                GUILayout.ExpandWidth(false));
					CFG.VerticalCutoff = GUILayout.HorizontalSlider(CFG.VerticalCutoff, 
					                                                -GLB.VSC.MaxSpeed, 
					                                                GLB.VSC.MaxSpeed);
				}
				GUILayout.EndHorizontal();
				//autopilot toggles
				GUILayout.BeginHorizontal();
				TCA.BlockThrottle(GUILayout.Toggle(CFG.BlockThrottle, 
				                                   CFG.VF[VFlight.AltitudeControl]?
				                                   "Change altitude with throttle controls." :
				                                   "Set vertical speed with throttle controls.", 
				                                   GUILayout.ExpandWidth(false)));
				CFG.VSControlSensitivity = Utils.FloatSlider("Sensitivity", CFG.VSControlSensitivity, 0.001f, 0.05f, "P2");
				GUILayout.EndHorizontal();
				GUILayout.BeginHorizontal();
				if(GUILayout.Button(new GUIContent("Stop", "Kill horizontal velocity"), 
				                    CFG.HF[HFlight.Stop]? Styles.green_button : Styles.yellow_button,
				                    GUILayout.Width(50)))
					CFG.HF.Toggle(HFlight.Stop);
				if(GUILayout.Button(new GUIContent("Anchor", "Hold current position"), 
				                    CFG.HF[HFlight.AnchorHere] || CFG.HF[HFlight.Anchor]? 
				                    Styles.green_button : Styles.yellow_button,
				                    GUILayout.Width(65)))
					CFG.HF.Toggle(HFlight.AnchorHere);
				if(GUILayout.Button(new GUIContent("Land", "Try to land on a nearest flat surface"), 
				                    CFG.AP[Autopilot.Land]? Styles.green_button : Styles.yellow_button,
				                    GUILayout.Width(50)))
					CFG.AP.Toggle(Autopilot.Land);
				if(GUILayout.Button(new GUIContent("Cruise", "Maintain course and speed"), 
				                    CFG.HF[HFlight.CruiseControl]? Styles.green_button : Styles.yellow_button,
				                    GUILayout.Width(65)))
					CFG.HF.Toggle(HFlight.CruiseControl);
				if(GUILayout.Button(new GUIContent("Hover", "Maintain altitude"), 
				                    CFG.VF[VFlight.AltitudeControl]? Styles.green_button : Styles.yellow_button,
				                    GUILayout.Width(60)))
					CFG.VF.Toggle(VFlight.AltitudeControl);
				TCA.AltitudeAboveTerrain(GUILayout.Toggle(CFG.AltitudeAboveTerrain, 
				                                          "Follow Terrain", 
				                                          GUILayout.ExpandWidth(false)));
				GUILayout.EndHorizontal();
				//navigator toggles
				GUILayout.BeginHorizontal();
				if(VSL.HasTarget)
				{
					if(GUILayout.Button("Go To Target", 
					                    CFG.Nav[Navigation.GoToTarget]? Styles.green_button 
					                    : Styles.yellow_button,
					                    GUILayout.Width(90)))
						CFG.Nav.On(Navigation.GoToTarget);
					if(GUILayout.Button("Follow Target", 
						CFG.Nav[Navigation.FollowTarget]? Styles.green_button 
						: Styles.yellow_button,
						GUILayout.Width(90)))
						CFG.Nav.On(Navigation.FollowTarget);
				}
				else 
				{
					GUILayout.Label("Go To Target", Styles.grey, GUILayout.Width(90));
					GUILayout.Label("Follow Target", Styles.grey, GUILayout.Width(90));
				}
				if(selecting_map_target)
				{
					if(GUILayout.Button("Cancel", Styles.red_button, GUILayout.Width(120)))
					{
						selecting_map_target = false;
						MapView.ExitMapView();
					}
				}
				else if(VSL.HasTarget && 
				        !(VSL.vessel.targetObject is WayPoint) && 
				        (CFG.Waypoints.Count == 0 || VSL.vessel.targetObject != CFG.Waypoints.Peek().GetTarget()))
				{
					if(GUILayout.Button(new GUIContent("Add As Waypoint", "Add current target as a waypoint"), 
					                    Styles.yellow_button, GUILayout.Width(120)))
					{
						CFG.Waypoints.Enqueue(new WayPoint(VSL.vessel.targetObject));
						CFG.ShowWaypoints = true;
					}
				}
				else if(GUILayout.Button(new GUIContent("Add Waypoint", "Select a new waypoint on the map"), 
				                         Styles.yellow_button, GUILayout.Width(120)))
				{
					selecting_map_target = true;
					CFG.ShowWaypoints = true;
					MapView.EnterMapView();
				}
				if(CFG.Waypoints.Count > 0)
				{
					if(GUILayout.Button("Follow Path", 
					                    CFG.Nav[Navigation.FollowPath]? Styles.green_button 
					                    : Styles.yellow_button,
					                    GUILayout.Width(90)))
						CFG.Nav.Toggle(Navigation.FollowPath);
				}
				else GUILayout.Label("Follow Path", Styles.grey, GUILayout.Width(90));
				CFG.MaxNavSpeed = Utils.FloatSlider("Max.V m/s", CFG.MaxNavSpeed, GLB.PN.MinSpeed, GLB.PN.MaxSpeed, "F0", 100);
				GUILayout.EndHorizontal();
			}
			else GUILayout.Label("Autopilot Not Available", Styles.grey, GUILayout.ExpandWidth(true));
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
				waypointsScroll = GUILayout.BeginScrollView(waypointsScroll, GUILayout.Height(controlsHeight));
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
						FlightGlobals.fetch.SetVesselTarget(wp);
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
				GUILayout.EndVertical();
				GUILayout.EndScrollView();
				GUILayout.EndVertical();
			}
			GUILayout.EndVertical();
		}

		static void EnginesControl()
		{
			GUILayout.BeginVertical();
			if(CFG.EnginesProfiles.Active.Single.Count > 0)
			{
				if(GUILayout.Button(CFG.ShowManualLimits? "Hide Manual Limits" : "Show Manual Limits", 
				                    Styles.yellow_button,
				                    GUILayout.ExpandWidth(true)))
					CFG.ShowManualLimits = !CFG.ShowManualLimits;
				if(CFG.ShowManualLimits) CFG.EnginesProfiles.DrawManual(controlsHeight);
			}
			if(GUILayout.Button(CFG.ShowEnginesProfiles? "Hide Engines Profiles" : "Show Engines Profiles", 
			                    Styles.yellow_button,
			                    GUILayout.ExpandWidth(true)))
				CFG.ShowEnginesProfiles = !CFG.ShowEnginesProfiles;
			if(CFG.ShowEnginesProfiles) CFG.EnginesProfiles.Draw(controlsHeight*2);
			GUILayout.EndVertical();
		}

		#region MapView
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
		double clicked_time;
		void MapOverlay()
		{
			if(selecting_map_target)
			{
				//stop picking on leaving map view
				selecting_map_target &= MapView.MapIsEnabled;
				if(!selecting_map_target) return;
				var coords = Utils.GetMouseCoordinates(vessel.mainBody);
				if(coords != null)
				{
					var t = new WayPoint(coords);
					DrawMapViewGroundMarker(vessel.mainBody, coords.Lat, coords.Lon, new Color(1.0f, 0.56f, 0.0f));
					GUI.Label(new Rect(Input.mousePosition.x + 15, Screen.height - Input.mousePosition.y, 200, 50), 
					          string.Format("{0} {1}\n{2}", coords, Utils.DistanceToStr(t.DistanceTo(vessel)), 
					                        ScienceUtil.GetExperimentBiome(vessel.mainBody, coords.Lat, coords.Lon)));
					if(!clicked)
					{ 
						if(Input.GetMouseButtonDown(0)) clicked = true;
						else if(Input.GetMouseButtonDown(1))  
						{ clicked_time = Planetarium.GetUniversalTime(); clicked = true; }
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
							selecting_map_target &= Planetarium.GetUniversalTime() - clicked_time >= 0.5;
							clicked = false; 
						}
					}
				}
			}
			if(MapView.MapIsEnabled && CFG.ShowWaypoints)
			{
				var i = 0;
				var num = (float)(CFG.Waypoints.Count-1);
				WayPoint wp0 = null;
				foreach(var wp in CFG.Waypoints)
				{
					wp.UpdateCoordinates(vessel.mainBody);
					var c = marker_color(i, num);
					if(wp0 == null) DrawMapViewPath(vessel, wp, c);
					else DrawMapViewPath(vessel.mainBody, wp0, wp, c);
					DrawMapViewGroundMarker(vessel.mainBody, wp.Lat, wp.Lon, c);
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

		public static void DrawMapViewGroundMarker(CelestialBody body, double lat, double lon, Color c, float r = IconSize, Texture2D texture = null)
		{
			//TODO: cache local center coordinates of the marker
			var up = body.GetSurfaceNVector(lat, lon);
			var h  = Utils.TerrainAltitude(body, lat, lon);
			if(h < body.Radius) h = body.Radius;
			var center = body.position + h * up;
			if(IsOccluded(center, body)) return;

			if(texture == null) texture = WayPointMarker;
			var icon_center = PlanetariumCamera.Camera.WorldToScreenPoint(ScaledSpace.LocalToScaledSpace(center));
			var icon_rect = new Rect(icon_center.x - r * 0.5f, (float)Screen.height - icon_center.y - r * 0.5f, r, r);
			Graphics.DrawTexture(icon_rect, texture, new Rect(0f, 0f, 1f, 1f), 0, 0, 0, 0, c, IconMaterial);
		}

		public static void DrawMapViewPath(CelestialBody body, WayPoint wp0, WayPoint wp1, Color c)
		{
			var D = wp1.AngleTo(wp0);
			var N = (int)Mathf.Clamp((float)D*Mathf.Rad2Deg, 2, 5);
			var dD = D/N;
			for(int i = 1; i<N; i++)
			{
				var p = wp0.PointBetween(wp1, dD*i);
				DrawMapViewGroundMarker(body, p.Lat, p.Lon, c, IconSize/2, PathNodeMarker);
			}
		}

		static void DrawMapViewPath(Vessel v, WayPoint wp1, Color c)
		{
			var wp0 = new WayPoint();
			wp0.Lat = v.latitude; wp0.Lon = v.longitude;
			DrawMapViewPath(v.mainBody, wp0, wp1, c);
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
			GUILayout.Label(string.Format("Torque Error: {0:F1}kNm", TCA.TorqueError), GUILayout.ExpandWidth(false));
			GUILayout.Label(string.Format("Vertical Speed Factor: {0:P1}", VSL.VSF), GUILayout.ExpandWidth(false));
			GUILayout.EndHorizontal();
			eInfoScroll = GUILayout.BeginScrollView(eInfoScroll, GUILayout.Height(controlsHeight*4));
			GUILayout.BeginVertical();
			foreach(var e in VSL.ActiveEngines)
			{
				if(!e.Valid) continue;
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

		static void windowHelp(int windowID)
		{
			GUILayout.BeginVertical();
			helpScroll = GUILayout.BeginScrollView(helpScroll);
			GUILayout.Label(GLB.Instructions, GUILayout.MaxWidth(helpWidth));
			GUILayout.EndScrollView();
			if(GUILayout.Button("Close")) showHelp = !showHelp;
			GUILayout.EndVertical();
			GUI.DragWindow();
		}

		public void OnGUI()
		{
			if(TCA == null || !TCA.Controllable || !CFG.GUIVisible || !showHUD) return;
			Styles.Init();
			MainWindow = 
				GUILayout.Window(TCA.GetInstanceID(), 
				                 MainWindow, 
								 DrawMainWindow, 
								 TCATitle,
				                 GUILayout.Width(controlsWidth),
				                 GUILayout.Height(controlsHeight));
			MainWindow.clampToScreen();
			if(showHelp) 
			{
				HelpWindow = 
					GUILayout.Window(TCA.GetInstanceID()+1, 
					                 HelpWindow, 
					                 windowHelp, 
					                 "Instructions",
					                 GUILayout.Width(helpWidth),
					                 GUILayout.Height(helpHeight));
				HelpWindow.clampToScreen();
			}
		}
	}
}