//   NavigationTab.cs
//
//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2017 Allis Tauri

using System;
using UnityEngine;
using FinePrint;
using AT_Utils;

namespace ThrottleControlledAvionics
{
	public class NavigationTab : ControlTab
	{
		public NavigationTab(ModuleTCA tca) : base(tca) 
		{
			var rnd = new System.Random();
			wp_editor_ID = rnd.Next();
		}

		#region HFlight
		HorizontalSpeedControl HSC;
		Anchor ANC;
		AutoLander LND;
		CruiseControl CC;
		BearingControl BRC;

		void HFlightControls()
		{
			GUILayout.BeginHorizontal();
			if(HSC != null)
			{
				if(Utils.ButtonSwitch("Level", CFG.HF[HFlight.Level], 
				                      "Point thrust downward", GUILayout.ExpandWidth(true)))
					TCA.SquadConfigAction(cfg => cfg.HF.XToggle(HFlight.Level));
			}
			if(CC != null)
			{
				if(Utils.ButtonSwitch("Cruise", CFG.HF[HFlight.CruiseControl], 
				                      "Maintain course and speed", GUILayout.ExpandWidth(true)))
				{
					CFG.HF.XToggle(HFlight.CruiseControl);
					if(CFG.HF[HFlight.CruiseControl]) follow_me();
				}
			}
			if(BRC != null) BRC.Draw();
			GUILayout.EndHorizontal();
			GUILayout.BeginHorizontal();
			if(HSC != null)
			{
                #if DEBUG
				HSC.DrawDebugLines();
                #endif
				if(Utils.ButtonSwitch("Stop", CFG.HF[HFlight.Stop], 
				                      "Kill horizontal velocity", GUILayout.ExpandWidth(true)))
				{
					if(CFG.HF[HFlight.Stop]) TCA.SquadConfigAction(cfg => cfg.HF.OffIfOn(HFlight.Stop));
					else TCA.SquadConfigAction(cfg => { cfg.HF.XOn(HFlight.Stop); cfg.StopMacro(); });
				}
			}
			if(ANC != null)
			{
				if(Utils.ButtonSwitch("Anchor", CFG.Nav.Any(Navigation.AnchorHere, Navigation.Anchor), 
				                     "Hold current position", GUILayout.ExpandWidth(true)))
					TCA.SquadConfigAction(cfg => cfg.Nav.XToggle(Navigation.AnchorHere));
			}
			if(LND != null)
			{
                #if DEBUG
				LND.DrawDebugLines();
                #endif
				if(Utils.ButtonSwitch("Land", CFG.AP1[Autopilot1.Land], 
				                      "Try to land on a nearest flat surface", GUILayout.ExpandWidth(true)))
				{
					var state = !CFG.AP1[Autopilot1.Land];
					if(state) { follow_me(); CFG.AP1.XOn(Autopilot1.Land); }
					else TCA.SquadConfigAction(cfg => cfg.AP1.XOffIfOn(Autopilot1.Land));
				}
			}
			GUILayout.EndHorizontal();
		}
		#endregion

		#region Navigation
		Vessel vessel { get { return TCA.vessel; } }

		PointNavigator PN;
		BallisticJump BJ;

		public bool SelectingTarget { get; private set; }
		bool select_single;

		public static void OnAwake()
		{
			WayPointMarker = GameDatabase.Instance.GetTexture(Globals.RADIATION_ICON, false);
			PathNodeMarker = GameDatabase.Instance.GetTexture(Globals.CIRCLE_ICON, false);
		}

		void NavigationControls()
		{
			GUILayout.BeginHorizontal();
			if(BJ != null) BJ.Draw();
			if(PN != null)
			{
				if(VSL.HasTarget && !CFG.Nav.Paused)
				{
					if(Utils.ButtonSwitch("Go To", CFG.Nav[Navigation.GoToTarget],
					                      "Fly to current target", GUILayout.ExpandWidth(true)))
					{
                        VSL.Engines.ActivateEnginesAndRun(() => 
                        {
                            CFG.Nav.XOn(Navigation.GoToTarget);
                            if(CFG.Nav[Navigation.GoToTarget]) follow_me();    
                        });
					}
					if(Utils.ButtonSwitch("Follow", CFG.Nav[Navigation.FollowTarget], 
					                      "Follow current target", GUILayout.ExpandWidth(true)))
                        VSL.Engines.ActivateEnginesAndRun(() => TCA.SquadAction(tca =>
                    {
                        if(TCA.vessel.targetObject as Vessel == tca.vessel) return;
                        tca.vessel.targetObject = TCA.vessel.targetObject;
                        tca.CFG.Nav.XOn(Navigation.FollowTarget);
                    }));
                }
				else 
				{
					GUILayout.Label(new GUIContent("Go To", CFG.Nav.Paused? "Paused" : "No target selected"),  
					                Styles.inactive_button, GUILayout.ExpandWidth(true));
					GUILayout.Label(new GUIContent("Follow", CFG.Nav.Paused? "Paused" : "No target selected"),  
					                Styles.inactive_button, GUILayout.ExpandWidth(true));
				}
			}
			GUILayout.EndHorizontal();
            if(BJ != null && BJ.ShowOptions)
				BJ.DrawOptions();
			GUILayout.BeginHorizontal();
			if(SQD != null && SQD.SquadMode)
			{
				if(CFG.Nav.Paused)
					GUILayout.Label(new GUIContent("Follow Me", "Make the squadron follow"),  
					                Styles.inactive_button, GUILayout.ExpandWidth(true));
				else if(GUILayout.Button(new GUIContent("Follow Me", "Make the squadron follow"), 
				                         Styles.active_button, GUILayout.ExpandWidth(true)))
					follow_me();
			}
			if(PN != null)
			{
				if(SelectingTarget)
					SelectingTarget &= !GUILayout.Button("Cancel", Styles.close_button, GUILayout.Width(120));
                else if(!UI.RemoteControl &&
                        VSL.HasTarget && 
				        (VSL.TargetIsNavPoint ||
				         !VSL.TargetIsWayPoint && 
				         (CFG.Path.Count == 0 || VSL.Target != CFG.Path.Peek().GetTarget())))
				{
					if(GUILayout.Button(new GUIContent("Add As Waypoint", "Add current target as a waypoint"), 
					                    Styles.active_button, GUILayout.Width(120)))
					{
						var t = VSL.TargetAsWP;
						CFG.Path.Enqueue(t);
						CFG.ShowPath = true;
					}
				}
				else if(GUILayout.Button(new GUIContent("Add Waypoint", "Select a new waypoint"), 
				                         Styles.active_button, GUILayout.Width(120)))
				{
					SelectingTarget = true;
					CFG.ShowPath = true;
				}
				if(CFG.Path.Count > 0 && !CFG.Nav.Paused)
				{
					if(Utils.ButtonSwitch("Follow Route", CFG.Nav[Navigation.FollowPath], "", GUILayout.ExpandWidth(true)))
					{
                        VSL.Engines.ActivateEnginesAndRun(() => 
                        {
                            CFG.Nav.XToggle(Navigation.FollowPath);
                            if(CFG.Nav[Navigation.FollowPath]) follow_me();
                        });
					}
				}
				else GUILayout.Label(new GUIContent("Follow Route", CFG.Nav.Paused? "Paused" : "Add some waypoints first"), 
				                     Styles.inactive_button, GUILayout.ExpandWidth(true));
			}
			GUILayout.EndHorizontal();
			GUILayout.BeginHorizontal();
			if(PN != null || CC != null)
			{
				var max_nav_speed = Utils.FloatSlider("", CFG.MaxNavSpeed, 
				                                      CFG.HF[HFlight.CruiseControl]? GLB.CC.MaxRevSpeed : GLB.PN.MinSpeed, GLB.PN.MaxSpeed, 
				                                      "0.0 m/s", 60, "Maximum horizontal speed on autopilot");
				if(Mathf.Abs(max_nav_speed-CFG.MaxNavSpeed) > 1e-5)
					TCA.SquadConfigAction(cfg => cfg.MaxNavSpeed = max_nav_speed);
			}
			GUILayout.EndHorizontal();
		}

		#region WaypointList
		Vector2 waypointsScroll, pathsScroll, stockScroll;
		string path_name = "";
		bool show_path_library;
		bool show_stock_waypoints;
        bool was_in_map_view;

		public void TargetUI()
		{
			if(SelectingTarget)
				SelectingTarget &= !GUILayout.Button("Cancel", Styles.close_button, GUILayout.ExpandWidth(true));
            else if(CFG.Target)
			{
                GUILayout.BeginHorizontal();
                if(GUILayout.Button(new GUIContent("Edit Target", "Edit target point"), 
                                    Styles.active_button, GUILayout.ExpandWidth(true)))
                {
                    if(!CFG.Target.IsMovable)
                        VSL.UpdateTarget(CFG.Target.CopyMovable());
                    edit_waypoint(CFG.Target);
                }
                if(VSL.TargetUsers.Count > 0)
					GUILayout.Label(new GUIContent("Del Target", "Target point is in use"),
					                Styles.grey_button, GUILayout.ExpandWidth(true));
				else if(GUILayout.Button(new GUIContent("Del Target", "Remove target point"), 
				                         Styles.danger_button, GUILayout.ExpandWidth(true)))
					VSL.SetTarget(null);
                GUILayout.EndHorizontal();
			}
            else if(GUILayout.Button(new GUIContent("Set Surface Target", "Select target point on the surface"), 
                                     Styles.active_button, GUILayout.ExpandWidth(true)))
                SetSurfaceTarget();
		}

        public void SetSurfaceTarget()
        {
            was_in_map_view = MapView.MapIsEnabled;
            select_single = true;
            SelectingTarget = true;
            CFG.GUIVisible = true;
            CFG.ShowPath = true;
            MapView.EnterMapView();
        }

		void path_library()
		{
			if(show_path_library)
			{
				GUILayout.BeginVertical(Styles.white);
				pathsScroll = GUILayout
					.BeginScrollView(pathsScroll, 
					                 GUILayout.Height(Utils.ClampH(TCAGui.LineHeight*(TCAScenario.Paths.Count+1), 
					                                               TCAGui.ControlsHeightHalf)));
				var selected_path = "";
				var delete_path = "";
				foreach(var path in TCAScenario.Paths.Names)
				{
					GUILayout.BeginHorizontal();
					if(GUILayout.Button(new GUIContent(path, "Click to load this path into ship's computer"), 
					                    Styles.boxed_label, GUILayout.ExpandWidth(true)))
						selected_path = path;
					if(GUILayout.Button(new GUIContent("X", "Delete this path"), 
					                    Styles.danger_button, GUILayout.Width(25))) 
						delete_path = path;
					GUILayout.EndHorizontal();
				}
				GUILayout.EndScrollView();
				if(!string.IsNullOrEmpty(selected_path)) 
				{
					CFG.Path = TCAScenario.Paths.GetPath(selected_path);
					path_name = CFG.Path.Name;
					show_path_library = false;
				}
				else if(!string.IsNullOrEmpty(delete_path))
				{
					TCAScenario.Paths.Remove(delete_path);
					show_path_library &= !TCAScenario.Paths.Empty;
				}
				GUILayout.EndVertical();
			}
		}

		void stock_waypoints(WaypointManager WPM)
		{
			if(show_stock_waypoints && WPM != null)
			{
				GUILayout.BeginVertical(Styles.white);
				stockScroll = GUILayout
					.BeginScrollView(stockScroll, 
					                 GUILayout.Height(Utils.ClampH(TCAGui.LineHeight*(TCAScenario.Paths.Count+1), 
					                                               TCAGui.ControlsHeightHalf)));
				var displayed = 0;
				Waypoint selected = null;
				foreach(var wp in WPM.Waypoints)
				{
					if(!wp.isNavigatable || wp.celestialBody != vessel.mainBody) continue;
					if(GUILayout.Button(new GUIContent(wp.FullName, "Click to add this waypoint into the path"), 
					                    Styles.boxed_label, GUILayout.ExpandWidth(true)))
						selected = wp;
					displayed++;
				}
				if(displayed == 0)
					GUILayout.Label("No waypoints on this planet.", Styles.boxed_label, GUILayout.ExpandWidth(true));
				GUILayout.EndScrollView();
				if(displayed > 1 && 
				   GUILayout.Button("Add All", Styles.enabled_button, GUILayout.ExpandWidth(true)))
				{
					WaypointManager.Instance().Waypoints.ForEach(wp => CFG.Path.Enqueue(new WayPoint(wp)));
					show_stock_waypoints = false;
				}
				if(selected != null)
					CFG.Path.Enqueue(new WayPoint(selected));
				GUILayout.EndVertical();
			}
		}

		public void WaypointList()
		{
			if(PN == null) return;
			var WPM = WaypointManager.Instance();
			if(CFG.Path.Count == 0) 
			{
				GUILayout.BeginHorizontal();
				if(TCAScenario.Paths.Count > 0)
					Utils.ButtonSwitch("Navigation Paths", ref show_path_library, "", GUILayout.ExpandWidth(true));
				if(WPM != null && WPM.Waypoints.Count > 0)
					Utils.ButtonSwitch("Contract Waypoints", ref show_stock_waypoints, "", GUILayout.ExpandWidth(true));
				GUILayout.EndHorizontal();
				stock_waypoints(WPM);
				path_library();
				return;
			}
			GUILayout.BeginVertical();
			GUILayout.BeginHorizontal();
			if(GUILayout.Button(CFG.ShowPath? "Hide Waypoints" : "Show Waypoints", 
			                    Styles.active_button,
			                    GUILayout.ExpandWidth(true)))
				CFG.ShowPath = !CFG.ShowPath;
			if(TCAScenario.Paths.Empty)
				GUILayout.Label("Load Path", Styles.inactive_button, GUILayout.ExpandWidth(false));
			else Utils.ButtonSwitch("Load Path", ref show_path_library, "", GUILayout.ExpandWidth(false));
			if(WPM == null || WPM.Waypoints.Count == 0)
				GUILayout.Label("Add From Contracts", Styles.inactive_button, GUILayout.ExpandWidth(false));
			else Utils.ButtonSwitch("Add From Contracts", ref show_stock_waypoints, "", GUILayout.ExpandWidth(false));
			GUILayout.EndHorizontal();
			stock_waypoints(WPM);
			path_library();
			if(CFG.ShowPath)
			{
				GUILayout.BeginVertical(Styles.white);
				waypointsScroll = GUILayout
					.BeginScrollView(waypointsScroll, 
					                 GUILayout.Height(Utils.ClampH(TCAGui.LineHeight*(CFG.Path.Count+1), 
					                                               TCAGui.ControlsHeightHalf)));
				GUILayout.BeginVertical();
				int i = 0;
				var num = (float)(CFG.Path.Count-1);
				var col = GUI.contentColor;
				WayPoint del = null;
				WayPoint up = null;
				foreach(var wp in CFG.Path)
				{
					GUI.contentColor = marker_color(i, num);
					var label = string.Format("{0}) {1}", 1+i, wp.GetName());
					if(wp == edited_waypoint) label += " *";
					if(CFG.Target == wp)
					{
						var hd = (float)wp.DistanceTo(vessel);
						var vd = (float)(wp.Pos.Alt-vessel.altitude);
						var info = string.Format("Distance: ◀ {0} ▲ {1}", 
						                         Utils.formatBigValue(hd, "m"),
						                         Utils.formatBigValue(vd, "m"));
						if(VSL.HorizontalSpeed.Absolute > 0.1)
							info += string.Format(", ETA {0:c}", new TimeSpan(0,0,(int)(hd/VSL.HorizontalSpeed.Absolute)));
						GUILayout.Label(info, Styles.white, GUILayout.ExpandWidth(true));
					}
					GUILayout.BeginHorizontal();
					if(GUILayout.Button(new GUIContent(label, string.Format("{0}\nPush to target this waypoint", wp.SurfaceDescription(vessel))), 
					                    GUILayout.ExpandWidth(true)))
                        VSL.SetTarget(null, wp);
					GUI.contentColor = col;
					GUILayout.FlexibleSpace();
					if(GUILayout.Button("Edit", Styles.normal_button))
						edit_waypoint(wp);
					if(GUILayout.Button(new GUIContent("^", "Move up"), Styles.normal_button))
						up = wp;
					if(LND != null && 
					   Utils.ButtonSwitch("Land", wp.Land, "Land on arrival"))
						wp.Land = !wp.Land;
					if(Utils.ButtonSwitch("||", wp.Pause, "Pause on arrival", GUILayout.Width(25))) 
						wp.Pause = !wp.Pause;
					if(GUILayout.Button(new GUIContent("X", "Delete waypoint"), 
					                    Styles.danger_button, GUILayout.Width(25))) 
						del = wp;
					GUILayout.EndHorizontal();
					i++;
				}
				GUI.contentColor = col;
				if(del != null) CFG.Path.Remove(del);
				else if(up != null) CFG.Path.MoveUp(up);
				if(CFG.Path.Count == 0 && CFG.Nav) CFG.HF.XOn(HFlight.Stop);
				GUILayout.EndVertical();
				GUILayout.EndScrollView();
				GUILayout.BeginHorizontal();
				path_name = GUILayout.TextField(path_name, GUILayout.ExpandWidth(true));
				if(string.IsNullOrEmpty(path_name))
					GUILayout.Label("Save Path", Styles.inactive_button, GUILayout.Width(70));
				else
				{
					var existing = TCAScenario.Paths.Contains(path_name);
					if(GUILayout.Button(existing? "Overwrite" : "Save Path",
					                    existing? Styles.danger_button : Styles.enabled_button,
					                    GUILayout.Width(70)))
					{
						CFG.Path.Name = path_name;
						TCAScenario.Paths.SavePath(CFG.Path);
					}
				}
				if(GUILayout.Button("Clear Path", Styles.danger_button, GUILayout.ExpandWidth(false)))
					CFG.Path.Clear();
				GUILayout.EndHorizontal();
				GUILayout.EndVertical();
			}
			GUILayout.EndVertical();
		}
		#endregion

		#region WaypointEditor
		const int wp_edit_width = 350;
		const int wp_edit_height = 100;
		static Rect wp_editor_pos = new Rect((Screen.width-wp_edit_width)/2, 
		                                     (Screen.height-wp_edit_height)/2, 
		                                     wp_edit_width, wp_edit_height);
		readonly int wp_editor_ID;
		FloatField LatField = new FloatField(min: -90, max: 90);
		FloatField LonField = new FloatField(min: -180, max: 180);
		FloatField AltField = new FloatField();
		static Color edited_color = new Color(177f/255, 0, 1);

		WayPoint edited_waypoint;
		string edited_waypoint_name = "";

		void edit_waypoint(WayPoint wp)
		{
			if(!wp.IsMovable) 
			{
				Utils.Message("{0} cannot be edited.", wp.Name);
				return;
			}
			edited_waypoint = wp;
			edited_waypoint_name = wp.Name;
			LatField.Value = Utils.CenterAngle((float)wp.Pos.Lat);
			LonField.Value = Utils.CenterAngle((float)wp.Pos.Lon);
			AltField.Value = (float)wp.Pos.Alt;
		}

		void waypoint_editor(int windowID)
		{
			var close = false;
			GUILayout.BeginVertical();
			GUILayout.BeginHorizontal();
			GUILayout.Label("Name:", GUILayout.Width(70));
			edited_waypoint_name = GUILayout.TextField(edited_waypoint_name, GUILayout.ExpandWidth(true));
			GUILayout.EndHorizontal();
			GUILayout.BeginHorizontal();
			GUILayout.Label("Latitude:", GUILayout.Width(70));
			LatField.Draw("°", 1, "F1");
			GUILayout.EndHorizontal();
			GUILayout.BeginHorizontal();
			GUILayout.Label("Longitude:", GUILayout.Width(70));
			LonField.Draw("°", 1, "F1");
			GUILayout.EndHorizontal();
			GUILayout.BeginHorizontal();
			GUILayout.Label("Altitude:", GUILayout.Width(70));
			AltField.Draw("°", 100, "F0");
			GUILayout.EndHorizontal();
			GUILayout.BeginHorizontal();
			if(GUILayout.Button("Delete", Styles.danger_button))
			{
				CFG.Path.Remove(edited_waypoint);
				close = true;
			}
			GUILayout.FlexibleSpace();
			if(GUILayout.Button("Cancel", Styles.active_button)) close = true;
			GUILayout.FlexibleSpace();
            if(GUILayout.Button(new GUIContent("◉", "Target this waypoint"), Styles.enabled_button))
                VSL.SetTarget(null, edited_waypoint);
            if(GUILayout.Button(new GUIContent("⊥", "Set altitude to ground level"), Styles.active_button))
            {
                edited_waypoint.Pos.SetAlt2Surface(VSL.Body);
                AltField.Value = (float)edited_waypoint.Pos.Alt;
            }
            if(Utils.ButtonSwitch("||", edited_waypoint.Pause, "Pause on arrival", GUILayout.Width(25))) 
                edited_waypoint.Pause = !edited_waypoint.Pause;
            if(LND != null && 
			   Utils.ButtonSwitch("Land", edited_waypoint.Land, "Land on arrival"))
				edited_waypoint.Land = !edited_waypoint.Land;
			if(GUILayout.Button("Apply", Styles.confirm_button))
			{
				LatField.UpdateValue(); LonField.UpdateValue(); AltField.UpdateValue();
				edited_waypoint.Name = edited_waypoint_name;
				edited_waypoint.Pos.Lat = Utils.ClampAngle(LatField.Value);
				edited_waypoint.Pos.Lon = Utils.ClampAngle(LonField.Value);
				edited_waypoint.Pos.Alt = Math.Max(AltField.Value, 
				                                   edited_waypoint.Pos.SurfaceAlt(vessel.mainBody));
				edited_waypoint.Update(VSL);
				close = true;
			}
			GUILayout.EndHorizontal();
			GUILayout.EndVertical();
			GUIWindowBase.TooltipsAndDragWindow();
			if(close) edited_waypoint = null;
		}

		public void WaypointEditorWindow()
		{
			if(edited_waypoint == null) 
			{
				Utils.LockIfMouseOver("TCAWaypointManager", wp_editor_pos, false);
				return;
			}
			Utils.LockIfMouseOver("TCAWaypointManager", wp_editor_pos);
			wp_editor_pos = 
				GUILayout.Window(wp_editor_ID, 
				                 wp_editor_pos, 
				                 waypoint_editor, 
				                 "Edit Waypoint",
				                 GUILayout.Width(wp_edit_width),
				                 GUILayout.Height(wp_edit_height)).clampToScreen();
		}
		#endregion

		#region Waypoints Overlay
		readonly ActionDamper AddTargetDamper = new ActionDamper();
		public static Texture2D WayPointMarker, PathNodeMarker;

		WayPoint selected_waypoint;
		Coordinates orig_coordinates;
		bool changing_altitude;
		float last_mouse_y;

		static float marker_alpha(float dist)
		{
			if(!MapView.MapIsEnabled && dist > 0)
				return Mathf.Sqrt(1-Mathf.Min(dist, GLB.WaypointFadoutDist)/GLB.WaypointFadoutDist);
			return 1;
		}

		static Color marker_color(int i, float N, float dist = -1)
		{ 
			var c = Color.red;
			if(N > 0)
			{
				var t = i/N;
				c = t < 0.5f ? 
					Color.Lerp(Color.red, Color.green, t*2).Normalized() : 
					Color.Lerp(Color.green, Color.cyan, (t-0.5f)*2).Normalized(); 
			}
			c.a = marker_alpha(dist);
			return c;
		}

		static void DrawLabelAtPointer(string text, double distance)
		{ Markers.DrawLabelAtPointer(string.Format("{0}\nDistance: {1}", text, Utils.formatBigValue((float)distance, "m"))); }

		void select_waypoint(WayPoint wp)
		{
			if(SelectingTarget || 
			   selected_waypoint != null) return;
			if(Input.GetMouseButtonDown(0) || Input.GetMouseButtonDown(2)) 
			{
				if(!wp.IsMovable)
					Utils.Message("{0} is not movable.", wp.Name);
				else
				{
					selected_waypoint = wp;
					orig_coordinates = wp.Pos.Copy();
					changing_altitude = Input.GetMouseButtonDown(2);
					if(changing_altitude) last_mouse_y = Input.mousePosition.y;
				}
			}
			else if(Input.GetMouseButtonDown(1))
				edit_waypoint(wp);
		}

		//adapted from MechJeb
		bool clicked;
		DateTime clicked_time;
		public void WaypointOverlay()
		{
			if(PN == null || TCA == null || !TCA.Available || !GUIWindowBase.HUD_enabled) return;
			if(SelectingTarget)
			{
				var coords = MapView.MapIsEnabled? 
					Coordinates.GetAtPointer(vessel.mainBody) :
					Coordinates.GetAtPointerInFlight();
				if(coords != null)
				{
					var t = new WayPoint(coords);
					Markers.DrawCBMarker(vessel.mainBody, coords, new Color(1.0f, 0.56f, 0.0f), WayPointMarker);
					DrawLabelAtPointer(coords.FullDescription(vessel), t.DistanceTo(vessel));
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
							t.Update(VSL);
							t.Movable = true;
							if(select_single)
							{
								t.Name = "Target";
								SelectingTarget = false;
								select_single = false;
								VSL.SetTarget(null, t);
                                if(!was_in_map_view)
                                    MapView.ExitMapView();
							}
							else 
							{
								t.Name = "Waypoint "+(CFG.Path.Count+1);
								AddTargetDamper.Run(() => CFG.Path.Enqueue(t));
							}
							CFG.ShowPath = true;
							clicked = false;
						}
						if(Input.GetMouseButtonUp(1))
						{ 
							SelectingTarget &= (DateTime.Now - clicked_time).TotalSeconds >= GLB.ClickDuration;
							clicked = false; 
						}
					}
				}
			}
			bool current_target_drawn = false;
			var camera = Markers.CurrentCamera;
			if(CFG.ShowPath)
			{
				var i = 0;
				var num = (float)(CFG.Path.Count-1);
				WayPoint wp0 = null;
				var total_dist = 0f;
				var dist2cameraF = Mathf.Pow(Mathf.Max((camera.transform.position - 
				                                        VSL.vessel.transform.position).magnitude, 1), 
				                             GLB.CameraFadeinPower);
				foreach(var wp in CFG.Path)
				{
					current_target_drawn |= wp.Equals(CFG.Target);
					wp.UpdateCoordinates(vessel.mainBody);
					var dist = -1f;
					if(wp0 != null && wp != selected_waypoint)
					{
						total_dist += (float)wp.DistanceTo(wp0, VSL.Body)/dist2cameraF;
						dist = total_dist;
					}
					var r = Markers.DefaultIconSize;
					var c = marker_color(i, num, dist);
					if(wp0 == null) DrawPath(vessel, wp, c);
					else DrawPath(vessel.mainBody, wp0, wp, c);
					if(wp == edited_waypoint) 
					{ c = edited_color; r = Markers.DefaultIconSize*2; }
					else if(wp.Land) r = Markers.DefaultIconSize*2;
					if(DrawWayPoint(wp, c, size:r))
					{
						DrawLabelAtPointer(wp.FullInfo(vessel), wp.DistanceTo(vessel));
						select_waypoint(wp);
					}
					else if(wp == selected_waypoint)
						DrawLabelAtPointer(wp.FullInfo(vessel), wp.DistanceTo(vessel));
					wp0 = wp; 
					i++;
				}
			}
			//current target and anchor
            if(CFG.Anchor != null) 
			{
				DrawWayPoint(CFG.Anchor, Color.cyan, "Anchor");
				current_target_drawn |= CFG.Anchor.Equals(CFG.Target);
			}
            if(CFG.Target && !current_target_drawn && 
			   (!CFG.Target.IsVessel || CFG.Target.GetVessel().LandedOrSplashed))
				DrawWayPoint(CFG.Target, Color.magenta, "Target");
			//custom markers
			VSL.Info.CustomMarkersWP.ForEach(m => DrawWayPoint(m, Color.red, m.Name));
			VSL.Info.CustomMarkersVec.ForEach(m => Markers.DrawWorldMarker(m, Color.red, "Custom WayPoint", WayPointMarker));
			//modify the selected waypoint
			if(!SelectingTarget && selected_waypoint != null)
			{
				if(changing_altitude)
				{
					var dist2camera = (selected_waypoint.GetTransform().position-camera.transform.position).magnitude;
					var dy = (Input.mousePosition.y-last_mouse_y)/Screen.height *
						dist2camera*Math.Tan(camera.fieldOfView*Mathf.Deg2Rad)*2;
					last_mouse_y = Input.mousePosition.y;
					selected_waypoint.Pos.Alt += dy;
					selected_waypoint.Update(VSL);
				}
				else
				{
					var coords = MapView.MapIsEnabled? 
						Coordinates.GetAtPointer(vessel.mainBody) :
						Coordinates.GetAtPointerInFlight();
					if(coords != null) 
					{ 
						selected_waypoint.Pos = coords;
						selected_waypoint.Update(VSL);
					}
				}
				if(Input.GetMouseButtonUp(0) ||
				   (changing_altitude && Input.GetMouseButtonUp(2)))
				{ 
					if(changing_altitude)
						selected_waypoint.Pos.Alt = Math.Max(selected_waypoint.Pos.Alt, 
						                                     selected_waypoint.Pos.SurfaceAlt(vessel.mainBody));
					selected_waypoint = null;
					orig_coordinates = null;
					changing_altitude = false;
				}
				else if(Input.GetMouseButtonDown(1))
				{ 
					selected_waypoint.Pos = orig_coordinates;
					selected_waypoint.Update(VSL);
					selected_waypoint = null;
					orig_coordinates = null;
					changing_altitude = false;
				}
			}
			#if DEBUG
			//			VSL.Engines.All.ForEach(e => e.engine.thrustTransforms.ForEach(t => DrawWorldMarker(t.position, Color.red, e.name)));
			//			DrawWorldMarker(VSL.vessel.transform.position, Color.yellow, "Vessel");
			//			DrawWorldMarker(VSL.Physics.wCoM, Color.green, "CoM");
			#endif
		}

		public bool DrawWayPoint(WayPoint wp, Color c, string label = null, float size = Markers.DefaultIconSize)
		{
			Vector3d worldPos;
			if(Markers.DrawCBMarker(vessel.mainBody, wp.Pos, c, out worldPos, WayPointMarker, size))
			{
				DrawLabelAtPointer(label ?? wp.FullInfo(vessel), wp.DistanceTo(vessel));
				select_waypoint(wp);
				return true;
			}
			return false;
		}

		static readonly float path_point_size = Markers.DefaultIconSize/2;
		public static void DrawPath(CelestialBody body, WayPoint wp0, WayPoint wp1, Color c, float alpha = -1)
		{
			if(alpha >= 0) c.a = alpha;
			if(c.a.Equals(0)) return;
			var D = wp1.AngleTo(wp0);
			var N = (int)Mathf.Clamp((float)D*Mathf.Rad2Deg, 1, 5);
			var dD = D/N;
			var last_point = wp0.WorldPos(body);
			Vector3d point;
			Color line_color = c;
			line_color.a /= 2;
			for(int i = 1; i<N; i++)
			{
				var p = wp0.PointBetween(wp1, dD*i);
				p.SetAlt2Surface(body);
				Markers.DrawCBMarker(body, p, c, out point, PathNodeMarker, path_point_size);
				Utils.GLLine(last_point, point, line_color);
				last_point = point;
			}
			Utils.GLLine(last_point, wp1.WorldPos(body), line_color);
		}

		public static void DrawPath(Vessel v, WayPoint wp1, Color c)
		{ DrawPath(v.mainBody, new WayPoint(v.latitude, v.longitude, v.altitude), wp1, c, 1); }
		#endregion
		#endregion


		public void DrawWaypoints()
		{
			WaypointEditorWindow();
			if(Event.current.type == EventType.Repaint)
				WaypointOverlay();
		}

		public override void Draw()
		{
			if(!VSL.OnPlanet) 
            {
                GUILayout.BeginVertical();
				GUILayout.Label("Controls unavailable in orbit", Styles.yellow, GUILayout.ExpandWidth(true));
                WaypointList();
                GUILayout.EndVertical();
            }
			else
			{
				HFlightControls();
				NavigationControls();
                WaypointList();
			}
		}
	}
}

