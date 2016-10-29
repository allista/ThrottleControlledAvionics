//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2016 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.
//
using System;
using System.Linq;
using System.Collections.Generic;
using UnityEngine;
using AT_Utils;

namespace ThrottleControlledAvionics
{
	public class NavigationPanel : ControlPanel
	{
		Vessel vessel { get { return TCA.vessel; } }

		public NavigationPanel(ModuleTCA tca) : base(tca) 
		{
			var rnd = new System.Random();
			wp_editor_ID = rnd.Next();
		}

		AutoLander LND;
		PointNavigator PN;
		BallisticJump BJ;

		static Camera current_camera 
		{ get { return MapView.MapIsEnabled? PlanetariumCamera.Camera : FlightCamera.fetch.mainCamera; } }

		Vector2 waypointsScroll;
		public bool SelectingTarget { get; private set; }
		bool select_single;

		public static void OnAwake()
		{
			WayPointMarker = GameDatabase.Instance.GetTexture(WPM_ICON, false);
			PathNodeMarker = GameDatabase.Instance.GetTexture(PN_ICON, false);
		}

		public override void Draw()
		{
			if(PN == null || !VSL.OnPlanet) return;
			GUILayout.BeginHorizontal();
			if(BJ != null) BJ.Draw();
			if(VSL.HasTarget && !CFG.Nav.Paused)
			{
				if(Utils.ButtonSwitch("Go To", CFG.Nav[Navigation.GoToTarget], "Fly to current target", GUILayout.Width(50)))
				{
					CFG.Nav.XOn(Navigation.GoToTarget);
					if(CFG.Nav[Navigation.GoToTarget]) follow_me();
				}
				if(Utils.ButtonSwitch("Follow", CFG.Nav[Navigation.FollowTarget], "Follow current target", GUILayout.Width(50)))
					apply(tca => 
				{
					if(TCA.vessel.targetObject as Vessel == tca.vessel) return;
					tca.vessel.targetObject = TCA.vessel.targetObject;
					tca.CFG.Nav.XOn(Navigation.FollowTarget);
				});
			}
			else 
			{
				GUILayout.Label(new GUIContent("Go To", CFG.Nav.Paused? "Paused" : "No target selected"),  
				                Styles.inactive_button, GUILayout.Width(50));
				GUILayout.Label(new GUIContent("Follow", CFG.Nav.Paused? "Paused" : "No target selected"),  
				                Styles.inactive_button, GUILayout.Width(50));
			}
			if(SQD != null && SQD.SquadMode)
			{
				if(CFG.Nav.Paused)
					GUILayout.Label(new GUIContent("Follow Me", "Make the squadron follow"),  
					                Styles.inactive_button, GUILayout.Width(75));
				else if(GUILayout.Button(new GUIContent("Follow Me", "Make the squadron follow"), 
				                         Styles.active_button, GUILayout.Width(75)))
					follow_me();
			}
			if(SelectingTarget)
				SelectingTarget &= !GUILayout.Button("Cancel", Styles.close_button, GUILayout.Width(120));
			else if(VSL.HasTarget && 
			        (VSL.TargetIsNavPoint ||
			        !VSL.TargetIsWayPoint && 
			         (CFG.Waypoints.Count == 0 || VSL.Target != CFG.Waypoints.Peek().GetTarget())))
			{
				if(GUILayout.Button(new GUIContent("Add As Waypoint", "Add current target as a waypoint"), 
				                    Styles.active_button, GUILayout.Width(120)))
				{
					var t = VSL.TargetAsWP;
					VSL.SetTarget(t);
					CFG.Waypoints.Enqueue(t);
					CFG.ShowWaypoints = true;
				}
			}
			else if(GUILayout.Button(new GUIContent("Add Waypoint", "Select a new waypoint"), 
			                         Styles.active_button, GUILayout.Width(120)))
			{
				SelectingTarget = true;
				CFG.ShowWaypoints = true;
			}
			if(CFG.Waypoints.Count > 0 && !CFG.Nav.Paused)
			{
				if(Utils.ButtonSwitch("Follow Route", CFG.Nav[Navigation.FollowPath], "", GUILayout.Width(90)))
				{
					CFG.Nav.XToggle(Navigation.FollowPath);
					if(CFG.Nav[Navigation.FollowPath])
						follow_me();
				}
			}
			else GUILayout.Label(new GUIContent("Follow Route", CFG.Nav.Paused? "Paused" : "Add some waypoints first"), 
			                     Styles.inactive_button, GUILayout.Width(90));
			var max_nav_speed = Utils.FloatSlider("", CFG.MaxNavSpeed, 
			                                      CFG.HF[HFlight.CruiseControl]? GLB.CC.MaxRevSpeed : GLB.PN.MinSpeed, GLB.PN.MaxSpeed, 
			                                      "0.0 m/s", 60, "Maximum horizontal speed on autopilot");
			if(Mathf.Abs(max_nav_speed-CFG.MaxNavSpeed) > 1e-5)
				apply_cfg(cfg => cfg.MaxNavSpeed = max_nav_speed);
			GUILayout.EndHorizontal();
		}

		public void AddSingleWaypointInMapView()
		{
			if(SelectingTarget)
				SelectingTarget &= !GUILayout.Button("Cancel", Styles.close_button, GUILayout.ExpandWidth(false));
			else if(CFG.Target != null)
			{
				if(CFG.AP2 || CFG.AP1 || CFG.Nav)
					GUILayout.Label(new GUIContent("Del Target", "Target point is in use"),
					                Styles.grey_button, GUILayout.ExpandWidth(false));
				else if(GUILayout.Button(new GUIContent("Del Target", "Remove target point"), 
				                         Styles.danger_button, GUILayout.ExpandWidth(false)))
					VSL.SetTarget();
			}
			else if(GUILayout.Button(new GUIContent("Set Target", "Select target point"), 
			                         Styles.active_button, GUILayout.ExpandWidth(false)))
			{
				select_single = true;
				SelectingTarget = true;
				CFG.GUIVisible = true;
				CFG.ShowWaypoints = true;
				MapView.EnterMapView();
			}
		}

		public void WaypointList()
		{
			if(CFG.Waypoints.Count == 0) return;
			GUILayout.BeginVertical();
			if(GUILayout.Button(CFG.ShowWaypoints? "Hide Waypoints" : "Show Waypoints", 
			                    Styles.active_button,
			                    GUILayout.ExpandWidth(true)))
				CFG.ShowWaypoints = !CFG.ShowWaypoints;
			if(CFG.ShowWaypoints)
			{
				GUILayout.BeginVertical(Styles.white);
				waypointsScroll = GUILayout
					.BeginScrollView(waypointsScroll, 
					                 GUILayout.Height(Utils.ClampH(TCAGui.LineHeight*(CFG.Waypoints.Count+1), 
					                                               TCAGui.ControlsHeight)));
				GUILayout.BeginVertical();
				int i = 0;
				var num = (float)(CFG.Waypoints.Count-1);
				var del = new HashSet<WayPoint>();
				var col = GUI.contentColor;
				foreach(var wp in CFG.Waypoints)
				{
					GUILayout.BeginHorizontal();
					GUI.contentColor = marker_color(i, num);
					var label = string.Format("{0}) {1} [{2}]", 1+i, wp.GetName(), wp.Pos);
					if(CFG.Target == wp)
					{
						var d = wp.DistanceTo(vessel);
						label += string.Format(" ◀ {0}", Utils.formatBigValue((float)d, "m")); 
						if(vessel.horizontalSrfSpeed > 0.1)
							label += string.Format(", ETA {0:c}", new TimeSpan(0,0,(int)(d/vessel.horizontalSrfSpeed)));
					}
					if(GUILayout.Button(new GUIContent(label, "Target this waypoint"), GUILayout.ExpandWidth(true)))
						FlightGlobals.fetch.SetVesselTarget(wp.GetTarget());
					GUI.contentColor = col;
					GUILayout.FlexibleSpace();
					if(LND != null && 
					   Utils.ButtonSwitch("Land", wp.Land, "Land on arrival", GUILayout.Width(50))) 
						wp.Land = !wp.Land;
					if(Utils.ButtonSwitch("||", wp.Pause, "Pause on arrival", GUILayout.Width(25))) 
						wp.Pause = !wp.Pause;
					if(GUILayout.Button(new GUIContent("X", "Delete waypoint"), 
					                    Styles.danger_button, GUILayout.Width(25))) 
						del.Add(wp);
					GUILayout.EndHorizontal();
					i++;
				}
				GUI.contentColor = col;
				if(GUILayout.Button("Clear", Styles.danger_button, GUILayout.ExpandWidth(true)))
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

		WayPoint edited_waypoint;
		string edited_waypoint_name = "";

		void edit_waypoint(int windowID)
		{
			var close = false;
			GUILayout.BeginVertical();
			GUILayout.BeginHorizontal();
			GUILayout.Label("Name:", GUILayout.Width(70));
			edited_waypoint_name = GUILayout.TextField(edited_waypoint_name, GUILayout.ExpandWidth(true));
			GUILayout.EndHorizontal();
			GUILayout.BeginHorizontal();
			GUILayout.Label("Latitude:", GUILayout.Width(70));
			LatField.Draw("°", false, 1, "F1");
			GUILayout.EndHorizontal();
			GUILayout.BeginHorizontal();
			GUILayout.Label("Longitude:", GUILayout.Width(70));
			LonField.Draw("°", false, 1, "F1");
			GUILayout.EndHorizontal();
			GUILayout.BeginHorizontal();
			GUILayout.Label("Altitude:", GUILayout.Width(70));
			AltField.Draw("°", false, 100, "F0");
			GUILayout.EndHorizontal();
			GUILayout.BeginHorizontal();
			if(GUILayout.Button("Cancel", Styles.active_button)) close = true;
			GUILayout.FlexibleSpace();
			if(GUILayout.Button("Delete", Styles.danger_button))
			{
				CFG.Waypoints = new Queue<WayPoint>(CFG.Waypoints.Where(wp => wp != edited_waypoint));
				close = true;
			}
			GUILayout.FlexibleSpace();
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
			GUIWindowBase.TooltipsAndDragWindow(wp_editor_pos);
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
				                 edit_waypoint, 
				                 "Edit Waypoint",
				                 GUILayout.Width(wp_edit_width),
				                 GUILayout.Height(wp_edit_height)).clampToScreen();
		}
		#endregion

		#region Waypoints Overlay
		readonly ActionDamper AddTargetDamper = new ActionDamper();
		const string WPM_ICON = "ThrottleControlledAvionics/Icons/waypoint";
		const string PN_ICON  = "ThrottleControlledAvionics/Icons/path-node";
		const float  IconSize = 16;
		static Texture2D WayPointMarker, PathNodeMarker;

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

		static void DrawLabelAtPointer(string text)
		{ GUI.Label(new Rect(Input.mousePosition.x + 15, Screen.height - Input.mousePosition.y, 300, 200), text); }

		static void DrawLabelAtPointer(string text, double distance)
		{ DrawLabelAtPointer(string.Format("{0}\nDistance: {1}", text, Utils.formatBigValue((float)distance, "m"))); }


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
			{
				edited_waypoint = wp;
				edited_waypoint_name = wp.Name;
				LatField.Value = Utils.CenterAngle((float)wp.Pos.Lat);
				LonField.Value = Utils.CenterAngle((float)wp.Pos.Lon);
				AltField.Value = (float)wp.Pos.Alt;
			}
		}

		//adapted from MechJeb
		bool clicked;
		DateTime clicked_time;
		public void WaypointOverlay() //TODO: add waypoint editing by drag
		{
			if(TCA == null || !TCA.Available || !GUIWindowBase.HUD_enabled) return;
			Vector3d worldPos;
			if(SelectingTarget)
			{
				var coords = MapView.MapIsEnabled? 
					Coordinates.GetAtPointer(vessel.mainBody) :
					Coordinates.GetAtPointerInFlight();
				if(coords != null)
				{
					var t = new WayPoint(coords);
					DrawGroundMarker(vessel.mainBody, coords, new Color(1.0f, 0.56f, 0.0f), out worldPos);
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
								VSL.SetTarget(t);
								MapView.ExitMapView();
							}
							else AddTargetDamper.Run(() => CFG.Waypoints.Enqueue(t));
							CFG.ShowWaypoints = true;
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
			var camera = current_camera;
			if(CFG.ShowWaypoints)
			{
				var i = 0;
				var num = (float)(CFG.Waypoints.Count-1);
				WayPoint wp0 = null;
				var total_dist = 0f;
				var dist2cameraF = Mathf.Pow(Mathf.Max((camera.transform.position - 
				                                        VSL.vessel.transform.position).magnitude, 1), 
				                             GLB.CameraFadeinPower);
				foreach(var wp in CFG.Waypoints)
				{
					current_target_drawn |= wp.Equals(CFG.Target);
					wp.UpdateCoordinates(vessel.mainBody);
					var dist = -1f;
					if(wp0 != null && wp != selected_waypoint)
					{
						total_dist += (float)wp.DistanceTo(wp0, VSL.Body)/dist2cameraF;
						dist = total_dist;
					}
					var c = marker_color(i, num, dist);
					if(wp0 == null) DrawPath(vessel, wp, c);
					else DrawPath(vessel.mainBody, wp0, wp, c);
					if(DrawGroundMarker(vessel.mainBody, wp, c, out worldPos))
					{
						DrawLabelAtPointer(wp.SurfaceDescription(vessel), wp.DistanceTo(vessel));
						select_waypoint(wp);
					}
					else if(wp == selected_waypoint)
						DrawLabelAtPointer(wp.SurfaceDescription(vessel), wp.DistanceTo(vessel));
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
			if(CFG.Target != null && !current_target_drawn && 
			   (!CFG.Target.IsVessel || CFG.Target.GetVessel().LandedOrSplashed))
				DrawWayPoint(CFG.Target, Color.magenta, "Target");
			//custom markers
			VSL.Info.CustomMarkersWP.ForEach(m => DrawWayPoint(m, Color.red, m.Name));
			VSL.Info.CustomMarkersVec.ForEach(m => DrawWorldMarker(m, Color.red, "Custom WayPoint"));
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

		static Rect texture_rect = new Rect(0f, 0f, 1f, 1f);
		static bool DrawMarker(Vector3 icon_center, Color c, float r, Texture2D texture)
		{
			if(c.a.Equals(0)) return false;
			if(texture == null) texture = WayPointMarker;
			var icon_rect = new Rect(icon_center.x - r * 0.5f, (float)Screen.height - icon_center.y - r * 0.5f, r, r);
			Graphics.DrawTexture(icon_rect, texture, texture_rect, 0, 0, 0, 0, c, IconMaterial);
			return icon_rect.Contains(Event.current.mousePosition);
		}

		static bool DrawGroundMarker(CelestialBody body, Coordinates pos, Color c, out Vector3d worldPos, float r = IconSize, Texture2D texture = null)
		{
			worldPos = Vector3d.zero;
			if(c.a.Equals(0)) return false;
			Camera camera;
			Vector3d point;
			if(MapView.MapIsEnabled)
			{
				//TODO: cache local center coordinates of the marker
				camera = PlanetariumCamera.Camera;
				worldPos = body.position + (pos.Alt+body.Radius) * body.GetSurfaceNVector(pos.Lat, pos.Lon);
				if(IsOccluded(worldPos, body)) return false;
				point = ScaledSpace.LocalToScaledSpace(worldPos);
			}
			else
			{
				camera = FlightCamera.fetch.mainCamera;
				worldPos = body.GetWorldSurfacePosition(pos.Lat, pos.Lon, pos.Alt);
				if(camera.transform.InverseTransformPoint(worldPos).z <= 0) return false;
				point = worldPos;
			}
			return DrawMarker(camera.WorldToScreenPoint(point), c, r, texture);
		}

		static bool DrawGroundMarker(CelestialBody body, WayPoint wp, Color c, out Vector3d worldPos, float r = IconSize, Texture2D texture = null)
		{ return DrawGroundMarker(body, wp.Pos, c, out worldPos, r, texture); }

		static void DrawWorldMarker(Vector3d wPos, Color c, string label = "", float r = IconSize, Texture2D texture = null)
		{
			if(c.a.Equals(0)) return;
			var camera = MapView.MapIsEnabled ? PlanetariumCamera.Camera : FlightCamera.fetch.mainCamera;
			if(camera.transform.InverseTransformPoint(wPos).z <= 0) return;
			if(DrawMarker(camera.WorldToScreenPoint(MapView.MapIsEnabled? ScaledSpace.LocalToScaledSpace(wPos) : wPos), c, r, texture) &&
			   !string.IsNullOrEmpty(label)) DrawLabelAtPointer(label);

		}

		void DrawWayPoint(WayPoint wp, Color c, string label = null, float r = IconSize, Texture2D texture = null)
		{
			Vector3d worldPos;
			if(DrawGroundMarker(vessel.mainBody, wp, c, out worldPos, r, texture))
			{
				DrawLabelAtPointer(label ?? wp.SurfaceDescription(vessel), wp.DistanceTo(vessel));
				select_waypoint(wp);
			}
		}

		static void DrawPath(CelestialBody body, WayPoint wp0, WayPoint wp1, Color c, float alpha = -1)
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
				DrawGroundMarker(body, p, c, out point, IconSize/2, PathNodeMarker);
				Utils.GLLine(last_point, point, line_color);
				last_point = point;
			}
			Utils.GLLine(last_point, wp1.WorldPos(body), line_color);
		}

		static void DrawPath(Vessel v, WayPoint wp1, Color c)
		{ DrawPath(v.mainBody, new WayPoint(v.latitude, v.longitude, v.altitude), wp1, c, 1); }

		//Tests if byBody occludes worldPosition, from the perspective of the planetarium camera
		static bool IsOccluded(Vector3d worldPosition, CelestialBody byBody)
		{
			var c_pos = MapView.MapIsEnabled? 
				ScaledSpace.ScaledToLocalSpace(PlanetariumCamera.Camera.transform.position) :
				(Vector3d)FlightCamera.fetch.mainCamera.transform.position;
			return Vector3d.Angle(c_pos-worldPosition, byBody.position-worldPosition) <= 90.0;
		}
		#endregion
	}
}

