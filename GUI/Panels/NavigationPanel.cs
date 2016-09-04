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

		public NavigationPanel(ModuleTCA tca) : base(tca) {}

		AutoLander LND;
		PointNavigator PN;
		BallisticJump BJ;

		bool select_single;
		public bool SelectingTarget { get; private set; }
		Vector2 waypointsScroll;
		readonly ActionDamper AddTargetDamper = new ActionDamper();
		const string WPM_ICON = "ThrottleControlledAvionics/Icons/waypoint";
		const string PN_ICON  = "ThrottleControlledAvionics/Icons/path-node";
		const float  IconSize = 16;
		static Texture2D WayPointMarker, PathNodeMarker;

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
			            !(VSL.Target is WayPoint) && 
			            (CFG.Waypoints.Count == 0 || VSL.Target != CFG.Waypoints.Peek().GetTarget()))
			{
				if(GUILayout.Button(new GUIContent("Add As Waypoint", "Add current target as a waypoint"), 
				                    Styles.active_button, GUILayout.Width(120)))
				{
					CFG.Waypoints.Enqueue(new WayPoint(VSL.Target));
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
				if(GUILayout.Button(new GUIContent("Del Target", "Remove target point"), 
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
					var label = string.Format("{0}) {1}", 1+i, wp.GetName());
					if(CFG.Target == wp)
					{
						var d = wp.DistanceTo(vessel);
						label += string.Format(" <= {0}", Utils.formatBigValue((float)d, "m")); 
						if(vessel.horizontalSrfSpeed > 0.1)
							label += string.Format(", ETA {0:c}", new TimeSpan(0,0,(int)(d/vessel.horizontalSrfSpeed)));
					}
					if(GUILayout.Button(label,GUILayout.ExpandWidth(true)))
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

		#region Waypoints Overlay
		static Color marker_color(int i, float N)
		{ 
			if(N.Equals(0)) return Color.red;
			var t = i/N;
			return t < 0.5f ? 
				Color.Lerp(Color.red, Color.green, t*2).Normalized() : 
				Color.Lerp(Color.green, Color.cyan, (t-0.5f)*2).Normalized(); 
		}

		static void DrawLabelAtPointer(string text)
		{ GUI.Label(new Rect(Input.mousePosition.x + 15, Screen.height - Input.mousePosition.y, 300, 200), text); }

		static void DrawLabelAtPointer(string text, double distance)
		{ DrawLabelAtPointer(string.Format("{0}\nDistance: {1}", text, Utils.formatBigValue((float)distance, "m"))); }

		//adapted from MechJeb
		bool clicked;
		DateTime clicked_time;
		public void WaypointOverlay()
		{
			if(TCA == null || !TCA.Available || !TCAGui.HUD_enabled) return;
			if(SelectingTarget)
			{
				var coords = MapView.MapIsEnabled? 
					Coordinates.GetAtPointer(vessel.mainBody) :
					Coordinates.GetAtPointerInFlight();
				if(coords != null)
				{
					var t = new WayPoint(coords);
					DrawGroundMarker(vessel.mainBody, coords, new Color(1.0f, 0.56f, 0.0f));
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
							if(select_single)
							{
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
			if(CFG.ShowWaypoints)
			{
				var i = 0;
				var num = (float)(CFG.Waypoints.Count-1);
				WayPoint wp0 = null;
				foreach(var wp in CFG.Waypoints)
				{
					current_target_drawn |= wp.Equals(CFG.Target);
					wp.UpdateCoordinates(vessel.mainBody);
					var c = marker_color(i, num);
					if(wp0 == null) DrawPath(vessel, wp, c);
					else DrawPath(vessel.mainBody, wp0, wp, c);
					if(DrawGroundMarker(vessel.mainBody, wp, c))
						DrawLabelAtPointer(wp.SurfaceDescription(vessel), wp.DistanceTo(vessel));
					wp0 = wp; i++;
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
			if(texture == null) texture = WayPointMarker;
			r *= ScreenSafeUI.PixelRatio;
			var icon_rect = new Rect(icon_center.x - r * 0.5f, (float)Screen.height - icon_center.y - r * 0.5f, r, r);
			Graphics.DrawTexture(icon_rect, texture, texture_rect, 0, 0, 0, 0, c, IconMaterial);
			return icon_rect.Contains(Event.current.mousePosition);
		}

		static bool DrawGroundMarker(CelestialBody body, Coordinates pos, Color c, float r = IconSize, Texture2D texture = null)
		{
			Vector3d center;
			Camera camera;
			if(MapView.MapIsEnabled)
			{
				//TODO: cache local center coordinates of the marker
				camera = PlanetariumCamera.Camera;
				center = body.position + (body.TerrainAltitude(pos.Lat, pos.Lon)+body.Radius) * body.GetSurfaceNVector(pos.Lat, pos.Lon);
			}
			else
			{
				camera = FlightCamera.fetch.mainCamera;
				center = body.GetWorldSurfacePosition(pos.Lat, pos.Lon, body.TerrainAltitude(pos.Lat, pos.Lon)+GLB.WaypointHeight);
				if(camera.transform.InverseTransformPoint(center).z <= 0) return false;
			}
			return !IsOccluded(center, body) && 
				DrawMarker(camera.WorldToScreenPoint(MapView.MapIsEnabled ? ScaledSpace.LocalToScaledSpace(center) : center), c, r, texture);
		}

		static bool DrawGroundMarker(CelestialBody body, WayPoint wp, Color c, float r = IconSize, Texture2D texture = null)
		{ return DrawGroundMarker(body, wp.Pos, c, r, texture); }

		static void DrawWorldMarker(Vector3d wPos, Color c, string label = "", float r = IconSize, Texture2D texture = null)
		{
			var camera = MapView.MapIsEnabled ? PlanetariumCamera.Camera : FlightCamera.fetch.mainCamera;
			if(camera.transform.InverseTransformPoint(wPos).z <= 0) return;
			if(DrawMarker(camera.WorldToScreenPoint(MapView.MapIsEnabled? ScaledSpace.LocalToScaledSpace(wPos) : wPos), c, r, texture) &&
			   !string.IsNullOrEmpty(label)) DrawLabelAtPointer(label);

		}

		void DrawWayPoint(WayPoint wp, Color c, string label = null, float r = IconSize, Texture2D texture = null)
		{
			if(DrawGroundMarker(vessel.mainBody, wp, c, r, texture))
				DrawLabelAtPointer(label ?? wp.SurfaceDescription(vessel), wp.DistanceTo(vessel));
		}

		static void DrawPath(CelestialBody body, WayPoint wp0, WayPoint wp1, Color c)
		{
			var D = wp1.AngleTo(wp0);
			var N = (int)Mathf.Clamp((float)D*Mathf.Rad2Deg, 2, 5);
			var dD = D/N;
			for(int i = 1; i<N; i++)
			{
				var p = wp0.PointBetween(wp1, dD*i);
				DrawGroundMarker(body, p, c, IconSize/2, PathNodeMarker);
			}
		}

		static void DrawPath(Vessel v, WayPoint wp1, Color c)
		{ DrawPath(v.mainBody, new WayPoint(v.latitude, v.longitude), wp1, c); }

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

