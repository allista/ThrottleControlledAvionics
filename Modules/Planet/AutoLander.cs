//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2015 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using AT_Utils;

namespace ThrottleControlledAvionics
{
	[CareerPart]
	[RequireModules(typeof(HorizontalSpeedControl),
	                typeof(AltitudeControl),
	                typeof(VerticalSpeedControl),
	                typeof(Anchor),
	                typeof(PointNavigator))]
	public class AutoLander : TCAModule
	{
		public class Config : ModuleConfig
		{
			[Persistent] public float MaxUnevenness        = 0.1f;
			[Persistent] public float UnevennessThreshold  = 0.3f;
			[Persistent] public float MaxHorizontalTime    = 5f;
			[Persistent] public float MinVerticalSpeed     = 0.1f;
			[Persistent] public float WideCheckAltitude    = 200f;
			[Persistent] public float MaxWideCheckAltitude = 1000f;
			[Persistent] public int   WideCheckLevel       = 5;
			[Persistent] public float NodeTargetRange      = 1;
			[Persistent] public float NodeAnchorF          = 0.5f;
			[Persistent] public float GearOnAtH            = 5;
			[Persistent] public float StopAtH              = 2;
			[Persistent] public float StopTimer            = 2;
			[Persistent] public float CutoffTimer          = 2;
			[Persistent] public int   RaysPerFrame         = 5;

			public float MaxStartAltitude;

			public override void Init ()
			{
				base.Init ();
				MaxStartAltitude = MaxWideCheckAltitude/2;
			}
		}
		static Config LND { get { return Globals.Instance.LND; } }

		static int RadarMask = (1 << LayerMask.NameToLayer("Local Scenery") | 1 << LayerMask.NameToLayer("Parts") | 1);
		enum Stage { None, Start, PointCheck, WideCheck, FlatCheck, MoveNext, StartLanding, Land }

		Stage stage;
		IEnumerator scanner;
		SurfaceNode[,] Nodes;
		readonly List<SurfaceNode> FlatNodes = new List<SurfaceNode>();
		HashSet<SurfaceNode> TriedNodes;
		SurfaceNode NextNode, FlattestNode;
		int side, bside, center, total_rays, done_rays;
		Vector3 right, fwd, up, down, dir, sdir, anchor;
		float MaxDistance, delta;
		readonly Timer StopTimer = new Timer();
		readonly Timer CutoffTimer = new Timer();

		public float Progress { get { return scanner != null? done_rays/(float)total_rays : 0f; } }

		public AutoLander(ModuleTCA tca) : base(tca) {}

		public override void Init()
		{
			base.Init();
			StopTimer.Period = LND.StopTimer;
			CutoffTimer.Period = LND.CutoffTimer;
			CFG.AP1.AddHandler(this, Autopilot1.Land);
			TriedNodes = new HashSet<SurfaceNode>(new SurfaceNode.Comparer(VSL.Geometry.R));
		}

		protected override void UpdateState() 
		{ 
			base.UpdateState();
			IsActive &= VSL.OnPlanet && CFG.AP1[Autopilot1.Land]; 
		}

		public void LandCallback(Multiplexer.Command cmd)
		{
			stage = Stage.None;
			scanner = null;
			NextNode = null;
			Nodes = null;
			StopTimer.Reset();
			CutoffTimer.Reset();

			switch(cmd)
			{
			case Multiplexer.Command.On:
			case Multiplexer.Command.Resume:
				if(VSL.LandedOrSplashed) { CFG.AP1.Off(); break; }
				CFG.HF.On(HFlight.Stop);
				set_initial_altitude();
				CFG.AltitudeAboveTerrain = true;
				TriedNodes = new HashSet<SurfaceNode>(new SurfaceNode.Comparer(VSL.Geometry.R));
				break;

			case Multiplexer.Command.Off:
				CFG.VF.On(VFlight.AltitudeControl);
				if(!VSL.LandedOrSplashed)
					CFG.DesiredAltitude = VSL.Altitude;
				ClearStatus();
				break;
			}
		}

		SurfaceNode get_surface_node(float radius)
		{
			RaycastHit raycastHit;
			var c = VSL.Geometry.C+dir*(VSL.Geometry.R+0.1f);
			if(Physics.SphereCast(c, radius, dir, out raycastHit, MaxDistance, RadarMask))
			{
				if(VSL.Body.ocean && 
				   (raycastHit.point-VSL.Body.position).magnitude < VSL.Body.Radius)
					return null;
				return new SurfaceNode(c, raycastHit.point, up);
			}
			return null;
		}

		IEnumerator surface_scanner(int lvl, SurfaceNode start = null, float d = 0)
		{
			if(VSL.refT == null) yield break;
			//initialize the system
			side   = lvl*2-1;
			bside  = side+2;
			center = (bside-1)/2;
			up     = VSL.Physics.Up;
			down   = -VSL.Altitude.Relative*up;
			right  = Vector3d.Cross(VSL.Physics.Up, VSL.OnPlanetParams.Fwd).normalized;
			fwd    = Vector3.Cross(right, up);
			Nodes  = new SurfaceNode[bside,bside];
			delta  = d > 0? d : CFG.DesiredAltitude/bside*lvl;
			anchor = Vector3.zero;
			if(start == null) anchor = VSL.Physics.wCoM+down;
			else anchor = start.position;
			//job progress
			total_rays = bside*bside+side*side+1;
			done_rays = 0;
			var frame_rays = 0;
//			Utils.Log("Scanning Surface: {0}x{0} -> {1}x{1}", bside, side);//debug
			yield return null;
			//cast the rays
			for(int i = 0; i < bside; i++)
				for(int j = 0; j < bside; j++)
				{
					sdir = anchor-VSL.Physics.wCoM;
					MaxDistance = sdir.magnitude * 10;
					dir = (sdir+right*(i-center)*delta+fwd*(j-center)*delta).normalized;
					Nodes[i,j] = get_surface_node(delta/2);
					done_rays++;
					frame_rays++;
					if(frame_rays >= LND.RaysPerFrame)
					{
						frame_rays = 0;
						yield return null;
					}
				}
			//compute unevenness
			for(int i = 1; i <= side; i++)
				for(int j = 1; j <= side; j++)
				{
					var n = Nodes[i,j];
					if(n == null) continue;
					for(int u = i-1; u <= i+1; u += 2)
						for(int v = j-1; v <= j+1; v += 2)
						{
							var n1 = Nodes[u,v];
							if(n1 == null) continue;
							n.UpdateUnevenness(n1);
						}
					done_rays++;
					frame_rays++;
					if(frame_rays >= LND.RaysPerFrame)
					{
						frame_rays = 0;
						yield return null;
					}
				}
			if(lvl > 1)
			{
				FlatNodes.Clear();
				for(int i = 1; i <= side; i++)
					for(int j = 1; j <= side; j++)
					{
						var n = Nodes[i,j];
						if(n != null && n.flat && !TriedNodes.Contains(n))
							FlatNodes.Add(n);
					}
				done_rays++;
				yield return null;
				FlatNodes.Sort((n1, n2) => n1.distance.CompareTo(n2.distance));
			}
			#if DEBUG
//			print_nodes();
			#endif
		}

		SurfaceNode flattest_node 
		{
			get 
			{
				if(Nodes == null) return null;
				SurfaceNode flattest = null;
				for(int i = 1; i <= side; i++)
					for(int j = 1; j <= side; j++)
					{
						var n = Nodes[i,j];
						if(n == null) continue;
						if(TriedNodes.Contains(n)) continue;
						if(flattest == null) flattest = n;
						else if(flattest.unevenness > n.unevenness)
							flattest = n;
					}
				return flattest;
			}
		}

		SurfaceNode center_node
		{ get { return Nodes == null ? null : Nodes[center, center]; } }

		void set_initial_altitude()
		{
			CFG.AltitudeAboveTerrain = true;
			VSL.Altitude.Update();
			CFG.DesiredAltitude = Utils.Clamp(VSL.Altitude.Relative+VSL.VerticalSpeed.Absolute*3, 
			                              VSL.Geometry.H*2, LND.MaxStartAltitude);
		}

		bool altitude_changed
		{
			get
			{
				var err = Mathf.Abs(VSL.Altitude.Relative-CFG.DesiredAltitude);
				if(err > 10)
				{
					CFG.VF.OnIfNot(VFlight.AltitudeControl);
					return false;
				}
				else if(err < 5)
				{
					CFG.VF.OffIfOn(VFlight.AltitudeControl);
					CFG.SmoothSetVSC(0);
				}
				return Mathf.Abs(VSL.VerticalSpeed.Absolute) < LND.MinVerticalSpeed;
			}
		}

		bool stopped
		{
			get
			{
				if(!CFG.Nav[Navigation.Anchor]) CFG.HF.OnIfNot(HFlight.Stop);
				if(VSL.Geometry.R/Utils.ClampL(VSL.HorizontalSpeed, 1e-10) > LND.MaxHorizontalTime)
					return StopTimer.TimePassed;
				else StopTimer.Reset();
				return false;
			}
		}

		bool fully_stopped 
		{ 
			get 
			{ 
				var s = stopped;
				var a = altitude_changed;
				return s && a;
			} 
		}

		bool moved_to_next_node
		{
			get
			{
				CFG.AltitudeAboveTerrain = true;
				CFG.VF.OnIfNot(VFlight.AltitudeControl);
				if(CFG.Anchor == null) { CFG.AP1.Off(); return false; }
				if(CFG.Anchor.CloseEnough(VSL))
				{
					if(NextNode.flat)
					{
						CFG.Nav.OnIfNot(Navigation.Anchor);
						if(CFG.Anchor.DistanceTo(VSL.vessel) < delta)
							return StopTimer.TimePassed;
					}
					else return true;
				}
				else if(!CFG.Nav[Navigation.Anchor] && 
				        (!CFG.Nav[Navigation.GoToTarget] || !VSL.Target.Equals(CFG.Anchor)))
				{
					SetTarget(CFG.Anchor);
					CFG.Nav.On(Navigation.GoToTarget);
				}
				StopTimer.Reset();
				return false;
			}
		}

		float distance_to_node(SurfaceNode n)
		{ return Vector3.ProjectOnPlane(n.position-VSL.Physics.wCoM, VSL.Physics.Up).magnitude/VSL.Geometry.R; }

		void wide_check(float delta_alt = 0)
		{
			if(CFG.DesiredAltitude < LND.WideCheckAltitude)
				CFG.DesiredAltitude = LND.WideCheckAltitude;
			if(VSL.Altitude.Relative > CFG.DesiredAltitude)
				CFG.DesiredAltitude = VSL.Altitude.Relative;
			else CFG.DesiredAltitude += delta_alt;
			if(CFG.DesiredAltitude > LND.MaxWideCheckAltitude)
			{
				CFG.AP1.Off();
				Status("red", "Unable to find suitale place for landing.");
			}
			else stage = Stage.WideCheck;
		}

		void try_move_to_flattest()
		{
			NextNode = FlattestNode;
			if(NextNode != null && 
			   distance_to_node(NextNode) > LND.NodeTargetRange*2 && 
			   NextNode.unevenness > LND.UnevennessThreshold) 
				move_next();
			else wide_check(LND.WideCheckAltitude);
		}

		void move_next()
		{
			CFG.Anchor = new WayPoint(VSL.Body.GetLatitude(NextNode.position),
			                          VSL.Body.GetLongitude(NextNode.position));
			CFG.Anchor.Radius = LND.NodeTargetRange;
			CFG.Target = CFG.Anchor;
			stage = Stage.MoveNext;
		}

		void land()
		{
			var c = center_node;
			CFG.Anchor = new WayPoint(VSL.Body.GetLatitude(c.position),
			                          VSL.Body.GetLongitude(c.position));
			CFG.Anchor.Radius = LND.NodeTargetRange;
			CFG.Target = CFG.Anchor;
			CFG.Nav.OnIfNot(Navigation.Anchor);
			CFG.DesiredAltitude = VSL.Geometry.H*LND.GearOnAtH;
			stage = Stage.Land;
		}

		bool scan(int lvl, SurfaceNode start = null, float d = 0)
		{
			if(scanner == null) scanner = surface_scanner(lvl, start, d);
			if(scanner.MoveNext()) return true;
			scanner = null;
			return false;
		}

		protected override void Update()
		{
			if(!IsActive || CFG.AP1.Paused) return;
			CFG.AltitudeAboveTerrain = true;
			CFG.BlockThrottle = true;
			switch(stage)
			{
			case Stage.None:
				CFG.VF.OnIfNot(VFlight.AltitudeControl);
				Status("Preparing for the landing sequence...");
				if(stopped) stage = Stage.PointCheck;	
				break;
			case Stage.PointCheck:
				Status("Checking the surface underneath the ship...");
				if(!stopped) break;
				if(scan(1, null, VSL.Geometry.D)) break;
				if(Nodes == null || center_node == null) 
				{ wide_check(); break; }
				TriedNodes.Add(center_node);
				if(!center_node.flat) wide_check();
				else land();
				break;
			case Stage.FlatCheck:
				Status("Checking potential landing sites...");
				if(!stopped) break;
				if(FlatNodes.Count > 0)
				{
					if(scan(1, FlatNodes[0], VSL.Geometry.D)) break;
					FlatNodes.RemoveAt(0);
					var c = center_node;
					if(c == null || !c.flat) break;
					NextNode = c;
					move_next();
					break;
				}
				else try_move_to_flattest();
				break;
			case Stage.WideCheck:
				if(!fully_stopped) { Status("Prepearing for surface scanning..."); break; }
				Status("yellow", "Scanning the surface: {0:P0}", Progress);
				if(scan(LND.WideCheckLevel)) break;
				FlattestNode = flattest_node;
				if(FlatNodes.Count > 0) 
				{ stage = Stage.FlatCheck; break; }
				else try_move_to_flattest();
				break;
			case Stage.MoveNext:
				if(NextNode.flat) Status("Moving to a potential landing site...");
				else Status("Searching for a landing site...");
				if(!moved_to_next_node) break;
				CFG.DesiredAltitude = VSL.Altitude.Relative;
				if(NextNode.flat) stage = Stage.PointCheck;
				else wide_check();
				break;
			case Stage.StartLanding:
				Status("lime", "Landing...");
				CFG.DesiredAltitude = VSL.Geometry.H*LND.GearOnAtH;
				apply_cfg(cfg => cfg.AP1.XOnIfNot(Autopilot1.Land));
				stage = Stage.Land;
				goto case Stage.Land;
			case Stage.Land:
				CFG.VTOLAssistON = true;
				if(CFG.AutoGear) Status("lime", "Landing...");
				else Status("yellow", "Landing. Autodeployment of landing gear is disabled.");
				if(CFG.DesiredAltitude > 0)
				{
					CFG.Nav.OnIfNot(Navigation.Anchor);
					CFG.VF.OnIfNot(VFlight.AltitudeControl);
					if(VSL.Altitude.Relative-CFG.DesiredAltitude > 5 || VSL.VerticalSpeed.Absolute < -1) break;
					CFG.VerticalCutoff = VSL.OnPlanetParams.SlowThrust? -0.5f : -1f;
					CFG.DesiredAltitude = -10;
				}
				CFG.VF.Off();
				if(VSL.LandedOrSplashed) 
				{ 
					if(!CutoffTimer.TimePassed) break;
					CFG.AP1.XOff(); 
					CFG.VerticalCutoff = -10; 
					CFG.VF.On(VFlight.AltitudeControl);
				}
				else
				{
					if(VSL.Altitude.Relative > LND.StopAtH*VSL.Geometry.H)
						CFG.Nav.OnIfNot(Navigation.Anchor);
					else CFG.HF.OnIfNot(HFlight.Stop);
					CFG.SmoothSetVSC((VSL.OnPlanetParams.SlowThrust? -0.5f : -1f)*Utils.ClampL(1-VSL.HorizontalSpeed, 0.1f), -1, 0);
				}
				CutoffTimer.Reset();
				break;
			default: 
				CFG.AP1.Off();
				break;
			}
		}

		#if DEBUG
		public void RadarBeam()
		{
			if(!CFG.AP1[Autopilot1.Land]) return;
			if(Nodes == null) return;
			if(scanner == null && NextNode != null)
			{
				Utils.GLLine(VSL.Physics.wCoM, NextNode.position, Color.Lerp(Color.blue, Color.red, NextNode.unevenness));
				return;
			}
			for(int i = 0; i < bside; i++)
				for(int j = 0; j < bside; j++)
				{
					var n = Nodes[i,j];
					if(n == null) continue;
					Utils.GLLine(VSL.Physics.wCoM, n.position, Color.Lerp(Color.blue, Color.red, n.unevenness));
				}
		}

		void print_nodes()
		{
			if(Nodes == null) return;
			var nodes = string.Format("Nodes: {0}x{0}\n", bside);
			for(int i = 0; i < bside; i++)
				for(int j = 0; j < bside; j++)
				{
					var n = Nodes[i,j];
					nodes += string.Format("[{0},{1}]: pos ", i, j);
					if(n == null) { nodes += "null\n"; continue; }
					nodes += Utils.formatVector(n.position) + "\n";
					nodes += string.Format("unevenness: {0}", n.unevenness) + "\n";
					nodes += string.Format("flat: {0}", n.flat) + "\n";
				}
			Utils.Log(nodes+"\n");
		}
		#endif

		class SurfaceNode
		{
			internal class Comparer : IEqualityComparer<SurfaceNode>
			{
				readonly float threshold;

				public Comparer(float threshold) { this.threshold = threshold; }

				public bool Equals(SurfaceNode n1, SurfaceNode n2)
				{ return (n1.position-n2.position).magnitude < threshold; }

				public int GetHashCode(SurfaceNode n) 
				{ return (int)(n.position.x/threshold) ^ (int)(n.position.y/threshold) ^ (int)(n.position.z/threshold); }
			}

			Vector3 up;
			public Vector3 position;
			public float unevenness;
			public float distance;
			public bool flat;
			public readonly  HashSet<SurfaceNode> neighbours = new HashSet<SurfaceNode>();

			public SurfaceNode(Vector3 ori, Vector3 pos, Vector3 up) 
			{ 
				distance = Vector3.ProjectOnPlane(pos-ori, up).magnitude; 
				position = pos;
				this.up  = up; 
			}

			public float DistanceTo(SurfaceNode n)
			{ return (n.position-position).magnitude; }

			public void UpdateUnevenness(SurfaceNode n)
			{
				if(neighbours.Contains(n)) return;
				var unev = Mathf.Abs(Vector3.Dot((n.position-position).normalized, up));
				unevenness += unev;
				n.unevenness += unev;
				neighbours.Add(n);
				n.neighbours.Add(this);
				flat = unevenness < LND.MaxUnevenness;
			}
		}
	}
}

