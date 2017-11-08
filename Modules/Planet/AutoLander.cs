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
			[Persistent] public float MaxHorizontalTime    = 5f;
			[Persistent] public float MinVerticalSpeed     = 0.1f;
			[Persistent] public float WideCheckAltitude    = 200f;
			[Persistent] public float MaxWideCheckAltitude = 1000f;
			[Persistent] public int   WideCheckLevel       = 5;
			[Persistent] public float NodeTargetRange      = 1;
			[Persistent] public float NodeAnchorF          = 0.5f;
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
		enum Stage { None, Start, PointCheck, WideCheck, FlatCheck, MoveNext, Land }
		Stage stage;

		IEnumerator scanner;
		SurfaceNode[,] Nodes;
		readonly List<SurfaceNode> FlatNodes = new List<SurfaceNode>();
		HashSet<SurfaceNode> TriedNodes;
        SurfaceNode StartNode, NextNode, FlattestNode, FlattestNodeTired;
        Coordinates anchor;
		int side, bside, center, total_rays, done_rays;
		Vector3 right, fwd, up, dir, sdir;
		float WideCheckAlt, MaxDistance, delta;
		readonly Timer StopTimer = new Timer();
		readonly Timer CutoffTimer = new Timer();

        public void StartFrom(WayPoint wp) { StartNode = new SurfaceNode(wp, VSL.Body); }
        public void StartFromTarget() { StartNode = new SurfaceNode(CFG.Target, VSL.Body); }

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

        public override void Disable()
        {
            CFG.AP1.OffIfOn(Autopilot1.Land);
        }

		public void LandCallback(Multiplexer.Command cmd)
		{
			stage = Stage.None;
			scanner = null;
			NextNode = null;
            FlattestNodeTired = null;
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
				UseTarget();
                TriedNodes = new HashSet<SurfaceNode>(new SurfaceNode.Comparer((VSL.Geometry.R*Mathf.Rad2Deg/VSL.Body.Radius)));
				break;

			case Multiplexer.Command.Off:
				if(!VSL.LandedOrSplashed)
				{
					CFG.VF.On(VFlight.AltitudeControl);
					CFG.DesiredAltitude = VSL.Altitude;
				}
				StartNode = null;
				ClearStatus();
				SetTarget();
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
                return new SurfaceNode(raycastHit.point, VSL.Body);
			}
			return null;
		}

		IEnumerator surface_scanner(int lvl, SurfaceNode start = null, float d = -1)
		{
			if(VSL.refT == null) yield break;
			//initialize the system
			side   = lvl*2-1;
			bside  = side+2;
			center = (bside-1)/2;
			up     = VSL.Physics.Up;
			right  = Vector3d.Cross(VSL.Physics.Up, VSL.OnPlanetParams.Fwd).normalized;
			fwd    = Vector3.Cross(right, up);
			Nodes  = new SurfaceNode[bside,bside];
            delta  = d > 0? d : WideCheckAlt/bside*lvl;
			anchor = null;
            anchor = start == null ? new Coordinates(VSL.Physics.wCoM, VSL.Body) : start.Pos.Copy();
            anchor.SetAlt2Surface(VSL.Body);
			//job progress
			total_rays = bside*bside+side*side+1;
			done_rays = 0;
			var frame_rays = 0;
			yield return null;
			//cast the rays
			for(int i = 0; i < bside; i++)
				for(int j = 0; j < bside; j++)
				{
                    sdir = anchor.WorldPos(VSL.Body)-VSL.Physics.wCoM;
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
                    var complete = true;
					if(n == null) continue;
					for(int u = i-1; u <= i+1; u += 2)
						for(int v = j-1; v <= j+1; v += 2)
						{
							var n1 = Nodes[u,v];
							if(n1 == null) 
                            {
                                complete = false;
                                continue;
                            }
							n.UpdateUnevenness(n1);
						}
                    n.complete = complete;
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
						if(n != null && 
                           n.complete && n.flat && 
                           !TriedNodes.Contains(n))
							FlatNodes.Add(n);
					}
				done_rays++;
				yield return null;
				FlatNodes.Sort((n1, n2) => n1.DistanceTo(VSL).CompareTo(n2.DistanceTo(VSL)));
			}
			#if DEBUG
			print_nodes();
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
                        if(n == null || !n.complete || TriedNodes.Contains(n)) continue;
						if(flattest == null) flattest = n;
						else if(flattest.unevenness > n.unevenness)
							flattest = n;
					}
				return flattest;
			}
		}

        SurfaceNode closest_node 
        {
            get 
            {
                if(Nodes == null) return null;
                SurfaceNode closest = null;
                var cdist = double.MaxValue;
                for(int i = 1; i <= side; i++)
                    for(int j = 1; j <= side; j++)
                    {
                        var n = Nodes[i,j];
                        if(n == null) continue;
                        var dist = n.DistanceTo(VSL);
                        if(cdist > dist)
                        {
                            closest = n;
                            cdist = dist;
                        }
                    }
                return closest;
            }
        }

		SurfaceNode center_node
		{ get { return Nodes == null ? null : Nodes[center, center]; } }

		void set_initial_altitude()
		{
			CFG.AltitudeAboveTerrain = true;
			VSL.Altitude.Update();
            WideCheckAlt = Utils.Clamp(VSL.Altitude.Relative+VSL.VerticalSpeed.Absolute*3, 
                                       VSL.Geometry.D*2, LND.MaxStartAltitude);
		}

		bool altitude_changed
		{
			get
			{
                var err = Mathf.Abs(VSL.Altitude.Relative-WideCheckAlt)/WideCheckAlt;
                if(err > 0.1f)
				{
					CFG.VF.OnIfNot(VFlight.AltitudeControl);
                    CFG.DesiredAltitude = WideCheckAlt;
					return false;
				}
				if(err < 0.05f)
				{
					CFG.VF.OffIfOn(VFlight.AltitudeControl);
                    CFG.VerticalCutoff = 0;
				}
				return Mathf.Abs(VSL.VerticalSpeed.Absolute) < LND.MinVerticalSpeed;
			}
		}

		bool stopped
		{
			get
			{
				if(!CFG.Nav[Navigation.Anchor]) CFG.HF.OnIfNot(HFlight.Stop);
				if(VSL.Geometry.R/Utils.ClampL(VSL.HorizontalSpeed, 1e-5f) > LND.MaxHorizontalTime)
					return StopTimer.TimePassed;
				StopTimer.Reset();
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

		void wide_check(float delta_alt = 0)
		{
            if(WideCheckAlt < VSL.Geometry.D*2)
                WideCheckAlt = VSL.Geometry.D*2;
            WideCheckAlt += delta_alt;
            if(VSL.Altitude.Relative > WideCheckAlt)
                WideCheckAlt = VSL.Altitude.Relative;
            if(WideCheckAlt > LND.MaxWideCheckAltitude)
			{
				CFG.AP1.Off();
				Status("red", "Unable to find suitale place for landing.");
			}
			else stage = Stage.WideCheck;
		}

		void search_for_next()
		{
            //move to flattest node if it is flatter than those we've already checked
            NextNode = FlattestNode;
            if(FlattestNode != null &&
               (FlattestNodeTired == null || 
                FlattestNodeTired.unevenness > NextNode.unevenness &&
                NextNode.DistanceTo(VSL)/VSL.Geometry.R > LND.NodeTargetRange*2))
            {
                FlattestNodeTired = NextNode;
                WideCheckAlt += LND.WideCheckAltitude *
                    (float)(NextNode.unevenness/LND.MaxUnevenness);
                move_next();
            }
			else 
                wide_check(LND.WideCheckAltitude);
		}

		void move_next()
		{
            CFG.Anchor = NextNode.ToWayPoint();
			CFG.Anchor.Radius = LND.NodeTargetRange;
            SetTarget(CFG.Anchor);
			stage = Stage.MoveNext;
		}

        void land(SurfaceNode n)
		{
            CFG.Anchor = n.ToWayPoint();
			CFG.Anchor.Radius = LND.NodeTargetRange;
            SetTarget(CFG.Anchor);
			CFG.Nav.OnIfNot(Navigation.Anchor);
            WideCheckAlt = VSL.Geometry.H*(LND.StopAtH+1);
			TCA.SquadConfigAction(cfg => cfg.AP1.XOnIfNot(Autopilot1.Land));
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
			if(CFG.AP1.Paused) return;
            CFG.DesiredAltitude = WideCheckAlt;
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
				if(scan(1, StartNode, VSL.Geometry.D)) break;
                if(center_node == null) { wide_check(); break; }
                TriedNodes.Add(center_node);
                if(!center_node.flat) wide_check();
                else land(center_node);
				break;
			case Stage.FlatCheck:
				Status("Checking potential landing sites...");
				if(!stopped) break;
				if(FlatNodes.Count > 0)
				{
					if(scan(1, FlatNodes[0], VSL.Geometry.D)) break;
					FlatNodes.RemoveAt(0);
                    if(center_node == null || !center_node.flat) break;
                    NextNode = center_node;
					move_next();
				}
				else 
                    search_for_next();
				break;
			case Stage.WideCheck:
				//FIXME: the first wide check sometimes causes uncontrolled ascent
				if(!fully_stopped) 
                { 
                    Status("Prepearing for surface scanning..."); 
                    break; 
                }
				if(scan(LND.WideCheckLevel)) 
                {
                    Status("Scanning for <color=yellow><b>flat</b></color> surface to land: <color=lime>{0:P1}</color>", Progress);
                    break;
                }
                FlattestNode = flattest_node;
				if(FlatNodes.Count > 0) 
                    stage = Stage.FlatCheck;
				else 
                    search_for_next();
				break;
			case Stage.MoveNext:
				if(NextNode.flat) Status("Moving to a potential landing site...");
				else Status("Searching for a landing site...");
				if(!moved_to_next_node) break;
                WideCheckAlt = VSL.Altitude.Relative;
				if(NextNode.flat) 
				{
					StartNode = NextNode;
					stage = Stage.PointCheck;
				}
				else wide_check();
				break;
			case Stage.Land:
				CFG.VTOLAssistON = true;
				if(CFG.AutoGear) Status("lime", "Landing...");
				else Status("yellow", "Landing. Autodeployment of landing gear is disabled.");
                if(WideCheckAlt > 0)
				{
					CFG.Nav.OnIfNot(Navigation.Anchor);
					CFG.VF.OnIfNot(VFlight.AltitudeControl);
                    if(VSL.Altitude.Relative-WideCheckAlt > 5 || VSL.VerticalSpeed.Absolute < -1) break;
					CFG.VerticalCutoff = VSL.Engines.Slow? -0.5f : -1f;
                    WideCheckAlt = -10;
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
					CFG.SmoothSetVSC((VSL.Engines.Slow? -0.5f : -1f)*Utils.ClampL(1-VSL.HorizontalSpeed, 0.1f), -1, 0);
				}
				CutoffTimer.Reset();
				break;
			default: 
				CFG.AP1.Off();
				break;
			}
		}

		#if DEBUG
		public void DrawDebugLines()
		{
			if(!CFG.AP1[Autopilot1.Land]) return;
			if(Nodes == null) return;
			if(scanner == null && NextNode != null)
			{
                Utils.GLLine(VSL.Physics.wCoM, NextNode.Pos.WorldPos(VSL.Body), Color.Lerp(Color.blue, Color.red, (float)NextNode.unevenness));
				return;
			}
			for(int i = 0; i < bside; i++)
				for(int j = 0; j < bside; j++)
				{
					var n = Nodes[i,j];
					if(n == null) continue;
                    Utils.GLLine(VSL.Physics.wCoM, n.Pos.WorldPos(VSL.Body), Color.Lerp(Color.blue, Color.red, (float)n.unevenness));
				}
		}

		void print_nodes()
		{
			if(Nodes == null) return;
			var nodes = string.Format("Nodes: {0}x{0}\n", bside);
			for(int i = 0; i < bside; i++)
				for(int j = 0; j < bside; j++)
                    nodes += string.Format("[{0},{1}]: {2}\n", i, j, Nodes[i,j]);
			Utils.Log(nodes+"\n");
		}
		#endif

        class SurfaceNode
        {
            internal class Comparer : IEqualityComparer<SurfaceNode>
            {
                readonly double threshold;

                public Comparer(double threshold) { this.threshold = threshold; }

                public bool Equals(SurfaceNode n1, SurfaceNode n2)
                { return n1.Pos.AngleTo(n2.Pos) < threshold; }

                public int GetHashCode(SurfaceNode n) 
                { return (int)(n.Pos.Lat/threshold) ^ (int)(n.Pos.Lon/threshold); }
            }

            public readonly CelestialBody Body;
            public Coordinates Pos;
            public double unevenness;
            public bool flat;
            public bool complete;
            public readonly  HashSet<SurfaceNode> neighbours = new HashSet<SurfaceNode>();

            public SurfaceNode(Vector3 pos, CelestialBody body)
            { 
                Pos = new Coordinates(pos, body);
                Body = body;
            }

            public SurfaceNode(WayPoint wp, CelestialBody body)
            { 
                Pos = wp.Pos;
                Body = body;
            }

            public WayPoint ToWayPoint()
            { return new WayPoint(Pos); }

            public double DistanceTo(VesselWrapper VSL)
            { return Pos.DistanceTo(VSL.vessel); }

            public void UpdateUnevenness(SurfaceNode n)
            {
                if(neighbours.Contains(n)) return;
                var unev = Math.Abs(Pos.Alt-n.Pos.Alt)/Pos.DistanceTo(n.Pos, Body);
                unevenness += unev;
                n.unevenness += unev;
                neighbours.Add(n);
                n.neighbours.Add(this);
                flat = unevenness < LND.MaxUnevenness;
            }

            public override string ToString()
            {
                return Utils.Format("{}, UNV: {}, flat: {}, complete: {}", Pos, unevenness, flat, complete);
            }
        }
	}
}

