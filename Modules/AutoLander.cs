//   AutoLander.cs
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
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace ThrottleControlledAvionics
{
	public class AutoLander : TCAModule
	{
		public class Config : ModuleConfig
		{
			new public const string NODE_NAME = "LND";

			[Persistent] public float MaxUnevenness        = 0.1f;
			[Persistent] public float MaxHorizontalTime    = 5f;
			[Persistent] public float MinVerticalSpeed     = 0.1f;
			[Persistent] public float WideCheckAltitude    = 200f;
			[Persistent] public float MaxWideCheckAltitude = 1000f;
			[Persistent] public int   WideCheckLevel       = 5;
			[Persistent] public float NodeTargetRange      = 15;
			[Persistent] public float NodeAnchorF          = 0.5f;
			[Persistent] public float LandingAlt           = 10;
			[Persistent] public float LandingFinalAlt      = 2;
			[Persistent] public float StopTimer            = 2;
			[Persistent] public float CutoffTimer          = 2;

			public float MaxStartAltitude;

			public override void Init ()
			{
				base.Init ();
				MaxStartAltitude = MaxWideCheckAltitude/2;
			}
		}
		static Config LND { get { return TCAScenario.Globals.LND; } }

		static int RadarMask = (1 << 15 | 1 << LayerMask.NameToLayer("Parts") | 1);
		enum Stage { None, Start, PointCheck, WideCheck, FlatCheck, MoveNext, Land }

		Stage stage;
		IEnumerator scanner;
		SurfaceNode[,] Nodes;
		readonly List<SurfaceNode> FlatNodes = new List<SurfaceNode>();
		SurfaceNode NextNode, FlattestNode;
		int side, bside, center;
		Vector3 right, fwd, up, down, dir, sdir;
		float MaxDistance, delta;
		float DesiredAltitude;
		readonly Timer StopTimer = new Timer();
		readonly Timer CutoffTimer = new Timer();

		public AutoLander(VesselWrapper vsl) { VSL = vsl; }

		public override void Init()
		{
			base.Init();
			StopTimer.Period = LND.StopTimer;
			CutoffTimer.Period = LND.CutoffTimer;
			CFG.AP.AddCallback(Autopilot.Land, Enable);
			#if DEBUG
			RenderingManager.AddToPostDrawQueue(1, RadarBeam);
			#endif
		}

		public override void UpdateState() 
		{ 
			IsActive = CFG.AP[Autopilot.Land] && VSL.OnPlanet;
			if(!IsActive) return;
		}

		public override void Enable(bool enable = true)
		{
			stage = Stage.None;
			CFG.HF.On(HFlight.Stop);
			scanner = null;
			NextNode = null;
			Nodes = null;
			StopTimer.Reset();
			CutoffTimer.Reset();
			if(enable) DesiredAltitude = 0;
			else
			{
				DesiredAltitude = VSL.Altitude;
				CFG.VF.On(VFlight.AltitudeControl);
				CFG.Nav.Off();
			}
		}

		SurfaceNode get_surface_node()
		{
			RaycastHit raycastHit;
			var c = VSL.C+dir*(VSL.R+0.1f);
			if(Physics.Raycast(c, dir, out raycastHit, MaxDistance, RadarMask))
			{
				var ray = raycastHit.distance*dir;
				if(VSL.mainBody.ocean && 
				   (c+ray-VSL.mainBody.position).magnitude < VSL.mainBody.Radius)
					return null;
				return new SurfaceNode(c, ray, up);
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
			up     = VSL.Up;
			down   = -VSL.Altitude*up;
			right  = VSL.refT.right;
			fwd    = Vector3.Cross(right, up);
			Nodes  = new SurfaceNode[bside,bside];
			delta  = d > 0? d : DesiredAltitude/bside*lvl;
			if(start == null)
			{
				sdir = down;
				MaxDistance = VSL.Altitude * 10;
			}
			else 
			{
				sdir = start.position-VSL.wCoM;
				MaxDistance = sdir.magnitude * 10;
			}
//			Utils.Log("Scanning Surface: {0}x{0} -> {1}x{1}", bside, side);//debug
			yield return null;
			//cast the rays
			for(int i = 0; i < bside; i++)
				for(int j = 0; j < bside; j++)
				{
					dir = (sdir+right*(i-center)*delta+fwd*(j-center)*delta).normalized;
					Nodes[i,j] = get_surface_node();
					yield return null;
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
					yield return null;
				}
			if(lvl > 1)
			{
				FlatNodes.Clear();
				for(int i = 1; i <= side; i++)
					for(int j = 1; j <= side; j++)
					{
						var n = Nodes[i,j];
						if(n != null && n.flat)
							FlatNodes.Add(n);
					}
				yield return null;
				FlatNodes.Sort((n1, n2) => n1.distance.CompareTo(n2.distance));
			}
//			print_nodes();//debug
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
						if(flattest == null) flattest = n;
						else if(flattest.unevenness > n.unevenness)
							flattest = n;
					}
				return flattest;
			}
		}

		SurfaceNode center_node
		{ get { return Nodes == null ? null : Nodes[center, center]; } }

		bool altitude_changed
		{
			get
			{
				CFG.AltitudeAboveTerrain = true;
				if(DesiredAltitude <= 0) 
				{
					VSL.UpdateAltitude();
					DesiredAltitude = VSL.Altitude;
				}
				CFG.DesiredAltitude = DesiredAltitude;
				var err = Mathf.Abs(VSL.Altitude-DesiredAltitude);
				if(err > 10)
				{
					CFG.VF.OnIfNot(VFlight.AltitudeControl);
					return false;
				}
				else if(err < 5)
				{
					CFG.VF.OffIfOn(VFlight.AltitudeControl);
					CFG.VerticalCutoff = 0;
				}
				return Mathf.Abs(VSL.VerticalSpeed) < LND.MinVerticalSpeed;
			}
		}

		bool stopped
		{
			get
			{
				CFG.HF.OnIfNot(HFlight.Stop);
				if(VSL.R/VSL.HorizontalSpeed > LND.MaxHorizontalTime)
					return StopTimer.Check;
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
				if(CFG.Anchor == null) { CFG.AP.Off(); return false; }
				if(CFG.Anchor.DistanceTo(VSL.vessel) < CFG.Anchor.Distance)
				{
					CFG.Nav.Off();
					if(NextNode.flat)
					{
						CFG.HF.OnIfNot(HFlight.Anchor);
						if(CFG.Anchor.DistanceTo(VSL.vessel) < VSL.R*LND.NodeAnchorF)
							return StopTimer.Check;
					}
					else return true;
				}
				else if(!CFG.HF[HFlight.Anchor] && 
				        (!CFG.Nav[Navigation.GoToTarget] || VSL.vessel.targetObject != CFG.Anchor))
				{
					SetTarget(CFG.Anchor);
					CFG.Nav.On(Navigation.GoToTarget);
				}
				StopTimer.Reset();
				return false;
			}
		}

		float distance_to_node(SurfaceNode n)
		{ return Vector3.ProjectOnPlane(n.position-VSL.wCoM, VSL.Up).magnitude; }

		void wide_check(float delta_alt = 0)
		{
			if(DesiredAltitude < LND.WideCheckAltitude)
				DesiredAltitude = LND.WideCheckAltitude;
			DesiredAltitude += delta_alt;
			if(DesiredAltitude > LND.MaxWideCheckAltitude)
			{
				ScreenMessages.PostScreenMessage("Unable to find suitale place for landing",
				                                 5, ScreenMessageStyle.UPPER_CENTER);
				CFG.AP.Off();
			}
			else stage = Stage.WideCheck;
		}

		void try_move_to_flattest()
		{
			NextNode = FlattestNode;
			if(NextNode != null && distance_to_node(NextNode) > LND.NodeTargetRange*2) move_next();
			else wide_check(LND.WideCheckAltitude);
		}

		void move_next()
		{
			CFG.Anchor = new WayPoint(VSL.mainBody.GetLatitude(NextNode.position),
			                          VSL.mainBody.GetLongitude(NextNode.position));
			CFG.Anchor.Distance = LND.NodeTargetRange;
			stage = Stage.MoveNext;
		}

		void land()
		{
			CFG.HF.Off();
			var c = center_node;
			CFG.Anchor = new WayPoint(VSL.mainBody.GetLatitude(c.position),
			                          VSL.mainBody.GetLongitude(c.position));
			CFG.Anchor.Distance = LND.NodeTargetRange;
			CFG.HF.On(HFlight.Anchor);
			DesiredAltitude = LND.LandingAlt+VSL.H;
			stage = Stage.Land;
		}

		bool scan(int lvl, SurfaceNode start = null, float d = 0)
		{
			if(scanner == null) scanner = surface_scanner(lvl, start, d);
			if(scanner.MoveNext()) return true;
			scanner = null;
			return false;
		}

		void gear_on()
		{
			if(!VSL.ActionGroups[KSPActionGroup.Gear])
				VSL.ActionGroups.SetGroup(KSPActionGroup.Gear, true);
		}

		public void Update()
		{
			if(!IsActive) return;
			switch(stage)
			{
			case Stage.None:
				CFG.AltitudeAboveTerrain = true;
				if(DesiredAltitude <= 0) 
				{   //here we just need the altitude control to prevent smashing into something while stopping
					VSL.UpdateAltitude();
					DesiredAltitude = VSL.Altitude > LND.MaxStartAltitude? LND.MaxStartAltitude : VSL.Altitude;
					CFG.DesiredAltitude = DesiredAltitude;
				}
				CFG.VF.OnIfNot(VFlight.AltitudeControl);
				if(stopped && VSL.Altitude < LND.MaxStartAltitude+10) 
				{
					DesiredAltitude = VSL.Altitude;
					stage = Stage.PointCheck;	
				}
				break;
			case Stage.PointCheck:
				SetState(TCAState.CheckingSite);
				if(!fully_stopped) break;
				if(scan(1, null, VSL.R*2)) break;
				if(Nodes == null || center_node == null || 
				   !center_node.flat) wide_check();
				else land();
				break;
			case Stage.FlatCheck:
				SetState(TCAState.Scanning);
				if(!fully_stopped) break;
				if(FlatNodes.Count > 0)
				{
					if(scan(1, FlatNodes[0], VSL.R*2)) break;
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
				SetState(TCAState.Scanning);
				if(!fully_stopped) break;
				if(scan(LND.WideCheckLevel)) break;
				FlattestNode = flattest_node;
				if(FlatNodes.Count > 0) 
				{ stage = Stage.FlatCheck; break; }
				else try_move_to_flattest();
				break;
			case Stage.MoveNext:
				SetState(NextNode.flat? TCAState.CheckingSite : TCAState.Searching);
				if(!moved_to_next_node) break;
				DesiredAltitude = VSL.Altitude;
				if(NextNode.flat) stage = Stage.PointCheck;
				else wide_check();
				break;
			case Stage.Land:
				SetState(TCAState.Landing);
				if(DesiredAltitude > 0)
				{
					CFG.HF.OnIfNot(HFlight.Anchor);
					DesiredAltitude = LND.LandingAlt+VSL.H;
					if(VSL.Altitude < LND.LandingAlt+VSL.H*2) gear_on();
					if(VSL.Altitude > DesiredAltitude &&
					   !altitude_changed) break;
					DesiredAltitude = -10;
				}
				gear_on();
				CFG.VF.Off();
				if(VSL.LandedOrSplashed) 
				{ 
					if(!CutoffTimer.Check) break;
					CFG.AP.Off(); 
					CFG.VerticalCutoff = -10; 
					CFG.VF.On(VFlight.AltitudeControl);
				}
				else
				{
					if(VSL.Altitude > LND.LandingFinalAlt+VSL.H) 
						CFG.HF.OnIfNot(HFlight.Anchor);
					else CFG.HF.OnIfNot(HFlight.Stop);
					set_VSpeed(VSL.SlowEngines? -0.5f :
					           Mathf.Lerp(-0.5f, -1, VSL.Altitude/(LND.LandingAlt+VSL.H)));
				}
				CutoffTimer.Reset();
				break;
			default: 
				CFG.AP.Off();
				break;
			}
		}

		void set_VSpeed(float val)
		{ CFG.VerticalCutoff = Mathf.Lerp(CFG.VerticalCutoff, val, TimeWarp.fixedDeltaTime); }

		#if DEBUG
		public void RadarBeam()
		{
			if(Nodes == null) return;
			if(scanner == null && NextNode != null)
			{
				GLUtils.GLTriangleMap(new Vector3[] { VSL.wCoM-VSL.refT.right*0.1f, VSL.wCoM+VSL.refT.right*0.1f, NextNode.position }, 
				                      Color.Lerp(Color.blue, Color.red, NextNode.unevenness));
				return;
			}
			for(int i = 0; i < bside; i++)
				for(int j = 0; j < bside; j++)
				{
					var n = Nodes[i,j];
					if(n == null) continue;
					GLUtils.GLTriangleMap(new Vector3[] { VSL.wCoM-VSL.refT.right*0.1f, VSL.wCoM+VSL.refT.right*0.1f, n.position }, 
					                      Color.Lerp(Color.blue, Color.red, n.unevenness));
				}
		}

		public override void Reset()
		{
			base.Reset();
			RenderingManager.RemoveFromPostDrawQueue(1, RadarBeam);
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
			Vector3 up;
			public Vector3 position;
			public float unevenness;
			public float distance;
			public bool flat;
			public readonly  HashSet<SurfaceNode> neighbours = new HashSet<SurfaceNode>();

			public SurfaceNode(Vector3 ori, Vector3 ray, Vector3 up) 
			{ 
				distance = Vector3.ProjectOnPlane(ray, up).magnitude; 
				position = ori+ray; 
				this.up = up; 
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

