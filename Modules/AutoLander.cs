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

			[Persistent] public float MaxUnevenness = 0.1f;
			[Persistent] public float MinHorizontalSpeed = 0.5f;

		}
		static Config LND { get { return TCAConfiguration.Globals.LND; } }

		enum Mode { None, Moving, Stopping, ChangingAltitude, Landing }

		static   int RadarMask = (1 << 15 | 1 << LayerMask.NameToLayer("Parts"));

		readonly Multiplexer<Mode> mode = new Multiplexer<Mode>();
		SurfaceNode[,] Nodes;
		int side, bside, center;
		Vector3 right, fwd, up, down;
		float MaxDistance;

		public AutoLander(VesselWrapper vsl) { VSL = vsl; }

		public override void UpdateState() { IsActive = CFG.AP[Autopilot.Land] && VSL.OnPlanet; }

		SurfaceNode get_surface_node(Vector3 dir)
		{
			RaycastHit raycastHit;
			if(Physics.Raycast(VSL.wCoM, dir, out raycastHit, MaxDistance, RadarMask))
				return new SurfaceNode(raycastHit.distance*dir+VSL.wCoM, up);
			return null;
		}

		IEnumerator scan_surface(int lvl, float delta)
		{
			if(Nodes != null || VSL.refT == null) yield break;
			//initialize the system
			side   = lvl*2-1;
			bside  = side+2;
			center = bside/2;
			up     = VSL.Up;
			down   = -VSL.Altitude*up;
			right  = VSL.refT.right;
			fwd    = Vector3.Cross(right, up);
			Nodes  = new SurfaceNode[bside,bside];
			//cast the rays
			MaxDistance = VSL.Altitude * 10;
			for(int i = 0; i < bside; i++)
				for(int j = 0; j < bside; j++)
				{
					Nodes[i,j] = get_surface_node((down+right*(i-center)*delta+fwd*(j-center)*delta).normalized);
					yield return null;
				}
			//compute unevenness
			for(int i = 1; i < side; i++)
				for(int j = 1; j < side; j++)
				{
					var n = Nodes[i,j];
					for(int u = i-1; u < i+1; u += 2)
						for(int v = j-1; v < j+1; v += 2)
							n.UpdateUnevenness(Nodes[u,v]);
					yield return null;
				}
		}

		public void Update()
		{
			if(!IsActive) return;
			//if just started, kill horizontal speed
			if(!mode) { CFG.HF.On(HFlight.Stop); mode.On(Mode.Stopping); return; }
			if(mode[Mode.Stopping] && VSL.HorizontalSpeed > LND.MinHorizontalSpeed) return;
			//if too high, fly lower
			//check right under the vessel
		}

		class SurfaceNode
		{
			Vector3 up;
			public Vector3 position;
			public float unevenness;
			public readonly  HashSet<SurfaceNode> neighbours = new HashSet<SurfaceNode>();

			public SurfaceNode(Vector3 pos, Vector3 up) 
			{ position = pos; this.up = up; }

			public void UpdateUnevenness(SurfaceNode n)
			{
				if(neighbours.Contains(n)) return;
				var unev = Mathf.Abs(Vector3.Dot((n.position-position).normalized, up));
				unevenness += unev;
				n.unevenness += unev;
				neighbours.Add(n);
				n.neighbours.Add(this);
			}
		}
	}
}

