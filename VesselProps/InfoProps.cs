//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2016 Allis Tauri
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ 
// or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.
//

using System.Collections.Generic;
using UnityEngine;
using AT_Utils;

namespace ThrottleControlledAvionics
{
    public class InfoProps : VesselProps
    {
        public InfoProps(VesselWrapper vsl) : base(vsl) {}

        public bool ElectricChargeAvailible
        {
            get
            {
                if(CheatOptions.InfiniteElectricity) return true;
                double amount, max_amount;
                vessel.GetConnectedResourceTotals(Utils.ElectricCharge.id, out amount, out max_amount);
                return amount > 0;
            }
        }

        public Vector3 Destination;
        public double Countdown = -1;
        public float  TTB = -1;

        public List<Vector3d> CustomMarkersVec = new List<Vector3d>();
        public List<WayPoint> CustomMarkersWP  = new List<WayPoint>();

        public void AddCustopWaypoint(Vector3d pos, string name = "Custom WayPoint")
        {
            var wp = new WayPoint(pos, VSL.vessel.mainBody);
            wp.Name = name;
            CustomMarkersWP.Add(wp);
        }

        public void AddCustopWaypoint(Coordinates pos, string name = "Custom WayPoint")
        {
            var wp = new WayPoint(pos);
            wp.Name = name;
            CustomMarkersWP.Add(wp);
        }

        public override void ClearFrameState()
        {
            CustomMarkersWP.Clear();
            CustomMarkersVec.Clear();
            Destination = Vector3.zero;
            Countdown = -1;
            TTB = -1;
        }

        public override void Update() {}

        public void Draw()
        {
            GUILayout.Label(new GUIContent(VSL.Info.Countdown >= 0? 
                                           string.Format("{0:F1}s", VSL.Info.Countdown) : "", 
                                           "Countdown" ),
                            VSL.Info.Countdown > 10? Styles.white : Styles.red, 
                            GUILayout.Width(90));
            GUILayout.Label(new GUIContent(VSL.Info.TTB >= 0 && VSL.Info.TTB < float.MaxValue? 
                                           string.Format("{0:F1}s", VSL.Info.TTB) : "",
                                           "Thrust Duration"), 
                            Styles.yellow, GUILayout.Width(90));
        }
    }
}

