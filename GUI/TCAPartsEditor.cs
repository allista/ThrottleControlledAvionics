//   TCAPartsEditor.cs
//
//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2017 Allis Tauri

using System;
using System.Collections.Generic;
using UnityEngine;
using AT_Utils;

namespace ThrottleControlledAvionics
{
    public class TCAPartsEditor : GUIWindowBase
    {
        class PrettyNode : TCAPartGraphNode
        {
            GUIContent button;
            public Rect button_rect;
            public bool Editable = true;

            public override void SetPart(TCAPart part)
            {
                base.SetPart(part);
                //remove Module from module title. Maybe drop it altogether from the TechTree?
                button = new GUIContent(part.Title.Replace("Module", "").Trim(), 
                                        //remove the "Updates TCA (tm)..." sentence.
                                        part.Description.Substring(part.Description.IndexOf('.')+1));
            }

            public bool Draw(VesselConfig CFG)
            {
                var button_pressed = false;
                if(CFG.EnabledTCAParts.Contains(part.Name))
                {
                    if(!part.Active)
                        GUILayout.Label(button, Styles.inactive_button);
                    else if(GUILayout.Button(button, Styles.enabled_button, GUILayout.ExpandWidth(true)) && Editable)
                    {
                        CFG.EnabledTCAParts.Remove(part.Name);
                        button_pressed = true;
                    }
                }
                else
                {
                    if(GUILayout.Button(button, Styles.active_button, GUILayout.ExpandWidth(true)) && Editable)
                    {
                        CFG.EnabledTCAParts.Add(part.Name);
                        button_pressed = true;
                    }
                }
                if(Event.current.type == EventType.Repaint)
                    button_rect = GUILayoutUtility.GetLastRect();
                return button_pressed;
            }
        }

        class PrettyPartGraph : TCAPartGraph<PrettyNode>
        {
            public bool Draw(VesselConfig CFG)
            {
                var button_pressed = false;
                GUILayout.BeginHorizontal();
                foreach(var tier in tiers.Keys)
                {
                    GUILayout.BeginVertical(Styles.white, GUILayout.ExpandHeight(true));
                    GUILayout.Label("Tier "+tier, GUILayout.ExpandWidth(true));
                    tiers[tier].ForEach(node => button_pressed = node.Draw(CFG) || button_pressed);
                    GUILayout.EndVertical();
                    if(tier < tiers.Count) GUILayout.Space(10);
                }
                GUILayout.EndHorizontal();
                return button_pressed;
            }

            public void SetEditable(bool editable)
            { tiers.ForEach(tier => tier.Value.ForEach(node => node.Editable = editable)); }
        }

        VesselConfig CFG;
        List<TCAPart> parts;
        PrettyPartGraph graph;

        public TCAPartsEditor()
        {
            width = 450;
            height = 300;
        }

        public override void Awake()
        {
            base.Awake();
            update_part_status();
            graph = TCAPartGraph<PrettyNode>.BuildGraph<PrettyPartGraph>(parts);
            if(HighLogic.LoadedSceneIsFlight) graph.SetEditable(false);
        }

        void update_part_status()
        {
            if(parts == null)
                parts = TCAModulesDatabase.GetPurchasedParts();
            parts.ForEach(p => p.UpdateInfo(CFG));
            if(HighLogic.LoadedSceneIsEditor)
                TCAGuiEditor.UpdateModules();
        }

        public override void Show(bool show)
        {
            base.Show(show);
            if(show) update_part_status();
        }

        public void SetCFG(VesselConfig cfg)
        {
            CFG = cfg;
            update_part_status();
        }

        protected override bool can_draw() { return CFG != null; }

        void MainWindow(int windowID)
        {
            GUILayout.BeginVertical();
            GUILayout.BeginVertical();
            if(graph.Draw(CFG)) update_part_status();
            GUILayout.EndVertical();
            if(!HighLogic.LoadedSceneIsFlight) 
            {
                if(GUILayout.Button(new GUIContent("Enable All", "Enable all disabled modules"), 
                                    Styles.active_button, GUILayout.ExpandWidth(true)))
                {
                    parts.ForEach(p => CFG.EnabledTCAParts.Add(p.Name));
                    update_part_status();
                }
            }
            if(GUILayout.Button("Close", Styles.close_button, GUILayout.ExpandWidth(true))) Show(false);
            GUILayout.EndVertical();
            TooltipsAndDragWindow();
        }

        public void Draw()
        {
            if(doShow)
            {
                LockControls();
                WindowPos = GUILayout.Window(GetInstanceID(), 
                                             WindowPos, MainWindow,
                                             HighLogic.LoadedSceneIsFlight?
                                             "Installed TCA Modules" :
                                             "Select TCA Modules", 
                                             GUILayout.Width(width),
                                             GUILayout.Height(height)).clampToScreen();
            }
            else UnlockControls();
        }

        static Color inactive = new Color(0.8f, 0.8f, 0.8f);
        void OnGUI()
        {
            GUI.depth = -1;
            if(doShow)
            {
                //draw dependecies
                if(Event.current.type == EventType.Repaint)
                {
                    foreach(var tier in graph.tiers)
                    {
                        var next_tier = tier.Key+1;
                        foreach(var node in tier.Value)
                        {
                            var sr = node.button_rect;
                            var start = new Vector2(WindowPos.x+sr.x+sr.width, WindowPos.y+sr.y+sr.height/2);
                            foreach(var next in node.outputs)
                            {
                                if(next.tier != next_tier) continue;
                                var er = (next as PrettyNode).button_rect;
                                var end = new Vector2(WindowPos.x+er.x, WindowPos.y+er.y+sr.height/2);
                                Drawing.DrawLine(start, end, next.part.Active? Color.green : inactive, 1, true);
                            }
                        }
                    }
                }
            }
        }
    }
}

