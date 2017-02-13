//   ModuleTester.cs
//
//  Author:
//       Allis Tauri <allista@gmail.com>
//
//  Copyright (c) 2017 Allis Tauri
#if DEBUG
using System;
using System.Linq;
using System.Collections.Generic;
using System.Reflection;
using UnityEngine;
using AT_Utils;

namespace ThrottleControlledAvionics
{
    public class ModuleTester : TCAModule
    {
        delegate void Test(System.Random rnd);
        SortedList<string, Test> tests = new SortedList<string, Test>();
        string current_module = "";
        Test current_test;
        System.Random rnd;

        public ModuleTester(ModuleTCA tca) : base(tca)
        {
        }

        public override void Init()
        {
            base.Init();
            tests.Clear();
            foreach(var m in TCA.AllModules)
            {
                var mtype = m.GetType();
                var test_info = mtype.GetMethod("Test");
                if(test_info != null) 
                    tests.Add(mtype.Name, (Test)Delegate.CreateDelegate(typeof(Test), m, test_info));
            }
            current_module = tests.Keys.FirstOrDefault() ?? "";
        }

        protected override void Update()
        {
            if(current_test == null) return;
            current_test(rnd);
        }

        public override void Draw()
        {
            if(tests.Count == 0) return;
            GUILayout.BeginVertical();
            current_module = Utils.LeftRightChooser(current_module, tests, "Select module to test");
            if(current_test != null)
            {
                if(GUILayout.Button("Stop the test", Styles.danger_button, GUILayout.ExpandWidth(true)))
                {
                    current_test = null;
                    rnd = null;
                }
            }
            else if(GUILayout.Button("Run the test", Styles.active_button, GUILayout.ExpandWidth(true)))
            {
                if(tests.TryGetValue(current_module, out current_test))
                    rnd = new System.Random(DateTime.Now.Second);
                else current_test = null;
            }
            GUILayout.EndVertical();
        }
    }
}
#endif
