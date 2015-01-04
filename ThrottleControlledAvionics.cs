/* Name:    Throttle Controlled Avionics
 * Version: 1.4.1  (KSP 0.90)
 * 
 * Author: Quinten Feys & Willem van Vliet
 * License: BY: Attribution-ShareAlike 3.0 Unported (CC BY-SA 3.0): http://creativecommons.org/licenses/by-sa/3.0/
 * 
 * 
 * Disclaimer: You use this at your own risk; this is an alpha plugin for an alpha game; if your computer disintegrates, it's not my fault. :P
 * Much thanks to everybody who's code I could look into
 * 
 */


//TODO: fix null pointer when engines are destroyed
//TODO: make the GUI better

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;
using KSP.IO;
using System.IO;

namespace ThrottleControlledAvionics
{
	[KSPAddon(KSPAddon.Startup.Flight, false)]
	public class ThrottleControlledAvionics : MonoBehaviour
	{
		#region variables
		
		public Vessel vessel;
		private bool enginesCounted;
		public List<EngineWrapper> engineTable;
		private Vector3 presentTorque;
		public Vector3 demand;
		public float minEfficiency;
		public Vector3 mainThrustAxis;     // as a pointing vector
		public enum thrustDirections { up, mean, custom };
		public thrustDirections direction;
		public Vector2 customDirectionVector;  // As (p, y)
		
		public bool isActive;
		private bool contUpdate;
		public bool detectStearingThrusters;
		public bool vtolMode;
		
		public SaveFile save;     
		private TCAGui gui;
		
		private Material diffuseMat;
		private LineRenderer upLine;
		private LineRenderer vecLine;
		private LineRenderer engLine;
		private LineRenderer dirLine;
		private LineRenderer gravLine;
		
		#endregion
		
		#region Unity interface
		
		public void Awake()  
		{			
			// open save
			save = new SaveFile();
			save.Load();

			// Initialising all variables
			vessel = null;			
			enginesCounted = false;
			engineTable = new List<EngineWrapper>();
			presentTorque = new Vector3(0f, 0f, 0f);
			demand = new Vector3(0f, 0f, 0f);
			minEfficiency = 0.66f;
			mainThrustAxis = Vector3.down;
			direction = thrustDirections.up;			
			isActive = false;
			contUpdate = true;
			detectStearingThrusters = false;
			
			diffuseMat = new Material (Shader.Find ("Particles/Additive")); //Unlit/Texture
			gui = new TCAGui (this);
		}
		
		/// <summary>
		/// Will be called every phisics tick. We check wether the vessel is acquired, list all the engines and calculate the engine limiters.
		/// </summary>
		public void FixedUpdate()
		{
			if (vessel == null)
			{
				vessel = FlightGlobals.ActiveVessel;
				Debug.Log("[TCA] acquired vessel: " + vessel);
			}
			
			if (contUpdate || !enginesCounted)
			{
				enginesCounted = true;
				List<EngineWrapper> engines = ListEngines();
				CalculateEngines(engines);
				
				mainThrustAxis = DetermineMainThrustAxis(engines); // relative to root part
				mainThrustAxis = corectForControlPodOrientation(mainThrustAxis, vessel.GetReferenceTransformPart().orgRot);
				engineTable = engines;
			}
			
			if (isActive && ElectricChargeAvailible(vessel))
			{
				if(!vtolMode) { //if in normal mode, trnaslate user input to steering and set thrust
					AjustDirection();
					SetThrottle();
				} else { //if in VTOL mode, zero velocity vector. Do this by faking the main thrust axis
					Vector3 temp = mainThrustAxis;
					Vector3 vesselVelocity = (vessel.situation == Vessel.Situations.FLYING)?vessel.GetSrfVelocity().normalized : vessel.GetObtVelocity().normalized;
					mainThrustAxis = vessel.transform.InverseTransformDirection(vesselVelocity); //fake it.
					AjustDirection();
					SetThrottle();
				}
			}
			
		}
		
		public void OnGUI()
		{
			gui.OnGUI ();
		}
		
		public void Update() {
			if (Input.GetKeyDown("y")) ActivateTCA(!isActive);
			if (Input.GetKeyDown(KeyCode.F2))
			{
				//gui.showHUD = !gui.showHUD; //hmmm... not quite, what is the hud was hidden already when we hit f2?
			}			
		}
		
		#endregion
		
		#region GUI
		
		/// <summary>
		/// This funtion will draw several lines to help seeing what is going on.
		/// 
		/// Red line: average thrust
		/// dark red line: Main Thrust Axis (as determined by the system or the user)
		/// white line: "up" considered by the vessel
		/// pink line: orbital or surface velocity
		/// </summary>
		/// 
		
		public void DrawDebugLines()
		{
			if (vessel != null){
				GameObject go;
				if(engLine == null){
					go = new GameObject();
					go.transform.parent = vessel.transform;
					go.transform.localPosition = Vector3.zero;
					go.transform.localRotation = Quaternion.identity;
					engLine = go.AddComponent<LineRenderer> ();
					engLine.SetColors(Color.red, new Color(1f,0f,0f,0f));
					engLine.SetWidth (-0.1f, 0.1f);
					engLine.useWorldSpace = false;
					engLine.material = diffuseMat;
				}
				if(dirLine == null){
					go = new GameObject();
					go.transform.parent = vessel.transform;
					go.transform.localPosition = Vector3.zero;
					go.transform.localRotation = Quaternion.identity;
					dirLine = go.AddComponent<LineRenderer> ();
					dirLine.SetColors(new Color(0.5f, 0f, 0f, 1f), new Color(0.5f,0f,0f,0f));
					dirLine.SetWidth (-0.1f, 0.1f);
					dirLine.useWorldSpace = false;
					dirLine.material = diffuseMat;
				}
				if(upLine == null){
					go = new GameObject();
					go.transform.parent = vessel.transform;
					go.transform.localPosition = Vector3.zero;
					go.transform.localRotation = Quaternion.identity;
					upLine = go.AddComponent<LineRenderer> ();
					upLine.SetColors(Color.white, new Color(1f,1f,1f,0f));
					upLine.SetWidth (-0.1f, 0.1f);
					upLine.useWorldSpace = false;
					upLine.material = diffuseMat;
				}
				if(gravLine == null){
					go = new GameObject();
					go.transform.parent = vessel.transform;
					go.transform.localPosition = Vector3.zero;
					go.transform.localRotation = Quaternion.identity;
					gravLine = go.AddComponent<LineRenderer> ();
					gravLine.SetColors(new Color(0.6f,0.4f,0.2f,1f), new Color(0.6f,0.4f,0.2f,0f));
					gravLine.SetWidth (-0.1f, 0.1f);
					gravLine.useWorldSpace = false;
					gravLine.material = diffuseMat;
					gravLine.SetPosition(0, Vector3.zero);
					gravLine.SetPosition(1, Vector3.forward * 10);
				}
				if(vecLine == null){
					go = new GameObject();
					vecLine = go.AddComponent<LineRenderer> ();
					vecLine.SetColors(new Color(0.7f,0.4f,0.8f,1f), new Color(0.7f,0.4f,0.8f,0f));
					vecLine.SetWidth (-0.1f, 0.1f);
					vecLine.useWorldSpace = true;
					vecLine.material = diffuseMat;
				}
				engLine.SetPosition (0, Vector3.zero);
				dirLine.SetPosition (0, Vector3.zero);
				upLine.SetPosition (0, Vector3.zero);
				vecLine.SetPosition (0, vessel.findWorldCenterOfMass());

				//set line targets:
				dirLine.SetPosition (1, mainThrustAxis.normalized * 10);
				Vector3 sum = new Vector3(0, 0, 0);
				foreach (EngineWrapper eng in engineTable)
				{
					sum += eng.maxThrust * eng.thrustVector;
				}
				engLine.SetPosition (1, sum.normalized * 10);
				upLine.SetPosition (1, Vector3.up * 10);
				vecLine.SetPosition (1, vessel.findWorldCenterOfMass() + ((vessel.situation == Vessel.Situations.FLYING)?vessel.GetSrfVelocity().normalized : vessel.GetObtVelocity().normalized) * 10); //GetSrfVelocity
				Vector3 gee = FlightGlobals.getGeeForceAtPosition( vessel.transform.position ); 
				gravLine.transform.rotation = Quaternion.LookRotation( gee.normalized );				
			}
		}
		
		#endregion
		
		
		#region Engine Logic
		
		/// <summary>
		/// This function activates or deactivates TCA. On deactivation, all engines are reset to max
		/// </summary>
		/// <param name="state">Whether TCA should be active or not</param>
		private void ActivateTCA(bool state)
		{
			isActive = state;
			if (!isActive) { SetAllEnginesMax(); }
		}
		
		/// <summary>
		/// Checks if the given vessel has any electric charge left
		/// </summary>
		/// <param name="v">The vessel</param>
		/// <returns>True if there is charge left</returns>
		public bool ElectricChargeAvailible(Vessel v)
		{
			List<Vessel.ActiveResource> ars = vessel.GetActiveResources();
			foreach (Vessel.ActiveResource ar in ars)
			{
				if (ar.info.name == "ElectricCharge" && ar.amount > 0) { return true; }
			}
			return false;
		}
		
		/// <summary>
		/// Lists all the engines of the active vessel. This function does not calculate any other thing.
		/// </summary>
		/// <returns></returns>
		private List<EngineWrapper> ListEngines()
		{
			vessel = FlightGlobals.ActiveVessel;
			List<EngineWrapper> engines = new List<EngineWrapper>();
			
			foreach (Part p in vessel.Parts)
			{
				EngineWrapper engine = null;
				foreach (PartModule module in p.Modules) {	
					if(module.moduleName == "ModuleEngines"){
						engine = new EngineWrapper((ModuleEngines) module);
						if (engine.isEnabled && !engine.throttleLocked){
							engines.Add(engine);
						}
					} else {
						if (module.moduleName == "ModuleEnginesFX")
						{
							engine = new EngineWrapper((ModuleEnginesFX) module);
							if (engine.isEnabled && !engine.throttleLocked){
								engines.Add(engine);
							}
						}
					}
				}				
			}
			return engines;
		}
		
		/// <summary>
		/// This function collects and calculates all the possibilities of every engine. It adds this information to the engineWrapper that are already listed.
		/// </summary>
		/// <param name="engines">A list of all the engine wrappers</param>
		private void CalculateEngines(List<EngineWrapper> engines)
		{
			//Debug.Log ("building engine table");
			foreach (EngineWrapper eng in engines)
			{
				float thrust = eng.maxThrust;
				Vector3 com = eng.vessel.findLocalCenterOfMass();				
				Vector3 r = eng.part.orgPos - com;
				Vector3 F = thrust * (eng.part.orgRot * Vector3.down) / vessel.GetTotalMass();
				Vector3 torque = Vector3.Cross(r, F);
				//Debug.Log (" F" + F.ToString() + "org" + eng.part.orgRot.ToString() + "down" + (eng.part.orgRot*Vector3.down).ToString() + "res:" + Vector3.Dot (eng.part.orgRot * Vector3.down, Vector3.down)); //we assume we want to go up. RCS support comes later);
				eng.steeringVector = torque;                            // The torque this engine can deliver represented as a rotation axis
				eng.thrustVector = (eng.part.orgRot * Vector3.down);    // The direction this engine is fireing in, measured form the down vector, relative form the root part
			}
		}
		
		/// <summary>
		/// This function determans the main thrust axis of the vessel. This can be preset or
		/// calculated based on the average direction of all the possible thrust.
		/// </summary>
		/// <param name="engineTable">The list of all the engineWrappers</param>
		/// <returns>The normalized avarage of all thrust, or preset directions</returns>
		private Vector3 DetermineMainThrustAxis(List<EngineWrapper> engineTable)
		{
			Destroy (null);
			switch (direction)
			{
			case thrustDirections.mean:
				Vector3 sum = new Vector3(0, 0, 0);
				foreach (EngineWrapper eng in engineTable)
				{
					sum += eng.maxThrust * eng.thrustVector;
				}
				return sum.normalized;
			case thrustDirections.up:
				return Vector3.down;
			case thrustDirections.custom:
				Vector3 ret;
				//x = cos(yaw)*cos(pitch);
				//y = sin(yaw)*cos(pitch);
				//z = sin(pitch);
				float pitchAngle = customDirectionVector.y * Mathf.Deg2Rad;
				float yawAngle = customDirectionVector.x * Mathf.Deg2Rad + Mathf.PI;
				ret.y = Mathf.Cos(yawAngle) * Mathf.Cos(pitchAngle);
				ret.z = Mathf.Sin(yawAngle) * Mathf.Cos(pitchAngle);
				ret.x = Mathf.Sin(pitchAngle);
				return ret;
			default:
				return Vector3.down;
			}
		}
		
		/// <summary>
		/// This function corrects the demanded rotation for the correct Command pod orientation. 
		/// </summary>
		/// <param name="thrustAxisBeforeCorection">The Orientation before the correction</param>
		/// <param name="OrientationControlPod">The Orientation of the control pod. (0,1,0) is straight up and needs no correction.</param>
		/// <returns></returns>
		private Vector3 corectForControlPodOrientation(Vector3 thrustAxisBeforeCorection, Quaternion OrientationControlPod)
		{
			return OrientationControlPod * thrustAxisBeforeCorection;
		}
		
		/// <summary>
		/// Reads the SAS
		/// </summary>
		private void AjustDirection()
		{
			FlightCtrlState ctrls = vessel.ctrlState;
			
			if (ctrls.pitch == 1 && demand.x >= 1) { demand.x += Time.fixedDeltaTime; }
			else if (ctrls.pitch == -1 && demand.x <= -1) { demand.x -= Time.fixedDeltaTime; }
			else { demand.x = ctrls.pitch; }
			
			if (ctrls.roll == 1 && demand.y >= 1) { demand.y += Time.fixedDeltaTime; }
			else if (ctrls.roll == -1 && demand.y <= -1) { demand.y -= Time.fixedDeltaTime; }
			else { demand.y = ctrls.roll; }
			
			if (ctrls.yaw == 1 && demand.z >= 1) { demand.z += Time.fixedDeltaTime; }
			else if (ctrls.yaw == -1 && demand.z <= -1) { demand.z -= Time.fixedDeltaTime; }
			else { demand.z = ctrls.yaw; }
			
			//demand = new Vector3(ctrls.pitch, ctrls.roll, ctrls.yaw);
		}

		/// <summary>
		/// Here is the thrust modifier of every engine calculated. This is based on how much steering they provide and how efficient they can thrust along the main thruxt axis.
		/// </summary>
		private void SetThrottle()
		{
			foreach (EngineWrapper eng in engineTable)
			{
				float steering = Vector3.Dot(
					Quaternion.AngleAxis(eng.steeringVector.magnitude, eng.steeringVector).eulerAngles,
					-demand
					);  // The amount this engine can fulfill the torque demand
				steering = steering / 100;  // Because the numbers were to high
				eng.steering = steering;
				
				float efficiency = Vector3.Dot(eng.thrustVector, mainThrustAxis);
				eng.efficiency = efficiency; // The amount this engine is pointing along the main thrust axis
				
				
				float throttle;                                          // The amount of throttle this engine has to deliver

				if (detectStearingThrusters)
				{
					if (efficiency > minEfficiency) {
						if(vtolMode) { //if in VTOL mode, we only use the engines for steering and counteracting gravity or old momentum.
							if(Math.Abs(vessel.obt_speed)>2f) { //since obt_speed is an double, we use Math instead of Mathf. weird.
								throttle = save.GetActiveMeanThrust() * efficiency;
							} else {
								throttle = 0f;
							}
						} else {
							throttle = save.GetActiveMeanThrust(); }
						}
					else { throttle = 0f; }
				}
				else { throttle = save.GetActiveMeanThrust(); }
			
				
				throttle += steering * save.GetActiveSensitivity();     //Adding the steering
				throttle = Mathf.Clamp(throttle, 0f, 100f);
				eng.thrustPercentage = throttle;
			}
		}
		
		
		private void SetAllEnginesMax()
		{
			foreach (EngineWrapper eng in engineTable)
			{
				eng.thrustPercentage = 100;
			}
		}
		
		#endregion
		
		internal void OnDestroy()
		{
			save.Save();
			gui.Destroy ();
		}
		
		
		// if other mods want to use the mod
		#region interface
		
		public bool IsActive()
		{
			return isActive;
		}
		
		public void SetActive(bool setting)
		{
			isActive = setting;
		}
		
		#endregion
		
		// used for debugging
		private void writeToFile(String text)
		{
			using (StreamWriter writer = new System.IO.StreamWriter("DumpFile.txt",true))
			{
				writer.Write(text+" \r\n");
			}
		}
		
		
		
	}
	
	public class SaveFile
	{
		int activeSave;
		object[,] saves;
		
		public SaveFile()
		{
			saves = new object[3,3];
			activeSave = 0;
		}
		
		public void Save()
		{
			String text = (String)saves[0, 0] + "\r\n" + (float)saves[0, 1] + "\r\n" + (float)saves[0, 2] + "\r\n\r\n" +
				(String)saves[1, 0] + "\r\n" + (float)saves[1, 1] + "\r\n" + (float)saves[1, 2] + "\r\n\r\n" +
					(String)saves[2, 0] + "\r\n" + (float)saves[2, 1] + "\r\n" + (float)saves[2, 2] + "\r\n\r\n";
			using (StreamWriter writer = new System.IO.StreamWriter("GameData/ThrottleControlledAvionics/TCASave.txt", false))
			{
				writer.Write(text);
			}
		}
		
		public void Load()
		{
			try
			{
				using (StreamReader reader = new System.IO.StreamReader("GameData/ThrottleControlledAvionics/TCASave.txt"))
				{
					for (int i = 0; i < 3; i++)
					{
						saves[i, 0] = reader.ReadLine();
						saves[i, 1] = float.Parse(reader.ReadLine());
						saves[i, 2] = float.Parse(reader.ReadLine());
						reader.ReadLine();
					}
				}
			}
			catch (Exception e)            
			{                
				if (e is System.IO.FileNotFoundException || e is System.IO.IsolatedStorage.IsolatedStorageException)
				{
					SetDefault();
				}
				else{
					writeToFile("We found a serous problem during Load:" + e);
					throw;
				}
			}
			
		}
		
		private void SetDefault()
		{
			for (int i = 0; i < 3; i++)
			{
				writeToFile("i = " + i);
				saves[i, 0] = "Setting " + i;
				saves[i, 1] = 1f;
				saves[i, 2] = 100f;
			}
		}
		
		#region setters
		public void SetActiveSave(int i)
		{
			activeSave = i;
		}
		
		public void SetActiveName(string name)
		{
			saves[activeSave, 0] = name;
		}
		
		public void SetActiveSensitivity(float tf)
		{
			saves[activeSave, 1] = tf;
		}
		
		public void SetActiveMeanThrust(float mt)
		{
			saves[activeSave, 2] = mt;
		}
		#endregion
		
		#region getters
		public string GetActiveName()
		{
			return (string)saves[activeSave, 0];
		}
		
		public string GetName(int i)
		{
			return (string)saves[i,0];
		}
		
		public float GetActiveSensitivity()
		{
			return (float)saves[activeSave, 1];
		}
		
		public float GetActiveMeanThrust()
		{
			return (float)saves[activeSave, 2];
		}
		#endregion
		
		// used for debugging
		private void writeToFile(String text)
		{
			using (StreamWriter writer = new System.IO.StreamWriter("DumpFile.txt", true))
			{
				writer.Write(text + " \r\n");
			}
		}
	}
}
