/* Name: Throttle Controlled Avionics, Fork by Allis Tauri
 *
 * Authors: Quinten Feys & Willem van Vliet & Allis Tauri
 * License: BY: Attribution-ShareAlike 3.0 Unported (CC BY-SA 3.0): 
 * http://creativecommons.org/licenses/by-sa/3.0/
 * 
 */

using System;
using System.IO;

namespace ThrottleControlledAvionics
{
	public class SaveFile
	{
		int activeSave;
		readonly object[,] saves;

		public SaveFile()
		{
			saves = new object[3, 3];
			activeSave = 0;
		}

		public void Save()
		{
			String text = (String)saves[0, 0] + "\n" + (float)saves[0, 1] + "\n" + (float)saves[0, 2] + "\n\n" +
				(String)saves[1, 0] + "\n" + (float)saves[1, 1] + "\n" + (float)saves[1, 2] + "\n\n" +
				(String)saves[2, 0] + "\n" + (float)saves[2, 1] + "\n" + (float)saves[2, 2] + "\n\n";
			using(var writer = new StreamWriter("GameData/ThrottleControlledAvionics/TCASave.txt", false))
				writer.Write(text);
		}

		public void Load()
		{
			try
			{
				using(var reader = new StreamReader("GameData/ThrottleControlledAvionics/TCASave.txt"))
				{
					for(int i = 0; i < 3; i++)
					{
						saves[i, 0] = reader.ReadLine();
						saves[i, 1] = float.Parse(reader.ReadLine());
						saves[i, 2] = float.Parse(reader.ReadLine());
						reader.ReadLine();
					}
				}
			}
			catch(Exception e)
			{                
				if(e is FileNotFoundException || e is System.IO.IsolatedStorage.IsolatedStorageException)
					SetDefault();
				else
				{
					Utils.writeToFile("We found a serous problem during Load:" + e);
					throw;
				}
			}
		}

		void SetDefault()
		{
			for(int i = 0; i < 3; i++)
			{
				Utils.writeToFile("i = " + i);
				saves[i, 0] = "Setting " + i;
				saves[i, 1] = 1f;
				saves[i, 2] = 100f;
			}
		}

		#region setters
		public void SetActiveSave(int i) { activeSave = i; }

		public void SetActiveName(string name) { saves[activeSave, 0] = name; }

		public void SetActiveSensitivity(float tf) { saves[activeSave, 1] = tf; }

		public void SetActiveMeanThrust(float mt) { saves[activeSave, 2] = mt; }
		#endregion

		#region getters
		public string GetActiveName() { return (string)saves[activeSave, 0]; }

		public string GetName(int i) { return (string)saves[i, 0]; }

		public float GetActiveSensitivity() { return (float)saves[activeSave, 1]; }

		public float GetActiveMeanThrust() { return (float)saves[activeSave, 2]; }
		#endregion
	}
}
