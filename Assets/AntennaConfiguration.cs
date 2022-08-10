using UnityEngine;
using System.Collections.Generic;

#if UNITY_EDITOR
using UnityEditor;
#endif

public class AntennaConfiguration : MonoBehaviour
{


    [HideInInspector]
    public List<GameObject> antennas = new List<GameObject>();
    [HideInInspector] 
    public List<TextAsset> radiation_patterns = new List<TextAsset>();
    
    bool showAntennaList = true;

    public bool radiationPatternActive = false;

    
    int radiation_pattern_button_counter = 0;





 #region Editor
#if UNITY_EDITOR
    [CustomEditor(typeof(AntennaConfiguration))]
    public class AntennaConfigurationEditor : Editor
    {
        public override void OnInspectorGUI()
        {
            base.OnInspectorGUI();

            AntennaConfiguration antennaConfiguration = (AntennaConfiguration)target;
            EditorGUILayout.Space();

            AntennaArrayDefinitionFunc(antennaConfiguration);
            

            

            
        }



        static void AntennaArrayDefinitionFunc(AntennaConfiguration antennaConfiguration)
        {
            
            
            GUIStyle myFoldoutStyle = new GUIStyle(EditorStyles.foldout);
            Color myStyleColor = Color.blue;
            myFoldoutStyle.fontStyle = FontStyle.Bold;
            myFoldoutStyle.normal.textColor = myStyleColor;

            //antennaConfiguration.showAntennaList = EditorGUILayout.Foldout(antennaConfiguration.showAntennaList, "Antenna array information", false);
            antennaConfiguration.showAntennaList = EditorGUILayout.Foldout(antennaConfiguration.showAntennaList, "Antenna array information", false, myFoldoutStyle);
            

            if (antennaConfiguration.showAntennaList)
            {
                EditorGUI.indentLevel++;
                // size
                List<GameObject> antennaArray = antennaConfiguration.antennas;
                int AntennaNumber = Mathf.Max(1, EditorGUILayout.IntField("Number of antennas", antennaArray.Count));

                // correct size
                while (AntennaNumber > antennaArray.Count)
                {
                    antennaArray.Add(null);
                }
                while (AntennaNumber < antennaArray.Count)
                {
                    antennaArray.RemoveAt(antennaArray.Count - 1);
                }

                // serialize
                for (int i = 0; i < antennaArray.Count; i++)
                {
                    antennaArray[i] = EditorGUILayout.ObjectField("Antenna " + i, antennaArray[i], typeof(GameObject), true) as GameObject;

                }
                
                // Add information about radiation pattern
                EditorGUILayout.Space();

                
                if (GUILayout.Button("If radiation pattern is used"))
                {
                    antennaConfiguration.radiation_pattern_button_counter++;
                    //Debug.Log(antennaConfiguration.radiation_pattern_button_counter);
                }
                
                

                if (antennaConfiguration.radiation_pattern_button_counter % 2 == 1)
                {
                    antennaConfiguration.radiationPatternActive = true;
                    // define list of csv files
                    List<TextAsset> eadfArray = antennaConfiguration.radiation_patterns;
                    // correct size
                    while (AntennaNumber > eadfArray.Count)
                    {
                        eadfArray.Add(null);
                    }
                    while (AntennaNumber < eadfArray.Count)
                    {
                        eadfArray.RemoveAt(eadfArray.Count - 1);
                    }
                    // serialize
                    for (int i = 0; i < antennaArray.Count; i++)
                    {
                        eadfArray[i] = EditorGUILayout.ObjectField("EADF " + i, eadfArray[i], typeof(TextAsset), true) as TextAsset;
                    }
                }
                else
                {
                    antennaConfiguration.radiationPatternActive = false;
                    List<TextAsset> eadfArray = antennaConfiguration.radiation_patterns;
                    //Debug.Log("Size eadfArray = " + eadfArray.Count);

                    for (int i = 0; i < eadfArray.Count; i++)
                    {
                        eadfArray.RemoveAt(i);
                    }
                    //Debug.Log("Size eadfArray = " + eadfArray.Count);
                }

                //Debug.Log(antennaConfiguration.radiationPattern);

                EditorGUI.indentLevel--;
            }
        }

    }

#endif
#endregion
}
