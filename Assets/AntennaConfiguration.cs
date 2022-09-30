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

    public bool radiationPatternActive = false;

    


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

            //antennaConfiguration.showAntennaList = EditorGUILayout.Foldout(antennaConfiguration.showAntennaList, "Antenna array information", false, myFoldoutStyle);
            EditorGUILayout.LabelField("Antenna array information", EditorStyles.boldLabel);
            //Debug.Log(antennaConfiguration.showAntennaList);

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
                int display_number = i + 1;
                antennaArray[i] = EditorGUILayout.ObjectField("Antenna " + display_number, antennaArray[i], typeof(GameObject), true) as GameObject;

            }

            EditorGUILayout.Space();

            //Debug.Log(antennaConfiguration.radiation_patterns);

            EditorGUI.indentLevel--;


            //antennaConfiguration.showRadiationList = EditorGUILayout.Foldout(antennaConfiguration.showRadiationList, "Radiation patterns files", false, myFoldoutStyle);

            EditorGUILayout.LabelField("Individual EADF files for each antenna", EditorStyles.boldLabel);
            if (antennaConfiguration.radiationPatternActive)
            {
                EditorGUI.indentLevel++;
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
                    int display_number = i + 1;
                    eadfArray[i] = EditorGUILayout.ObjectField("EADF " + display_number, eadfArray[i], typeof(TextAsset), true) as TextAsset;

                }
                EditorGUI.indentLevel--;
            }


        }

    }

#endif
    #endregion
}
