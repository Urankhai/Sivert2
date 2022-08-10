using UnityEngine;
using System.Collections.Generic;

#if UNITY_EDITOR
using UnityEditor;
#endif

public class AntennaConfiguration : MonoBehaviour
{

    public List<GameObject> antennas = new List<GameObject>();
    public List<TextAsset> radiation_patterns = new List<TextAsset>();

    bool showAntennaList = true;






 #region Editor
#if UNITY_EDITOR
    [CustomEditor(typeof(AntennaConfiguration))]
    public class AntennaConfigurationEditor : Editor
    {
        public override void OnInspectorGUI()
        {
            //base.OnInspectorGUI();

            AntennaConfiguration antennaConfiguration = (AntennaConfiguration)target;

            DrawLists(antennaConfiguration);

        }



        static void DrawLists(AntennaConfiguration antennaConfiguration)
        {
            EditorGUILayout.Space();
            antennaConfiguration.showAntennaList = EditorGUILayout.Foldout(antennaConfiguration.showAntennaList, "Antenna array information", false);

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
                EditorGUI.indentLevel--;
            }
        }

    }

#endif
#endregion
}
