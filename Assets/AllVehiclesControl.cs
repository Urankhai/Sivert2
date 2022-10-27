using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;
using Unity.Collections;
using UnityEngine.UI;


public class AllVehiclesControl : MonoBehaviour
{
    public NavMeshAgent[] carsArray;
    public Transform[] destinations;

    
    [HideInInspector] public NativeArray<Vector3> CarCoordinates;
    [HideInInspector] public NativeArray<Vector3> CarForwardVect;
    
    [HideInInspector] public NativeArray<int> CarsAntennaNumbers;
    [HideInInspector] public NativeArray<Vector3> CarsAntennaPositions;
    [HideInInspector] public NativeArray<Vector2Int> AntennaMatrix;
    [HideInInspector] public NativeArray<ChannelLinks> Channel_Links;
    [HideInInspector] public NativeArray<ChannelLinksCoordinates> Channel_Links_Coordinates;
    [HideInInspector] public NativeArray<V4Int> EADF_Edges;
    [HideInInspector] public NativeArray<Vector2> EADF_Values;

    public TextAsset eadfFile;
    //Dictionary<string, string> eadf_real_imag;
    List<Vector2> eadf_real_imag;
    List<Vector2> all_eadf_files;
    List<Vector2Int> temp_eadf_edges;
    List<int> all_eadf_sizes;

    int channel_link_num;
    
    private void OnEnable()
    {

        CarCoordinates = new NativeArray<Vector3>(carsArray.Length, Allocator.Persistent);
    
        
        CarForwardVect = new NativeArray<Vector3>(carsArray.Length, Allocator.Persistent);
        CarsAntennaNumbers = new NativeArray<int>(carsArray.Length, Allocator.Persistent);

        // create a list consists of sizes of EADF files
        all_eadf_sizes = new List<int>();
        all_eadf_files = new List<Vector2>();

        temp_eadf_edges = new List<Vector2Int>();

        int total_antenna_number = 0;
        int eadf_array_size_all = 0;
        for (int i = 0; i < carsArray.Length; i++)
        {
            // get information about the number of antennas
            int antenna_number = carsArray[i].GetComponent<AntennaConfiguration>().antennas.Count;
            bool eadf_activation = carsArray[i].GetComponent<AntennaConfiguration>().radiationPatternActive;
            Debug.Log(carsArray[i].name + " has " + antenna_number + " antennas; EADF is activated = " + eadf_activation);

            if (eadf_activation == true)
            {
                for (int ant_i = 0; ant_i < antenna_number; ant_i++)
                {
                    int temp_edge_lft = all_eadf_files.Count;
                    eadfFile = carsArray[i].GetComponent<AntennaConfiguration>().radiation_patterns[ant_i];
                    
                    eadf_real_imag = new List<Vector2>();// define a new list for eadf matrix
                    ReadEADFFile();
                    eadf_array_size_all += eadf_real_imag.Count;
                    all_eadf_sizes.Add(eadf_real_imag.Count);
                    all_eadf_files.AddRange(eadf_real_imag);

                    int temp_edge_rht = all_eadf_files.Count - 1;
                    temp_eadf_edges.Add(new Vector2Int(temp_edge_lft, temp_edge_rht));

                    Debug.Log("EADF file has been read for Ant " + ant_i + "; EADF length = " + eadf_real_imag.Count + "; check list length " + all_eadf_files.Count);
                    Debug.Log("Car" + i + " ant" + ant_i + ": eadf range [" + temp_edge_lft + ", " + temp_edge_rht + "]");
                    Debug.Log("EADF values [" + all_eadf_files[temp_edge_lft].x + " + " + all_eadf_files[temp_edge_lft].y + "i, " + all_eadf_files[temp_edge_rht].x + " + " + all_eadf_files[temp_edge_rht].y + "i]");
                }
            }

            CarsAntennaNumbers[i] = antenna_number;

            total_antenna_number += antenna_number;
        }
        Debug.Log("Total antenna number = " + total_antenna_number + "; all EADF size = " + eadf_array_size_all);

        CarsAntennaPositions = new NativeArray<Vector3>(total_antenna_number, Allocator.Persistent);
        AntennaMatrix = new NativeArray<Vector2Int>(total_antenna_number, Allocator.Persistent);

        int antenna_count_step = 0;
        for (int i = 0; i < carsArray.Length; i++)
        {
            // get information about the number of antennas
            int antenna_number = carsArray[i].GetComponent<AntennaConfiguration>().antennas.Count;
            for (int j = 0; j < antenna_number; j++)
            {
                AntennaMatrix[antenna_count_step] = new Vector2Int(i, j);
                antenna_count_step += 1;
            }
        }


        channel_link_num = 0;
        for (int i = 0; i < CarsAntennaNumbers.Length; i++)
        {
            int temp_sum = 0;
            for (int j = i + 1; j < CarsAntennaNumbers.Length; j++)
            {
                temp_sum += CarsAntennaNumbers[j];
            }
            channel_link_num += CarsAntennaNumbers[i] * temp_sum;
        }

        Channel_Links = new NativeArray<ChannelLinks>(channel_link_num, Allocator.Persistent);
        Channel_Links_Coordinates = new NativeArray<ChannelLinksCoordinates>(channel_link_num, Allocator.Persistent);

        EADF_Edges = new NativeArray<V4Int>(channel_link_num, Allocator.Persistent);
        EADF_Values = new NativeArray<Vector2>(eadf_array_size_all, Allocator.Persistent);

        int channel_link_count = 0;
        int temp_ant1_counter = 0;
        int temp_ant2_counter = 0;
        for (int car1 = 0; car1 < CarsAntennaNumbers.Length; car1++)
        {
            int car1_ant_num = CarsAntennaNumbers[car1];
            temp_ant2_counter += car1_ant_num;
            for (int car2 = car1 + 1; car2 < CarsAntennaNumbers.Length; car2++)
            {
                int car2_ant_num = CarsAntennaNumbers[car2];
                for (int ant1 = 0; ant1 < car1_ant_num; ant1++)
                {
                    int temp_ant1ID = temp_ant1_counter + ant1;
                    for (int ant2 = 0; ant2 < car2_ant_num; ant2++)
                    {
                        int temp_ant2ID = temp_ant2_counter + ant2;
                        Channel_Links[channel_link_count] = new ChannelLinks(car1, car2, ant1, temp_ant1ID, ant2, temp_ant2ID);
                        // keeping starting and ending points of EADF files inside a big nativearray
                        V4Int tempV4int = new V4Int(temp_eadf_edges[temp_ant1ID].x, temp_eadf_edges[temp_ant1ID].y, temp_eadf_edges[temp_ant2ID].x, temp_eadf_edges[temp_ant2ID].y);
                        EADF_Edges[channel_link_count] = tempV4int;


                        channel_link_count += 1;
                    }
                }
                temp_ant2_counter += car2_ant_num;
            }
            temp_ant1_counter += car1_ant_num;
        }

        
        // rewriting list<Vector2> into nativarray<Vector2>
        for (int eadf_i = 0; eadf_i < all_eadf_files.Count; eadf_i++)
        {
            EADF_Values[eadf_i] = all_eadf_files[eadf_i];
        }

        Debug.Log("Number of links = " + channel_link_count + "; size of the total EADF nativearray = " + all_eadf_files.Count);
    }

    void ReadEADFFile()
    {
        var splitFile = new string[] { "r\n", "\r", "\n" };
        var splitLine = new char[] { ',' };
        var Lines = eadfFile.text.Split(splitFile, System.StringSplitOptions.RemoveEmptyEntries);
        for (int i = 0; i < Lines.Length; i++)
        {
            //print("ID: " + i + "; line = " + Lines[i]);
            var line = Lines[i].Split(splitLine, System.StringSplitOptions.None);
            
            double eadf_real = double.Parse(line[0], System.Globalization.NumberStyles.Float);
            double eadf_imag = double.Parse(line[1], System.Globalization.NumberStyles.Float);

            Vector2 temp_line = new Vector2((float)eadf_real, (float)eadf_imag);

            eadf_real_imag.Add(temp_line);
        }
        
    }
    private void OnDestroy()
    {
        CarCoordinates.Dispose();
        
        CarForwardVect.Dispose();

        CarsAntennaNumbers.Dispose();
        CarsAntennaPositions.Dispose();
        AntennaMatrix.Dispose();


        Channel_Links.Dispose();
        Channel_Links_Coordinates.Dispose();

        EADF_Edges.Dispose();
        EADF_Values.Dispose();
    }
    
    

    void Start()
    {
        // defining cars and their positions
        for (int i = 0; i < carsArray.Length; i++)
        { 
            // turn on AI path finding
            carsArray[i].SetDestination(destinations[i].position);
            
            

            
        }
    }

    //private void FixedUpdate()
    private void FixedUpdate()
    {
        int counted_antenna = 0;
        for (int i = 0; i < carsArray.Length; i++)
        {
            // Current positions of all vehicles
            CarCoordinates[i] = carsArray[i].transform.Find("Antenna").position;
            CarForwardVect[i] = carsArray[i].transform.Find("Antenna").forward;

            // Updating previous positions of vehicles

            ////////////////////////////////////////
            // Important! All created antennas should have their unique name:
            // the first antenna goes Antenna
            // the second antenna has Antenna2
            // the third antenna has Antenna3
            // the forth antenna has Antenna4
            // the N-th antenna has name AntennaN
            // Important! Only the first antenna name does not have number!
            ////////////////////////////////////////
            for (int j = 0; j < CarsAntennaNumbers[i]; j++)
            {
                /*
                // this piece of code can be used to find coordinates of the antennas using the AntennaConfiguration script

                Vector3 temp_position = carsArray[i].GetComponent<AntennaConfiguration>().antennas[j].transform.position;
                CarsAntennaPositions[counted_antenna + j] = temp_position;
                Debug.Log("Script: coordinates of antenna[" + i + ", " + j + "] = " + temp_position);
                */

                // This piece of code can be used to search for the coordinates of the antennas via names
                string antenna_name = "Antenna";
                if (j > 0)
                { antenna_name += (j + 1); }
                // Check correct names of the antennas
                // int carID = i;
                // int antID = j;
                Vector3 antPos = carsArray[i].transform.Find(antenna_name).position;
                CarsAntennaPositions[counted_antenna + j] = antPos;
                //Debug.Log("Car" + i + ": coordinates of " + antenna_name + " = " + temp_coor);
                
            }
            counted_antenna = CarsAntennaNumbers[i];
        }


        for (int i = 0; i < Channel_Links.Length; i++)
        {

            //float t1 = Time.realtimeSinceStartup;
            int temp_ant1ID = Channel_Links[i].V1AntennaID;
            int temp_ant2ID = Channel_Links[i].V2AntennaID;
            
            Vector3 ant1_position = CarsAntennaPositions[temp_ant1ID]; // position of an antenna inside of the array of CarsAntennaPositions
            Vector3 ant2_position = CarsAntennaPositions[temp_ant2ID]; // position of an antenna inside of the array of CarsAntennaPositions
            //Debug.Log("t1: " + ((Time.realtimeSinceStartup - t1) * 1000000f) + " mics");

            Channel_Links_Coordinates[i] = new ChannelLinksCoordinates(ant1_position, ant2_position);

            /*
            float t2 = Time.realtimeSinceStartup;
            int car1 = Channel_Links[i].Car1;
            int ant1 = Channel_Links[i].V1Antenna;
            
            int car2 = Channel_Links[i].Car2;
            int ant2 = Channel_Links[i].V2Antenna;
            
            Vector3 old_ant1_position = carsArray[car1].GetComponent<AntennaConfiguration>().antennas[ant1].transform.position;
            Vector3 old_ant2_position = carsArray[car2].GetComponent<AntennaConfiguration>().antennas[ant2].transform.position;
            Debug.Log("t2: " + ((Time.realtimeSinceStartup - t2) * 1000000f) + " mics");

            Vector3 diff1 = old_ant1_position - ant1_position;
            Vector3 diff2 = old_ant2_position - ant2_position;
            Debug.Log("diff1 " + diff1 + "; diff2 " + diff2);
            */

            //Debug.Log("Cars control: car[" + car1 + "], ant[" + ant1 + "] position " + ant1_position + "; car[" + car2 + "], ant[" + ant2 + "] position " + ant2_position);
        }

    }


    
}
