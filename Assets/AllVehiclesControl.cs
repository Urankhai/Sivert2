using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;
using Unity.Collections;


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

    int channel_link_num;
    
    private void OnEnable()
    {

        CarCoordinates = new NativeArray<Vector3>(carsArray.Length, Allocator.Persistent);
    
        
        CarForwardVect = new NativeArray<Vector3>(carsArray.Length, Allocator.Persistent);
        CarsAntennaNumbers = new NativeArray<int>(carsArray.Length, Allocator.Persistent);

        int total_antenna_number = 0;
        for (int i = 0; i < carsArray.Length; i++)
        {
            // get information about the number of antennas
            int antenna_number = carsArray[i].GetComponent<AntennaConfiguration>().antennas.Count;
            bool eadf_activation = carsArray[i].GetComponent<AntennaConfiguration>().radiationPatternActive;
            Debug.Log(carsArray[i].name + " has " + antenna_number + " antennas; EADF is activated = " + eadf_activation);

            CarsAntennaNumbers[i] = antenna_number;

            total_antenna_number += antenna_number;
        }
        Debug.Log("Total antenna number = " + total_antenna_number);

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
                        channel_link_count += 1;
                    }
                }
                temp_ant2_counter += car2_ant_num;
            }
            temp_ant1_counter += car1_ant_num;
        }
        /*
        for (int car1ID = 0; car1ID < carsArray.Length; car1ID++)
        {
            int car1_ant_num = CarsAntennaNumbers[car1ID];
            for (int car1antID = 0; car1antID < car1_ant_num; car1antID++)
            {
                int temp_ant1ID_in_array= temp_ant1ID + car1antID;
            }
            temp_ant1ID += car1_ant_num;
        }
        */
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
