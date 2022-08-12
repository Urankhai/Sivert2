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


    }
    private void OnDestroy()
    {
        CarCoordinates.Dispose();
        
        CarForwardVect.Dispose();

        CarsAntennaNumbers.Dispose();
        CarsAntennaPositions.Dispose();
        AntennaMatrix.Dispose();
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
                if (carsArray[i].transform.Find(antenna_name) == null)
                {
                }
                else
                {
                    Vector3 temp_coor = carsArray[i].transform.Find(antenna_name).position;
                    CarsAntennaPositions[counted_antenna + j] = temp_coor;
                    //Debug.Log("Car" + i + ": coordinates of " + antenna_name + " = " + temp_coor);
                }
            }
            counted_antenna = CarsAntennaNumbers[i];
        }

    }


    
}
