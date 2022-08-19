using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Collections;
using Unity.Jobs;
using Unity.Burst;
using System.IO;
using System;
using UnityEngine.UI;

public partial class ChannelGenManager : MonoBehaviour
{
    int FrameCounter;
    int NeigbouringCount;
    
    [Space]
    [Header("PHYSICAL PARAMETERS")]
    [Space]
    public float PowerInMilliWatts;
    public float NoiseLevelInDB;
    public float SpeedofLight = 299792458.0f; // m/s
    public float CarrierFrequency = (float)(5.9 * Mathf.Pow(10, 9)); // GHz
    public float fsubcarriers = (float)200000; // kHz
    //public int NumberOfSubcarriers = 64; // 1024 for LTE
    private float SNR_avg = 0;
    private float MRC_SNR_avg = 0;
    public Text SNR_Text;
    public bool OmniAntenna = false;


    //float EdgeEffectCoeff = -100.0f;

    [Space]
    [Header("RESULTS FILE")]
    [Space]

    public string filename;

    /// <summary>
    ///  Data for Fourier transform
    /// </summary>

    double[] Y_output;
    double[] H_output;
    //double[] Y_noise_output;
    //double[] H_noise_output;
    double[] X_inputValues;
    [Space]
    [Header("DETAILS FOR DRAWING")]
    [Space]

    public Transform tfTime;
    public Transform tfFreq;

    // for saving data
    public List<List<string>> H_save = new List<List<string>>(); // for saving data into a csv file
    public List<List<string>> h_save = new List<List<string>>(); // for saving data into a csv file

    // control parameters
    public bool DrawingOverlaps = false;
    public bool DrawingPath1 = false;
    public bool DrawingPath2 = false;
    public bool DrawingPath3 = false;
    public int ChannelLinksNumber;
    public int ChannelLinkDraw = 4;


    // MPCs Data
    //NativeArray<V6> DMC_Native;
    //NativeArray<Vector3> DMC_perp;
    //NativeArray<float> DMC_attenuation;
    //NativeArray<V6> MPC1_Native;
    //NativeArray<Vector3> MPC1_perp;
    //NativeArray<float> MPC1_attenuation;
    //NativeArray<V6> MPC2_Native;
    //NativeArray<Vector3> MPC2_perp;
    //NativeArray<float> MPC2_attenuation;
    //NativeArray<V6> MPC3_Native;
    //NativeArray<Vector3> MPC3_perp;
    //NativeArray<float> MPC3_attenuation;
    int DMC_num;
    int MPC1_num;
    int MPC2_num;
    int MPC3_num;

    NativeArray<Vector2> Pattern;

    int MPC_num;
    NativeArray<V6> MPC_Native;
    NativeArray<Vector3> MPC_perp;
    NativeArray<float> MPC_attenuation;

    // LookUp Table Data
    float maxdistance;
    //int maxNumberSeenMPC2;
    //int maxNumberSeenMPC3;
    NativeArray<SeenPath2> LookUpTableMPC2; // this array shows all properties of the path2s
    NativeArray<Vector2Int> MPC2LUTID; // this array shows how many paths come from each MPC2
    NativeArray<SeenPath3> LookUpTableMPC3;
    NativeArray<Vector2Int> MPC3SeenID;

    // Coordinates of single antennas
    NativeArray<Vector3> CarCoordinates;
    NativeArray<Vector3> CarForwardVect;

    // Coordinates of multiple antennas
    NativeArray<int> CarsAntennaNumbers;
    NativeArray<Vector3> CarsAntennaPositions;
    NativeArray<Vector2Int> AntennaMatrix;

    NativeArray<ChannelLinks> Channel_Links;
    NativeArray<ChannelLinksCoordinates> Channel_Links_Coordinates;

    // Creating nativearrays in this script that should be destroyed
    NativeArray<Vector2Int> Links;
    
    
    
    NativeArray<Overlap> Overlaps1;
    NativeArray<Overlap> TxOverlaps2;
    NativeArray<Overlap> RxOverlaps2;
    NativeArray<Overlap> TxOverlaps3;
    NativeArray<Overlap> RxOverlaps3;
    int link_num;
    int car_num;
    

    //LoS
    NativeArray<RaycastCommand> commandsLoS;
    NativeArray<RaycastHit> resultsLoS;
    NativeArray<Vector3> CornersNormalsPerpendiculars;

    // Multi Antenna (MA)
    // MA LoS
    NativeArray<RaycastCommand> MA_commandsLoS;
    NativeArray<RaycastHit> MA_resultsLoS;

    NativeArray<float> SoA;
    NativeArray<int> Seen; // either 0 or 1
    NativeArray<RaycastCommand> commands; // for DMCs
    NativeArray<RaycastHit> results; // for DMCs

    // Multi antenna case
    NativeArray<float> MA_SoA;
    NativeArray<int> MA_Seen; // either 0 or 1
    NativeArray<RaycastCommand> MA_commands; // for DMCs
    NativeArray<RaycastHit> MA_results; // for DMCs

    static readonly int FFTNum = 64;
    public System.Numerics.Complex[] H = new System.Numerics.Complex[FFTNum]; // Half of LTE BandWidth, instead of 2048 subcarriers

    

    NativeArray<float> Subcarriers;
    NativeArray<float> InverseWavelengths;
    NativeArray<System.Numerics.Complex> H_LoS;
    NativeArray<System.Numerics.Complex> H_NLoS;

    NativeArray<System.Numerics.Complex> MA_H_LoS;
    NativeArray<System.Numerics.Complex> MA_H_NLoS;
    NativeArray<System.Numerics.Complex> MA_H_ToT;
    NativeArray<System.Numerics.Complex> MA_H_MRC;
    [Space]
    [Header("MISC for TESTING")]
    [Space]
    public float SubframePower;
    private void OnEnable()
    {
        SubframePower = PowerInMilliWatts / FFTNum;
        //Debug.Log("Power per subframe = " + SubframePower);

        Subcarriers = new NativeArray<float>(FFTNum, Allocator.Persistent);
        InverseWavelengths = new NativeArray<float>(FFTNum, Allocator.Persistent);
        
        for (int i = 0; i < FFTNum; i++)
        {
            Subcarriers[i] = CarrierFrequency + fsubcarriers * i;
            InverseWavelengths[i] = Subcarriers[i] / SpeedofLight;
        }

        Debug.Log("File name" + filename);

        List<float> listA = new List<float>();
        List<float> listB = new List<float>();

        using (var reader = new StreamReader(@"C:\Users\Administrator\Desktop\Aleksei\Parallel3DChaSi\GSCM1_InTimeDomain\Assets\EADF\HelixV2VEADF.csv"))
        {
            while (!reader.EndOfStream)
            {
                var line = reader.ReadLine();
                var values = line.Split(',');
                //(float)Convert.ToDouble("41.00027357629127");
                listA.Add((float)Convert.ToDouble(values[0]));
                listB.Add((float)Convert.ToDouble(values[1]));
            }
            //Debug.Log(listA.Count);
        }

        Pattern = new NativeArray<Vector2>(listA.Count, Allocator.Persistent); //new Vector2[17];
        
        for (int i=0; i < listA.Count; i++)
        {
            Pattern[i] = new Vector2(listA[i], listB[i]);
        }

    }

    private void OnDestroy()
    {
        Pattern.Dispose();
        Links.Dispose();
        
        
        Overlaps1.Dispose();
        TxOverlaps2.Dispose();
        RxOverlaps2.Dispose();
        TxOverlaps3.Dispose();
        RxOverlaps3.Dispose();

        commandsLoS.Dispose();
        resultsLoS.Dispose();

        MA_commandsLoS.Dispose();
        MA_resultsLoS.Dispose();
        
        // Single antenna case
        SoA.Dispose();
        Seen.Dispose();
        commands.Dispose();
        results.Dispose();

        // Multiple antenna case
        MA_SoA.Dispose();
        MA_Seen.Dispose();
        MA_commands.Dispose();
        MA_results.Dispose();

        Subcarriers.Dispose();
        InverseWavelengths.Dispose();
        H_LoS.Dispose();
        H_NLoS.Dispose();

        MA_H_LoS.Dispose();
        MA_H_NLoS.Dispose();
        MA_H_ToT.Dispose();
        MA_H_MRC.Dispose();
    }


    void Start()
    {
        FrameCounter = 0;
        NeigbouringCount = 0;
        /// for Fourier transform
        X_inputValues = new double[H.Length];
        for (int i = 0; i < H.Length; i++)
        { X_inputValues[i] = i; }

        #region Reading Info from Scripts
        #region MPCs
        // MPCs Data
        GameObject MPC_Spawner = GameObject.Find("CorrectPolygons");
        Correcting_polygons_Native MPC_Native_Script = MPC_Spawner.GetComponent<Correcting_polygons_Native>();

        DMC_num  = MPC_Native_Script.ActiveV6_DMC_NativeList.Length;
        MPC1_num = MPC_Native_Script.ActiveV6_MPC1_NativeList.Length;
        MPC2_num = MPC_Native_Script.ActiveV6_MPC2_NativeList.Length;
        MPC3_num = MPC_Native_Script.ActiveV6_MPC3_NativeList.Length;


        CornersNormalsPerpendiculars = MPC_Native_Script.Active_CornersNormalsPerpendiculars;
        
        
        // for all MPCs
        MPC_num = MPC_Native_Script.ActiveV6_MPC_NativeList.Length;
        MPC_Native = MPC_Native_Script.ActiveV6_MPC_NativeList;
        MPC_attenuation = MPC_Native_Script.ActiveV6_MPC_Power;
        MPC_perp = MPC_Native_Script.Active_MPC_Perpendiculars;

        #endregion

        #region LookUpTable
        // LookUp Table Data
        GameObject LookUpTable = GameObject.Find("LookUpTablesUpd");
        LookUpTableGenUpd LUT_Script = LookUpTable.GetComponent<LookUpTableGenUpd>();
        maxdistance = LUT_Script.MaxSeenDistance;
        

        LookUpTableMPC2 = LUT_Script.LookUpTableMPC2;
        MPC2LUTID = LUT_Script.MPC2LUTID;
        LookUpTableMPC3 = LUT_Script.LookUpTableMPC3;
        MPC3SeenID = LUT_Script.MPC3SeenID;
        #endregion

        #region Positions and calculate number of channel links

        // Coordinates of antennas
        GameObject VehiclesData = GameObject.Find("MovementManager");
        AllVehiclesControl ControlScript= VehiclesData.GetComponent<AllVehiclesControl>();
        CarCoordinates = ControlScript.CarCoordinates;
        CarForwardVect = ControlScript.CarForwardVect;
        

        // Coordinates of multiple antennas
        CarsAntennaNumbers = ControlScript.CarsAntennaNumbers;
        CarsAntennaPositions = ControlScript.CarsAntennaPositions;
        AntennaMatrix = ControlScript.AntennaMatrix;
        Channel_Links = ControlScript.Channel_Links;
        Channel_Links_Coordinates = ControlScript.Channel_Links_Coordinates;
        ChannelLinksNumber = Channel_Links.Length;


        // MA LoS
        MA_commandsLoS = new NativeArray<RaycastCommand>(Channel_Links.Length, Allocator.Persistent);
        MA_resultsLoS = new NativeArray<RaycastHit>(Channel_Links.Length, Allocator.Persistent);

        #endregion

        #endregion

        car_num = CarCoordinates.Length;

        link_num = car_num * (car_num - 1) / 2;
        
        Links = new NativeArray<Vector2Int>(link_num, Allocator.Persistent);
        Overlaps1 = new NativeArray<Overlap>(link_num, Allocator.Persistent);
        TxOverlaps2 = new NativeArray<Overlap>(link_num, Allocator.Persistent);
        RxOverlaps2 = new NativeArray<Overlap>(link_num, Allocator.Persistent);
        TxOverlaps3 = new NativeArray<Overlap>(link_num, Allocator.Persistent);
        RxOverlaps3 = new NativeArray<Overlap>(link_num, Allocator.Persistent);

        // LoS
        commandsLoS = new NativeArray<RaycastCommand>(2*link_num, Allocator.Persistent);
        resultsLoS = new NativeArray<RaycastHit>(2*link_num, Allocator.Persistent);

        
        // MPCs
        SoA = new NativeArray<float>( (DMC_num + MPC1_num + MPC2_num + MPC3_num) * car_num, Allocator.Persistent);
        Seen = new NativeArray<int>((DMC_num + MPC1_num + MPC2_num + MPC3_num) * car_num, Allocator.Persistent);
        commands = new NativeArray<RaycastCommand>((DMC_num + MPC1_num + MPC2_num + MPC3_num) * car_num, Allocator.Persistent);
        results = new NativeArray<RaycastHit>((DMC_num + MPC1_num + MPC2_num + MPC3_num) * car_num, Allocator.Persistent);

        // MPCs processing for the MA case
        
        MA_SoA = new NativeArray<float>((DMC_num + MPC1_num + MPC2_num + MPC3_num) * CarsAntennaPositions.Length, Allocator.Persistent);
        MA_Seen = new NativeArray<int>((DMC_num + MPC1_num + MPC2_num + MPC3_num) * CarsAntennaPositions.Length, Allocator.Persistent);
        MA_commands = new NativeArray<RaycastCommand>((DMC_num + MPC1_num + MPC2_num + MPC3_num) * CarsAntennaPositions.Length, Allocator.Persistent);
        MA_results = new NativeArray<RaycastHit>((DMC_num + MPC1_num + MPC2_num + MPC3_num) * CarsAntennaPositions.Length, Allocator.Persistent);

        int link_count = 0;
        for (int i = 0; i < car_num; i++)
        {
            for (int j = i + 1; j < car_num; j++)
            {
                Links[link_count] = new Vector2Int(i, j);
                link_count += 1;
            }
        }

        // Channels for all links
        H_LoS = new NativeArray<System.Numerics.Complex>(FFTNum * link_num, Allocator.Persistent);
        H_NLoS = new NativeArray<System.Numerics.Complex>(FFTNum * link_num, Allocator.Persistent);

        MA_H_LoS = new NativeArray<System.Numerics.Complex>(FFTNum * Channel_Links.Length, Allocator.Persistent);
        MA_H_NLoS = new NativeArray<System.Numerics.Complex>(FFTNum * Channel_Links.Length, Allocator.Persistent);

        MA_H_ToT = new NativeArray<System.Numerics.Complex>(FFTNum * Channel_Links.Length, Allocator.Persistent);
        // The size of MRC should lbe equal 
        MA_H_MRC = new NativeArray<System.Numerics.Complex>(FFTNum * CarsAntennaPositions.Length, Allocator.Persistent);
        /*
         * H1 00 10 -> 00: 10 + 11 + 12 (H00 = H1); 10 -> 00 + 01
         * H2 00 11
         * H3 00 12
         * H4 01 10
         * H5 01 11
         * H6 01 12
         */

        //EdgeEffectCoeff = 10.0f;
    }

    private void FixedUpdate()
    //void Update()
    {
        FrameCounter++;
        #region Writing data into csv file
        /*
        if (Mathf.Abs(-18.0f - CarCoordinates[1].z) < 1.0f)
        {
            Debug.Log("Frame number = " + FrameCounter);
            NeigbouringCount++;
            if (NeigbouringCount == 1)
            {
                Debug.Log("Frame number = " + FrameCounter);
                
                string path = @"C:\Users\Administrator\Desktop\Aleksei\Parallel3DChaSi\GSCM1_InTimeDomain\Assets\";
                string path2 = path + filename;// Application.persistentDataPath + "/H_freq1.csv";

                using (var file = File.CreateText(path2))
                {
                    foreach (var arr in H_save)
                    {
                        //if (String.IsNullOrEmpty(arr)) continue;
                        file.Write(arr[0]);
                        for (int i = 1; i < arr.Count; i++)
                        {
                            file.Write(',');
                            file.Write(arr[i]);
                        }
                        file.WriteLine();
                    }
                }

                Debug.Log("The lenght of the list<list> structure " + H_save.Count);
                //Application.Quit();

                EditorApplication.isPlaying = false;
            }
        }
        */
        #endregion

        #region Defining edges of seen areas for all links (The duration is about 20 micro seconds)
        AreaOverlaps areaOverlaps = new AreaOverlaps
        {
            MaxDist = maxdistance,
            Coordinates = CarCoordinates,
            Links = Links,

            AreaArray1 = Overlaps1,
            TxAreaArray2 = TxOverlaps2,
            RxAreaArray2 = RxOverlaps2,
            TxAreaArray3 = TxOverlaps3,
            RxAreaArray3 = RxOverlaps3,
        };
        JobHandle findoverlaps = areaOverlaps.Schedule(link_num, 1);
        findoverlaps.Complete();
        
        if (DrawingOverlaps)
        {
            DrawOverlaps(Overlaps1);
            DrawOverlaps(TxOverlaps2);
            DrawOverlaps(TxOverlaps3);
        }
        #endregion


        #region DrawOverlaps Function
        void DrawOverlaps(NativeArray<Overlap> Array)
        {
            for (int i = 0; i < Array.Length; i++)
            {
                if (Array[i].InfInf != new Vector2(0, 0))
                {
                    Vector3 II = new Vector3(Array[i].InfInf.x, 0, Array[i].InfInf.y);
                    Vector3 IS = new Vector3(Array[i].SupSup.x, 0, Array[i].InfInf.y);
                    Vector3 SI = new Vector3(Array[i].InfInf.x, 0, Array[i].SupSup.y);
                    Vector3 SS = new Vector3(Array[i].SupSup.x, 0, Array[i].SupSup.y);

                    if (i == 0)
                    {
                        Debug.DrawLine(II, IS, Color.cyan);
                        Debug.DrawLine(IS, SS, Color.cyan);
                        Debug.DrawLine(SS, SI, Color.cyan);
                        Debug.DrawLine(SI, II, Color.cyan);
                    }
                    else if (i == 1)
                    {
                        Debug.DrawLine(II + new Vector3(0, 1, 0), IS + new Vector3(0, 1, 0), Color.red);
                        Debug.DrawLine(IS + new Vector3(0, 1, 0), SS + new Vector3(0, 1, 0), Color.red);
                        Debug.DrawLine(SS + new Vector3(0, 1, 0), SI + new Vector3(0, 1, 0), Color.red);
                        Debug.DrawLine(SI + new Vector3(0, 1, 0), II + new Vector3(0, 1, 0), Color.red);
                    }
                    else
                    {
                        Debug.DrawLine(II + new Vector3(0, 2, 0), IS + new Vector3(0, 2, 0), Color.blue);
                        Debug.DrawLine(IS + new Vector3(0, 2, 0), SS + new Vector3(0, 2, 0), Color.blue);
                        Debug.DrawLine(SS + new Vector3(0, 2, 0), SI + new Vector3(0, 2, 0), Color.blue);
                        Debug.DrawLine(SI + new Vector3(0, 2, 0), II + new Vector3(0, 2, 0), Color.blue);
                    }
                }
            }
        }
        #endregion



        #region LoS Channel Calculation
        /*
        for (int i = 0; i < CarsAntennaPositions.Length; i++)
        {
            Vector3 antenna_coordinates = CarsAntennaPositions[i];
            int car_id = AntennaMatrix[i].x;
            int ant_id = AntennaMatrix[i].y;
            Debug.Log("Car[" + car_id + "]: coordinates of ant[" + ant_id + "] = " + antenna_coordinates);
        }

        for (int i = 0; i < Channel_Links_Coordinates.Length; i++)
        {
            int car1 = Channel_Links[i].Car1;
            int ant1 = Channel_Links[i].V1Antenna;

            int car2 = Channel_Links[i].Car2;
            int ant2 = Channel_Links[i].V2Antenna;

            Vector3 ant1_position = Channel_Links_Coordinates[i].Ant1Coordinates;
            Vector3 ant2_position = Channel_Links_Coordinates[i].Ant2Coordinates;

            Debug.Log("Cars control: car[" + car1 + "], ant[" + ant1 + "] position " + ant1_position + "; car[" + car2 + "], ant[" + ant2 + "] position " + ant2_position);
        }
        */

        MA_ParallelLoSDetection MA_LoSDetection = new MA_ParallelLoSDetection
        {
            AntennaPositions = Channel_Links_Coordinates,
            RayCastcommands = MA_commandsLoS,
        };
        JobHandle MA_LoSDetectionHandle = MA_LoSDetection.Schedule(Channel_Links_Coordinates.Length, 1);
        MA_LoSDetectionHandle.Complete();
        // parallel raycasting
        JobHandle MA_rayCastJobLoS = RaycastCommand.ScheduleBatch(MA_commandsLoS, MA_resultsLoS, 1, default);
        MA_rayCastJobLoS.Complete();

        MA_ParallelLoSChannel MA_LoSChannel = new MA_ParallelLoSChannel
        {
            OmniAntennaFlag = OmniAntenna,
            FFTSize = FFTNum,
            AntennasPositions = Channel_Links_Coordinates,
            CarsFwd = CarForwardVect,
            Channel_Links = Channel_Links,
            MA_raycastresults = MA_resultsLoS,
            inverseLambdas = InverseWavelengths,
            Pattern = Pattern,

            HLoS = MA_H_LoS,
        };
        JobHandle MA_LoSChannelHandle = MA_LoSChannel.Schedule(MA_H_LoS.Length, 64);
        MA_LoSChannelHandle.Complete();

        


        

        ParallelLoSDetection LoSDetection = new ParallelLoSDetection
        {
            CarsPositions = CarCoordinates,
            Links = Links,
            commands = commandsLoS,
        };
        JobHandle LoSDetectionHandle = LoSDetection.Schedule(2*link_num, 1);
        LoSDetectionHandle.Complete();
        // parallel raycasting
        JobHandle rayCastJobLoS = RaycastCommand.ScheduleBatch(commandsLoS, resultsLoS, 1, default);
        rayCastJobLoS.Complete();

        ParallelLoSChannel LoSChannel = new ParallelLoSChannel
        {
            OmniAntennaFlag = OmniAntenna,
            FFTSize = FFTNum,
            CarsPositions = CarCoordinates,
            CarsFwd = CarForwardVect,
            Links = Links,
            raycastresults = resultsLoS,
            inverseLambdas = InverseWavelengths,
            Pattern = Pattern,

            HLoS = H_LoS,
        };
        JobHandle LoSChannelHandle = LoSChannel.Schedule(H_LoS.Length, 64);
        LoSChannelHandle.Complete();


        /*
        int subcarier_position = 10;
        double SA_H = H_LoS[subcarier_position].Real + H_LoS[subcarier_position].Imaginary;
        double MA_H = MA_H_LoS[subcarier_position].Real + MA_H_LoS[subcarier_position].Imaginary;
        double diff_H = (H_LoS[subcarier_position].Real - MA_H_LoS[subcarier_position + 0*FFTNum].Real) + (H_LoS[subcarier_position].Imaginary - MA_H_LoS[subcarier_position + 0*FFTNum].Imaginary);
        Debug.Log("Single Antenna SA_H = " + SA_H + "; Multiple Antenna MA_H = " + MA_H + "; Channel difference = " + diff_H);
        */
        /*
        float ant_distance = resultsLoS[0].distance;
        if (ant_distance == 0)
        {
            Debug.Log("H[0] = " + H_LoS[0]);
            Debug.Log("Distance between antennas = " + ant_distance);
        }
        else
        {
            Debug.Log("H[0] = " + H_LoS[0]);
        }
        */
        #endregion


        #region MPC raycasting procedure (The duration is about: MA ~ 1.5 ms, SA ~ 0.7 ms)
        // Multiple antenna case

        //float t_MA_NLoS = Time.realtimeSinceStartup;

        ParallelRayCastingDataAntennas MA_RayCastingData = new ParallelRayCastingDataAntennas
        {
            CastingDistance = maxdistance,
            Antennas_Positions = CarsAntennaPositions,
            MPC_Array = MPC_Native,
            MPC_Perpendiculars = MPC_perp,

            MA_SoA = MA_SoA,
            MA_SeenIndicator = MA_Seen,
            MA_commands = MA_commands,
        };
        JobHandle MA_jobHandle_RayCastingData = MA_RayCastingData.Schedule(MPC_num * CarsAntennaPositions.Length, 32);
        //jobHandle_RayCastingData0.Complete();

        // parallel raycasting
        JobHandle MA_rayCastJob = RaycastCommand.ScheduleBatch(MA_commands, MA_results, 32, MA_jobHandle_RayCastingData);
        MA_rayCastJob.Complete();
        
        //Debug.Log("Time spent for MA NLoS Detection: " + ((Time.realtimeSinceStartup - t_MA_NLoS) * 1000f) + " ms");

        // Single antenna case

        //float t_SA_NLoS = Time.realtimeSinceStartup;

        ParallelRayCastingDataCars RayCastingData = new ParallelRayCastingDataCars
        {
            CastingDistance = maxdistance,
            Cars_Positions = CarCoordinates,
            MPC_Array = MPC_Native,
            MPC_Perpendiculars = MPC_perp,

            SoA = SoA,
            SeenIndicator = Seen,
            commands = commands,
        };
        JobHandle jobHandle_RayCastingData = RayCastingData.Schedule(MPC_num * car_num, 16);
        //jobHandle_RayCastingData0.Complete();

        // parallel raycasting
        JobHandle rayCastJob = RaycastCommand.ScheduleBatch(commands, results, 16, jobHandle_RayCastingData);
        rayCastJob.Complete();

        //Debug.Log("Time spent for SA NLoS Detection: " + ((Time.realtimeSinceStartup - t_SA_NLoS) * 1000f) + " ms");
        #endregion

        #region HashMap procedure
        //float tMA = Time.realtimeSinceStartup;
        // Multy Antenna case
        var MA_map = new NativeMultiHashMap<int, MA_Path_and_IDs>(MPC_num * Channel_Links.Length, Allocator.TempJob);
        var MA_idarray = new NativeArray<int>(MPC_num * Channel_Links.Length, Allocator.TempJob);
        MA_ChannelParametersAll MA_channelParameters = new MA_ChannelParametersAll
        {
            OmniAntennaFlag = OmniAntenna,
            MPC_Attenuation = MPC_attenuation,
            MPC_Array = MPC_Native,
            MPC_Perp = MPC_perp,
            DMCNum = DMC_num,
            MPC1Num = MPC1_num,
            MPC2Num = MPC2_num,
            MPC3Num = MPC3_num,
            MPCNum = MPC_num,

            LookUpTableMPC2 = LookUpTableMPC2,
            LUTIndexRangeMPC2 = MPC2LUTID,
            LookUpTableMPC3 = LookUpTableMPC3,
            LUTIndexRangeMPC3 = MPC3SeenID,

            MA_channellinks = Channel_Links,
            AntennasCoordinates = CarCoordinates,
            CarsForwardsDir = CarForwardVect,

            MA_Commands = MA_commands,
            MA_Results = MA_results,
            MA_SignOfArrival = MA_SoA,
            MA_SeenIndicator = MA_Seen,

            Pattern = Pattern,

            IDArray = MA_idarray,
            HashMap = MA_map.AsParallelWriter(),
        };
        JobHandle MA_channelParametersJob = MA_channelParameters.Schedule(MPC_num * Channel_Links.Length, 4);
        MA_channelParametersJob.Complete();
        //Debug.Log("tMA: " + ((Time.realtimeSinceStartup - tMA) * 1000f) + " ms");
        #region Drwaing possible paths
        if (DrawingPath1 || DrawingPath2 || DrawingPath3)
        {
            for (int i = (ChannelLinkDraw - 1) * MPC_num; i < ChannelLinkDraw * MPC_num; i++)
            {
                if (MA_map.TryGetFirstValue(i, out MA_Path_and_IDs path, out NativeMultiHashMapIterator<int> nativeMultiHashMapIterator))
                {
                    do
                    {
                        if (path.PathOrder == 1 && DrawingPath1)
                        {
                            Vector3 ant1_coor = CarsAntennaPositions[path.ChainIDs.ChLink.V1AntennaID];
                            Vector3 ant2_coor = CarsAntennaPositions[path.ChainIDs.ChLink.V2AntennaID];
                            Vector3 MPC_coor1 = MPC_Native[path.ChainIDs.ID1].Coordinates;
                            Debug.DrawLine(ant1_coor, MPC_coor1, Color.white);
                            Debug.DrawLine(MPC_coor1, ant2_coor, Color.grey);
                        }
                        else if (path.PathOrder == 2 && DrawingPath2)
                        {
                            Vector3 ant1_coor = CarsAntennaPositions[path.ChainIDs.ChLink.V1AntennaID];
                            Vector3 ant2_coor = CarsAntennaPositions[path.ChainIDs.ChLink.V2AntennaID];
                            Vector3 MPC_coor1 = MPC_Native[path.ChainIDs.ID1].Coordinates;
                            Vector3 MPC_coor2 = MPC_Native[path.ChainIDs.ID2].Coordinates;
                            Debug.DrawLine(ant1_coor, MPC_coor1, Color.yellow);
                            Debug.DrawLine(MPC_coor1, MPC_coor2, Color.yellow);
                            Debug.DrawLine(MPC_coor2, ant2_coor, Color.yellow);
                        }
                        else if (path.PathOrder == 3 && DrawingPath3)
                        {
                            Vector3 ant1_coor = CarsAntennaPositions[path.ChainIDs.ChLink.V1AntennaID];
                            Vector3 ant2_coor = CarsAntennaPositions[path.ChainIDs.ChLink.V2AntennaID];
                            Vector3 MPC_coor1 = MPC_Native[path.ChainIDs.ID1].Coordinates;
                            Vector3 MPC_coor2 = MPC_Native[path.ChainIDs.ID2].Coordinates;
                            Vector3 MPC_coor3 = MPC_Native[path.ChainIDs.ID3].Coordinates;
                            Debug.DrawLine(ant1_coor, MPC_coor1, Color.green);
                            Debug.DrawLine(MPC_coor1, MPC_coor2, Color.green);
                            Debug.DrawLine(MPC_coor2, MPC_coor3, Color.green);
                            Debug.DrawLine(MPC_coor3, ant2_coor, Color.green);
                            //Debug.Log(20 * Mathf.Log10(path3.AngularGain));
                        }
                    }
                    while (MA_map.TryGetNextValue(out path, ref nativeMultiHashMapIterator));
                }
            }
        }
        #endregion



        //float tSA = Time.realtimeSinceStartup;
        // Single Antenna case
        var map = new NativeMultiHashMap<int, Path_and_IDs>(MPC_num * link_num, Allocator.TempJob);
        var idarray = new NativeArray<int>(MPC_num * link_num, Allocator.TempJob);
        ChannelParametersAll channelParameters = new ChannelParametersAll
        {
            OmniAntennaFlag = OmniAntenna,
            MPC_Attenuation = MPC_attenuation,
            MPC_Array = MPC_Native,
            MPC_Perp = MPC_perp,
            DMCNum = DMC_num,
            MPC1Num = MPC1_num,
            MPC2Num = MPC2_num,
            MPC3Num = MPC3_num,
            MPCNum = MPC_num,

            LookUpTableMPC2 = LookUpTableMPC2,
            LUTIndexRangeMPC2 = MPC2LUTID,
            LookUpTableMPC3 = LookUpTableMPC3,
            LUTIndexRangeMPC3 = MPC3SeenID,

            channellinks = Links,
            CarsCoordinates = CarCoordinates,
            CarsForwardsDir = CarForwardVect,

            Commands = commands,
            Results = results,
            SignOfArrival = SoA,
            SeenIndicator = Seen,
            
            Pattern = Pattern,

            IDArray = idarray,
            HashMap = map.AsParallelWriter(),
        };
        JobHandle channelParametersJob = channelParameters.Schedule(MPC_num * link_num, 4);
        channelParametersJob.Complete();
        //Debug.Log("tSA: " + ((Time.realtimeSinceStartup - tSA) * 1000f) + " ms");
        #endregion

        
        #region Drwaing possible paths
        if (DrawingPath1 || DrawingPath2 || DrawingPath3)
        {
            for (int i = 0; i < MPC_num; i++)
            {
                if (map.TryGetFirstValue(i, out Path_and_IDs path, out NativeMultiHashMapIterator<int> nativeMultiHashMapIterator))
                {
                    do
                    {
                        if (path.PathOrder == 1 && DrawingPath1)
                        {
                            Vector3 car1_coor = CarCoordinates[path.ChainIDs.Car1];
                            Vector3 car2_coor = CarCoordinates[path.ChainIDs.Car2];
                            Vector3 MPC_coor1 = MPC_Native[path.ChainIDs.ID1].Coordinates;
                            Debug.DrawLine(car1_coor, MPC_coor1, Color.white);
                            Debug.DrawLine(MPC_coor1, car2_coor, Color.grey);
                        }
                        else if (path.PathOrder == 2 && DrawingPath2)
                        {
                            Vector3 car1_coor = CarCoordinates[path.ChainIDs.Car1];
                            Vector3 car2_coor = CarCoordinates[path.ChainIDs.Car2];
                            Vector3 MPC_coor1 = MPC_Native[path.ChainIDs.ID1].Coordinates;
                            Vector3 MPC_coor2 = MPC_Native[path.ChainIDs.ID2].Coordinates;
                            Debug.DrawLine(car1_coor, MPC_coor1, Color.white);
                            Debug.DrawLine(MPC_coor1, MPC_coor2, Color.white);
                            Debug.DrawLine(MPC_coor2, car2_coor, Color.white);
                        }
                        else if (path.PathOrder == 3 && DrawingPath3)
                        {
                            Vector3 car1_coor = CarCoordinates[path.ChainIDs.Car1];
                            Vector3 car2_coor = CarCoordinates[path.ChainIDs.Car2];
                            Vector3 MPC_coor1 = MPC_Native[path.ChainIDs.ID1].Coordinates;
                            Vector3 MPC_coor2 = MPC_Native[path.ChainIDs.ID2].Coordinates;
                            Vector3 MPC_coor3 = MPC_Native[path.ChainIDs.ID3].Coordinates;
                            Debug.DrawLine(car1_coor, MPC_coor1, Color.green);
                            Debug.DrawLine(MPC_coor1, MPC_coor2, Color.green);
                            Debug.DrawLine(MPC_coor2, MPC_coor3, Color.green);
                            Debug.DrawLine(MPC_coor3, car2_coor, Color.green);
                            //Debug.Log(20 * Mathf.Log10(path3.AngularGain));
                        }
                    }
                    while (map.TryGetNextValue(out path, ref nativeMultiHashMapIterator));
                }
            }
        }
        #endregion
        

        

        // MA case
        float t_MA_chan = Time.realtimeSinceStartup;
        NativeList<int> MA_nonzero_indexes = new NativeList<int>(Allocator.TempJob);
        IndexNonZeroFilter MA_nzindexes = new IndexNonZeroFilter
        {
            Array = MA_idarray,
        };
        JobHandle MA_jobHandleIndexNonZeroFilter = MA_nzindexes.ScheduleAppend(MA_nonzero_indexes, MA_idarray.Length, 64);
        MA_jobHandleIndexNonZeroFilter.Complete();

        NativeArray<Vector2Int> MA_link_ids = new NativeArray<Vector2Int>(Channel_Links.Length, Allocator.TempJob);
        LinkIndexes MA_linkIndexes = new LinkIndexes
        {
            MPCNum = MPC_num,
            NonZeroIndexes = MA_nonzero_indexes,

            LinkIDs = MA_link_ids,
        };
        JobHandle MA_jobHandlelinkIndexes = MA_linkIndexes.Schedule(Channel_Links.Length, 1, MA_jobHandleIndexNonZeroFilter);
        //MA_jobHandlelinkIndexes.Complete();
        
        MA_ParallelChannel MA_parallelChannel = new MA_ParallelChannel
        {
            FFTSize = FFTNum,
            InverseLambdas = InverseWavelengths,
            HashMap = MA_map,
            MPCNum = MPC_num,
            LinkNum = Channel_Links.Length,
            NonZeroIndexes = MA_nonzero_indexes,
            LinkIDs = MA_link_ids,

            H_NLoS = MA_H_NLoS,
        };
        JobHandle MA_parallelChannelJob = MA_parallelChannel.Schedule(FFTNum * Channel_Links.Length, 1, MA_jobHandlelinkIndexes);
        MA_parallelChannelJob.Complete();
        //Debug.Log("Time spent for MA channel calculation: " + ((Time.realtimeSinceStartup - t_MA_chan) * 1000f) + " ms");















        // SA case
        float t_SA_chan = Time.realtimeSinceStartup;
        NativeList<int> nonzero_indexes = new NativeList<int>(Allocator.TempJob);
        IndexNonZeroFilter nzindexes = new IndexNonZeroFilter
        {
            Array = idarray,
        };
        JobHandle jobHandleIndexNonZeroFilter = nzindexes.ScheduleAppend(nonzero_indexes, idarray.Length, 64);
        jobHandleIndexNonZeroFilter.Complete();

        NativeArray<Vector2Int> link_ids = new NativeArray<Vector2Int>(link_num, Allocator.TempJob);
        LinkIndexes linkIndexes = new LinkIndexes
        {
            MPCNum = MPC_num,
            NonZeroIndexes = nonzero_indexes,

            LinkIDs = link_ids,
        };
        JobHandle jobHandlelinkIndexes = linkIndexes.Schedule(link_num, 1, MA_jobHandleIndexNonZeroFilter);
        jobHandlelinkIndexes.Complete();
        

        ParallelChannel parallelChannel = new ParallelChannel
        {
            FFTSize = FFTNum,
            CarsPositions = CarCoordinates,
            CarsFwd = CarForwardVect,
            InverseLambdas = InverseWavelengths,
            HashMap = map,
            MPCNum = MPC_num,
            LinkNum = link_num,
            NonZeroIndexes = nonzero_indexes,
            LinkIDs = link_ids,

            H_NLoS = H_NLoS,
        };
        JobHandle parallelChannelJob = parallelChannel.Schedule(FFTNum * link_num, 1, jobHandlelinkIndexes);
        parallelChannelJob.Complete();
        //Debug.Log("Time spent for SA channel calculation: " + ((Time.realtimeSinceStartup - t_SA_chan) * 1000f) + " ms");
        //Debug.Log("Number of paths = " + nonzero_indexes.Length);

        //int test_subcarrier = 32;
        //double diff_H = H_NLoS[test_subcarrier].Real - MA_H_NLoS[test_subcarrier].Real + H_NLoS[test_subcarrier].Imaginary - MA_H_NLoS[test_subcarrier].Imaginary;
        //Debug.Log("Channel results difference = " + diff_H);



        #region FFT operation
        //float t_fft = Time.realtimeSinceStartup;
        System.Numerics.Complex[] outputSignal_Freq = FastFourierTransform.FFT(H, false);
        double RSS = 0;

        float t_H = Time.realtimeSinceStartup;
        for (int i = 0; i < H.Length; i++)
        { H[i] = H_LoS[i] + H_NLoS[i]; }
        for (int i =0; i < MA_H_ToT.Length; i++)
        { MA_H_ToT[i] = MA_H_LoS[i] + MA_H_NLoS[i]; }

        float absH = (float)((H[0].Real) * (H[0].Real) + (H[0].Imaginary) * (H[0].Imaginary));
        float SNR = 10 * Mathf.Log10(absH * PowerInMilliWatts) - NoiseLevelInDB;
        //Debug.Log("SNR no MRC = " + SNR);

        // MRC

        int MRC_car_ID = 0;
        int MRC_ant_ID = 0;
        List<int> links_IDs = new List<int>();
        for (int i = 0; i < Channel_Links.Length; i++)
        {
            if (Channel_Links[i].Car1 == MRC_car_ID && Channel_Links[i].V2Antenna == MRC_ant_ID)
            {
                links_IDs.Add(i);
            }
        }

        double MRC_H = 0;
        for (int i = 0; i < links_IDs.Count; i++)
        {
            MRC_H += MA_H_ToT[0 + links_IDs[i] * FFTNum].Real* MA_H_ToT[0 + links_IDs[i] * FFTNum].Real + MA_H_ToT[0 + links_IDs[i] * FFTNum].Imaginary* MA_H_ToT[0 + links_IDs[i] * FFTNum].Imaginary;
        }
        float abs_MRC_H = (float)MRC_H;
        float SNR_MRC = 10 * Mathf.Log10(abs_MRC_H * PowerInMilliWatts) - NoiseLevelInDB;

        SNR_avg += SNR;
        MRC_SNR_avg += SNR_MRC;
        if (FrameCounter % 10 == 0)
        {
            SNR_avg /= 10;
            MRC_SNR_avg /= 10;
            Debug.Log("SNR = " + SNR_avg + "; SNR MRC = " + MRC_SNR_avg);
            SNR_Text.text = "SNR = " + SNR_avg.ToString("F2") + "; SNR MRC = " + MRC_SNR_avg.ToString("F2");
            SNR_avg = 0;
            MRC_SNR_avg = 0;
        }
        

        //Debug.Log("Time spent for H calculation: " + ((Time.realtimeSinceStartup - t_H) * 1000f) + " ms");

        Y_output = new double[H.Length];
        H_output = new double[H.Length];
        
        List<string> h_snapshot = new List<string>();
        List<string> H_snapshot = new List<string>();

        for (int i = 0; i < H.Length; i++)
        {
            
            Y_output[i] = 10 * Mathf.Log10( Mathf.Pow( (float)System.Numerics.Complex.Abs(outputSignal_Freq[i]), 2 ) + 0.0000000000001f);
            H_output[i] = 10 * Mathf.Log10( Mathf.Pow( (float)System.Numerics.Complex.Abs(H[i]), 2 ) + 0.0000000000001f);


            RSS += Mathf.Pow((float)System.Numerics.Complex.Abs(H_LoS[i]), 2);// * SubframePower;
            

            // procedure to write to a file
            string h_string = Mathf.Pow((float)System.Numerics.Complex.Abs(outputSignal_Freq[i]), 2).ToString();
            h_snapshot.Add(h_string); // channel in time domain

            // apparently, we need to convert a complex number to a string using such a weird method QUICK FIX
            
            string H_string;
            if (H[i].Imaginary > 0)
            {
                H_string = H[i].Real.ToString() + "+" + H[i].Imaginary.ToString() + "i";
                //double H_real = H[i].Real;
                //double H_imag = H[i].Imaginary;
            }
            else // in case of negative imaginary part
            {
                H_string = H[i].Real.ToString() + H[i].Imaginary.ToString() + "i";
            }
            H_snapshot.Add(H_string); // channel in frequence domain
            
        }

        /*
        if (RSS != 0)
        {
            //Debug.Log("RSS = " + 10 * Mathf.Log10((float)RSS));

            float test_distance = (CarCoordinates[0] - CarCoordinates[1]).magnitude;
            float test_gain = 10 * Mathf.Log10(Mathf.Pow( 1/(InverseWavelengths[0] * 4 * Mathf.PI * test_distance) , 2));
            Debug.Log("RSS = " + 10 * Mathf.Log10((float)RSS) + " dB; distance " + test_distance + "; path gain = " + test_gain);
        }
        */

        h_save.Add(h_snapshot);
        H_save.Add(H_snapshot);

        Drawing.drawChart(tfTime, X_inputValues, Y_output, "time");
        Drawing.drawChart(tfFreq, X_inputValues, H_output, "frequency");
        //Debug.Log("Time spent for FFT: " + ((Time.realtimeSinceStartup - t_fft) * 1000f) + " ms");
        #endregion

        link_ids.Dispose();
        nonzero_indexes.Dispose();
        map.Dispose();
        idarray.Dispose();

        MA_link_ids.Dispose();
        MA_map.Dispose();
        MA_idarray.Dispose();
        MA_nonzero_indexes.Dispose();

        //DMC_Paths.Dispose();
        //DMC_test.Dispose();
        //MPC1_Paths.Dispose();
        //MPC1_test.Dispose();
    }
    
   

    
}

[BurstCompile]
public struct IndexNonZeroFilter : IJobParallelForFilter
{
    public NativeArray<int> Array;
    public bool Execute(int index)
    {
        return Array[index] == 1;
    }
}

[BurstCompile]
public struct LinkIndexes : IJobParallelFor
{
    [ReadOnly] public int MPCNum;
    [ReadOnly] public NativeArray<int> NonZeroIndexes;

    [WriteOnly] public NativeArray<Vector2Int> LinkIDs;
    public void Execute(int link_id)
    {
        int min_i = NonZeroIndexes.Length;
        int max_i = 0;
        for (int i = 0; i < NonZeroIndexes.Length; i++)
        {
            if (NonZeroIndexes[i] >= link_id * MPCNum && i < min_i)
            { min_i = i; }
            if (NonZeroIndexes[i] <= (link_id + 1) * MPCNum && i > max_i)
            { max_i = i; }
        }
        LinkIDs[link_id] = new Vector2Int(min_i, max_i);
    }
}

public struct ChannelLinks
{
    public int Car1;
    public int Car2;
    public int V1Antenna;
    public int V1AntennaID;
    public int V2Antenna;
    public int V2AntennaID;
    public ChannelLinks(int car1, int car2, int ant1, int ant1ID, int ant2, int ant2ID)
    {
        Car1 = car1;
        Car2 = car2;
        V1Antenna = ant1;
        V1AntennaID = ant1ID;
        V2Antenna = ant2;
        V2AntennaID = ant2ID;
    }
}

public struct CarIDAntIDAntPos
{
    public int CarID;
    public int AntID;
    public Vector3 AntPos;
    public CarIDAntIDAntPos(int carID, int antID, Vector3 antPos)
    {
         CarID = carID;
         AntID = antID;
        AntPos = antPos;
    }
}

public struct ChannelLinksCoordinates
{
    public Vector3 Ant1Coordinates;
    public Vector3 Ant2Coordinates;
    
    public ChannelLinksCoordinates(Vector3 coord1, Vector3 coord2)
    {
        Ant1Coordinates = coord1;
        Ant2Coordinates = coord2;
    }
}

public struct Path_and_IDs
{
    public PathChain ChainIDs;
    public Path PathParameters;
    public int PathOrder;

    public Path_and_IDs(PathChain pathChain, Path pathParameters, int order)
    {
        ChainIDs = pathChain;
        PathParameters = pathParameters;
        PathOrder = order;
    }
}

public struct PathChain
{
    // for tracking the all paths inclusions
    public int Car1;
    public int ID1;
    public int ID2;
    public int ID3;
    public int Car2;
    public PathChain(int car1, int i1, int i2, int i3, int car2)
    {
        Car1 = car1;
        ID1 = i1;
        ID2 = i2;
        ID3 = i3;
        Car2 = car2;
    }
}

public struct Path
{
    public Vector2Int Car_IDs;
    public float Distance;
    public float Attenuation;
    public Path(Vector2Int link, float d, float att)
    {
        Car_IDs = link;
        Distance = d;
        Attenuation = att;
    }
}

public struct MA_Path_and_IDs
{
    public MA_PathChain ChainIDs;
    public MA_Path PathParameters;
    public int PathOrder;

    public MA_Path_and_IDs(MA_PathChain pathChain, MA_Path pathParameters, int order)
    {
        ChainIDs = pathChain;
        PathParameters = pathParameters;
        PathOrder = order;
    }
}
public struct MA_PathChain
{
    // for tracking the all paths inclusions
    public ChannelLinks ChLink;
    public int ID1;
    public int ID2;
    public int ID3;
    
    public MA_PathChain(ChannelLinks chlink, int i1, int i2, int i3)
    {
        ChLink = chlink;
        ID1 = i1;
        ID2 = i2;
        ID3 = i3;
    }
}
public struct MA_Path
{
    public ChannelLinks ChLink;
    public float Distance;
    public float Attenuation;
    public MA_Path(ChannelLinks link, float d, float att)
    {
        ChLink = link;
        Distance = d;
        Attenuation = att;
    }
}

