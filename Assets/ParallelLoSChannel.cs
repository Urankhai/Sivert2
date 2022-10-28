using UnityEngine;
using Unity.Collections;
using Unity.Jobs;
using Unity.Burst;


[BurstCompile]
public struct MA_ParallelLoSDetection : IJobParallelFor
{
    [ReadOnly] public NativeArray<ChannelLinksCoordinates> AntennaPositions;

    [WriteOnly] public NativeArray<RaycastCommand> RayCastcommands;
    public void Execute(int index)
    {
        Vector3 antenna1 = AntennaPositions[index].Ant1Coordinates;
        Vector3 antenna2 = AntennaPositions[index].Ant2Coordinates;

        Vector3 temp_direction = antenna2 - antenna1;
        RayCastcommands[index] = new RaycastCommand(antenna1, temp_direction.normalized, temp_direction.magnitude);

    }
}

//[BurstCompile]
public struct MA_ParallelLoSChannel : IJobParallelFor
{
    [ReadOnly] public bool OmniAntennaFlag;
    [ReadOnly] public int FFTSize;
    [ReadOnly] public NativeArray<ChannelLinksCoordinates> AntennasPositions;
    [ReadOnly] public NativeArray<ChannelLinks> Channel_Links;
    [ReadOnly] public NativeArray<Vector3> CarsFwd;
    [ReadOnly] public NativeArray<RaycastHit> MA_raycastresults;
    [ReadOnly] public NativeArray<float> inverseLambdas;
    [ReadOnly] public NativeArray<Vector2> Pattern;

    [ReadOnly] public NativeArray<Vector2> EADFs; // all eadf values in one nativearray
    [ReadOnly] public NativeArray<V4Int> EADF_link_ranges; // this structure points to positions of a particular EADF in EADFs


    [WriteOnly] public NativeArray<System.Numerics.Complex> HLoS;

    public void Execute(int index)
    {
        int i_link = Mathf.FloorToInt(index / FFTSize);


        if (MA_raycastresults[i_link].distance == 0)
        {

            int i_sub = index - i_link * FFTSize; // calculating index within FFT array

            Vector3 ant1 = AntennasPositions[i_link].Ant1Coordinates; //CarsPositions[Links[i_link].x];
            Vector3 ant2 = AntennasPositions[i_link].Ant2Coordinates;

            Vector3 fwd1 = CarsFwd[Channel_Links[i_link].Car1];
            Vector3 fwd2 = CarsFwd[Channel_Links[i_link].Car2];

            float GroundReflCoef = 0.7f;


            Vector3 LoS_dir = ant2 - ant1;
            Vector3 LoS_dir_flat = new Vector3(LoS_dir.x, 0, LoS_dir.z);
            Vector3 LoS_dir_nrom = LoS_dir_flat.normalized;


            
            //if (OmniAntennaFlag == false)
            //{
                float phi1 = Mathf.Acos(Vector3.Dot(fwd1, LoS_dir_nrom));
                var pattern1 = new NativeSlice<Vector2>(EADFs, EADF_link_ranges[i_link].EADF1_Lft, EADF_link_ranges[i_link].EADF1_Rng);
                System.Numerics.Complex Gain1 = EADF_rec(pattern1, phi1);

                float phi2 = Mathf.Acos(Vector3.Dot(fwd2, -LoS_dir_nrom));
                var pattern2 = new NativeSlice<Vector2>(EADFs, EADF_link_ranges[i_link].EADF2_Lft, EADF_link_ranges[i_link].EADF2_Rng);
                System.Numerics.Complex Gain2 = EADF_rec(pattern2, phi2);

                System.Numerics.Complex antenna_gain = System.Numerics.Complex.Multiply(Gain1, Gain2);
            //}

            // line of sight parameters

            float LoS_dist = LoS_dir.magnitude;
            float LoS_gain = 1 / (inverseLambdas[i_sub] * 4 * Mathf.PI * LoS_dist);
            //float LoS_gain = 1 / (inverseLambdas[i_sub] * 4 * Mathf.PI * LoS_dist);

            double ReExpLoS = Mathf.Cos(2.0f * Mathf.PI * inverseLambdas[i_sub] * LoS_dist);
            double ImExpLoS = Mathf.Sin(2.0f * Mathf.PI * inverseLambdas[i_sub] * LoS_dist);
            // defining exponent
            System.Numerics.Complex ExpLoS = System.Numerics.Complex.Multiply(new System.Numerics.Complex(ReExpLoS, ImExpLoS), antenna_gain);


            // ground reflection parameters
            float Fresnel_coef = 1 * GroundReflCoef; // TODO: should be calculated correctly
            float totalhight = ant1.y + ant2.y;
            float ground_dist = Mathf.Sqrt(LoS_dist * LoS_dist + totalhight * totalhight);
            float ground_gain = Fresnel_coef / (inverseLambdas[i_sub] * 4 * Mathf.PI * ground_dist);

            double ReExpGround = Mathf.Cos(2.0f * Mathf.PI * inverseLambdas[i_sub] * ground_dist);
            double ImExpGround = Mathf.Sin(2.0f * Mathf.PI * inverseLambdas[i_sub] * ground_dist);
            // defining exponent
            System.Numerics.Complex ExpGround = System.Numerics.Complex.Multiply(new System.Numerics.Complex(ReExpGround, ImExpGround), antenna_gain);

            HLoS[index] = LoS_gain * ExpLoS + ground_gain * ExpGround;

            float asd = 1;
        }
        else
        { HLoS[index] = 0; }

    }


    private System.Numerics.Complex EADF_rec(NativeSlice<Vector2> Pattern, float angle)
    {
        System.Numerics.Complex Gain = 0;
        
        int L = Pattern.Length;
        for (int i = 0; i < L; i++)
        {
            //db1 = exp(1j * ang1.* mu1);
            float mu = -(L - 1) / 2 + i;
            System.Numerics.Complex db = new System.Numerics.Complex(Mathf.Cos(angle * mu), Mathf.Sin(angle * mu));
            
            System.Numerics.Complex complex_pattern = new System.Numerics.Complex(Pattern[i].x, Pattern[i].y);
            
            Gain += System.Numerics.Complex.Multiply(complex_pattern, db);
            
        }
        
        return Gain;
    }
}

[BurstCompile]
public struct ParallelLoSChannel : IJobParallelFor
{
    [ReadOnly] public bool OmniAntennaFlag;
    [ReadOnly] public int FFTSize;
    [ReadOnly] public NativeArray<Vector3> CarsPositions;
    [ReadOnly] public NativeArray<Vector3> CarsFwd;
    [ReadOnly] public NativeArray<Vector2Int> Links;
    [ReadOnly] public NativeArray<RaycastHit> raycastresults;
    [ReadOnly] public NativeArray<float> inverseLambdas;
    [ReadOnly] public NativeArray<Vector2> Pattern;


    [WriteOnly] public NativeArray<System.Numerics.Complex> HLoS;
    
    public void Execute(int index)
    {
        int i_link = Mathf.FloorToInt(index / FFTSize);


        if (raycastresults[i_link].distance == 0)
        {

            int i_sub = index - i_link * FFTSize; // calculating index within FFT array

            Vector3 car1 = CarsPositions[Links[i_link].x];
            Vector3 car2 = CarsPositions[Links[i_link].y];

            Vector3 fwd1 = CarsFwd[Links[i_link].x];
            Vector3 fwd2 = CarsFwd[Links[i_link].y];

            float GroundReflCoef = 0.7f;


            Vector3 LoS_dir = car2 - car1;
            Vector3 LoS_dir_flat = new Vector3(LoS_dir.x, 0, LoS_dir.z);
            Vector3 LoS_dir_nrom = LoS_dir_flat.normalized;



            float antenna_gain = 1;
            if (OmniAntennaFlag == false)
            {
                float phi1 = Mathf.Acos(Vector3.Dot(fwd1, LoS_dir_nrom));
                float phi2 = Mathf.Acos(Vector3.Dot(fwd2, -LoS_dir_nrom));

                antenna_gain = EADF_rec(Pattern, phi1, phi2);
            }

            // line of sight parameters

            float LoS_dist = LoS_dir.magnitude;
            float LoS_gain = antenna_gain / (inverseLambdas[i_sub] * 4 * Mathf.PI * LoS_dist);
            //float LoS_gain = 1 / (inverseLambdas[i_sub] * 4 * Mathf.PI * LoS_dist);

            double ReExpLoS = Mathf.Cos(2.0f * Mathf.PI * inverseLambdas[i_sub] * LoS_dist);
            double ImExpLoS = Mathf.Sin(2.0f * Mathf.PI * inverseLambdas[i_sub] * LoS_dist);
            // defining exponent
            System.Numerics.Complex ExpLoS = new System.Numerics.Complex(ReExpLoS, ImExpLoS);


            // ground reflection parameters
            float Fresnel_coef = antenna_gain * GroundReflCoef; // TODO: should be calculated correctly
            float totalhight = car1.y + car2.y;
            float ground_dist = Mathf.Sqrt(LoS_dist * LoS_dist + totalhight * totalhight);
            float ground_gain = Fresnel_coef / (inverseLambdas[i_sub] * 4 * Mathf.PI * ground_dist);

            double ReExpGround = Mathf.Cos(2.0f * Mathf.PI * inverseLambdas[i_sub] * ground_dist);
            double ImExpGround = Mathf.Sin(2.0f * Mathf.PI * inverseLambdas[i_sub] * ground_dist);
            // defining exponent
            System.Numerics.Complex ExpGround = new System.Numerics.Complex(ReExpGround, ImExpGround);

            HLoS[index] = LoS_gain * ExpLoS + ground_gain * ExpGround;
        }
        else
        { HLoS[index] = 0; }
        
    }
    

    private float EADF_rec(NativeArray<Vector2> Pattern, float angle1, float angle2)
    {
        System.Numerics.Complex Gain1 = 0;
        System.Numerics.Complex Gain2 = 0;
        int L = Pattern.Length;
        for (int i = 0; i < L; i++)
        {
            //db1 = exp(1j * ang1.* mu1);
            float mu = -(L - 1)/2 + i;
            System.Numerics.Complex db1 = new System.Numerics.Complex(Mathf.Cos(angle1 * mu), Mathf.Sin(angle1 * mu));
            System.Numerics.Complex db2 = new System.Numerics.Complex(Mathf.Cos(angle2 * mu), Mathf.Sin(angle2 * mu));

            System.Numerics.Complex complex_pattern = new System.Numerics.Complex(Pattern[i].x, Pattern[i].y);
            Gain1 += System.Numerics.Complex.Multiply(complex_pattern, db1);
            Gain2 += System.Numerics.Complex.Multiply(complex_pattern, db2);
        }
        System.Numerics.Complex mult = System.Numerics.Complex.Multiply(Gain1, Gain2);
        float Gain = (float)System.Numerics.Complex.Abs(mult);
        return Gain;
    }
}



[BurstCompile]
public struct ParallelLoSDetection : IJobParallelFor
{
    [ReadOnly] public NativeArray<Vector3> CarsPositions;
    [ReadOnly] public NativeArray<Vector2Int> Links;
    [WriteOnly] public NativeArray<RaycastCommand> commands;
    public void Execute(int index)
    {
        int number_of_links = Links.Length;
        if (index < number_of_links)
        {
            int TxCarID = Links[index].x;
            int RxCarID = Links[index].y;
            Vector3 temp_direction = CarsPositions[RxCarID] - CarsPositions[TxCarID];
            commands[index] = new RaycastCommand(CarsPositions[TxCarID], temp_direction.normalized, temp_direction.magnitude);
        }
        else
        {
            int TxCarID = Links[index - number_of_links].y;
            int RxCarID = Links[index - number_of_links].x;
            Vector3 temp_direction = CarsPositions[RxCarID] - CarsPositions[TxCarID];
            commands[index] = new RaycastCommand(CarsPositions[TxCarID], temp_direction.normalized, temp_direction.magnitude);
        }
    }
}

