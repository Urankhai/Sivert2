using UnityEngine;
using Unity.Collections;
using Unity.Jobs;
using Unity.Burst;

[BurstCompile]
public struct ParallelChannel : IJobParallelFor
{
    [ReadOnly] public int FFTSize;
    [ReadOnly] public NativeArray<Vector3> CarsPositions;
    [ReadOnly] public NativeArray<Vector3> CarsFwd;
    //[ReadOnly] public NativeArray<Vector2Int> Links;
    [ReadOnly] public NativeArray<float> InverseLambdas;
    [ReadOnly] public NativeMultiHashMap<int, Path_and_IDs> HashMap;
    [ReadOnly] public int MPCNum;
    [ReadOnly] public int LinkNum;
    [ReadOnly] public NativeArray<int> NonZeroIndexes;
    [ReadOnly] public NativeArray<Vector2Int> LinkIDs;

    [WriteOnly] public NativeArray<System.Numerics.Complex> H_NLoS;
    public void Execute(int index)
    {
        int i_link = Mathf.FloorToInt(index / FFTSize);
        int i_sub = index - i_link * FFTSize; // calculating index within FFT array

        // defining zero temp value
        System.Numerics.Complex temp_HNLoS = new System.Numerics.Complex(0, 0);


        
        //for (int i = i_link * MPCNum; i < (i_link + 1) * MPCNum; i++)
        for (int i = LinkIDs[i_link].x; i <= LinkIDs[i_link].y; i++)
        {
            
            if (HashMap.TryGetFirstValue(NonZeroIndexes[i], out Path_and_IDs path, out NativeMultiHashMapIterator<int> nativeMultiHashMapIterator))
            {
                // if there are many paths, then sum them up
                do
                {
                    float HNLoS_dist = path.PathParameters.Distance;

                    //float frequence = 299792458.0f * InverseLambdas[i_sub];
                    //float inverse_lambda = InverseLambdas[i_sub];
                    //float test_pi = Mathf.PI;
                    

                    float HNLoS_dist_gain = 1.0f / (InverseLambdas[i_sub] * 4 * Mathf.PI * HNLoS_dist); // Free space loss
                    float HNLoS_attnuation = path.PathParameters.Attenuation;

                    //float test_arg = 2.0f * Mathf.PI * InverseLambdas[i_sub] * HNLoS_dist;

                    double ReExpNLoS = Mathf.Cos(2.0f * Mathf.PI * InverseLambdas[i_sub] * HNLoS_dist);
                    double ImExpNLoS = Mathf.Sin(2.0f * Mathf.PI * InverseLambdas[i_sub] * HNLoS_dist);
                    // defining exponent
                    System.Numerics.Complex ExpNLoS = new System.Numerics.Complex(ReExpNLoS, ImExpNLoS);

                    temp_HNLoS += HNLoS_attnuation * HNLoS_dist_gain * ExpNLoS;                    

                }
                while (HashMap.TryGetNextValue(out path, ref nativeMultiHashMapIterator));
            }
        }
        H_NLoS[index] = temp_HNLoS;        
    }
}

[BurstCompile]
public struct MA_ParallelChannel : IJobParallelFor
{
    [ReadOnly] public int FFTSize;
    //[ReadOnly] public NativeArray<Vector2Int> Links;
    [ReadOnly] public NativeArray<float> InverseLambdas;
    [ReadOnly] public NativeMultiHashMap<int, MA_Path_and_IDs> HashMap;
    [ReadOnly] public int MPCNum;
    [ReadOnly] public int LinkNum;
    [ReadOnly] public NativeArray<int> NonZeroIndexes;
    [ReadOnly] public NativeArray<Vector2Int> LinkIDs;

    [WriteOnly] public NativeArray<System.Numerics.Complex> H_NLoS;
    public void Execute(int index)
    {
        int i_link = Mathf.FloorToInt(index / FFTSize);
        int i_sub = index - i_link * FFTSize; // calculating index within FFT array

        // defining zero temp value
        System.Numerics.Complex temp_HNLoS = new System.Numerics.Complex(0, 0);



        //for (int i = i_link * MPCNum; i < (i_link + 1) * MPCNum; i++)
        for (int i = LinkIDs[i_link].x; i <= LinkIDs[i_link].y; i++)
        {

            if (HashMap.TryGetFirstValue(NonZeroIndexes[i], out MA_Path_and_IDs path, out NativeMultiHashMapIterator<int> nativeMultiHashMapIterator))
            {
                // if there are many paths, then sum them up
                do
                {
                    float HNLoS_dist = path.PathParameters.Distance;

                    //float frequence = 299792458.0f * InverseLambdas[i_sub];
                    //float inverse_lambda = InverseLambdas[i_sub];
                    //float test_pi = Mathf.PI;


                    float HNLoS_dist_gain = 1.0f / (InverseLambdas[i_sub] * 4 * Mathf.PI * HNLoS_dist); // Free space loss
                    float HNLoS_attnuation = path.PathParameters.Attenuation;

                    //float test_arg = 2.0f * Mathf.PI * InverseLambdas[i_sub] * HNLoS_dist;

                    double ReExpNLoS = Mathf.Cos(2.0f * Mathf.PI * InverseLambdas[i_sub] * HNLoS_dist);
                    double ImExpNLoS = Mathf.Sin(2.0f * Mathf.PI * InverseLambdas[i_sub] * HNLoS_dist);
                    // defining exponent
                    System.Numerics.Complex ExpNLoS = new System.Numerics.Complex(ReExpNLoS, ImExpNLoS);

                    temp_HNLoS += HNLoS_attnuation * HNLoS_dist_gain * ExpNLoS;

                }
                while (HashMap.TryGetNextValue(out path, ref nativeMultiHashMapIterator));
            }
        }
        H_NLoS[index] = temp_HNLoS;
    }
}

