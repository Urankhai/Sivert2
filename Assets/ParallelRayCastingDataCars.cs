using UnityEngine;
using Unity.Collections;
using Unity.Jobs;
using Unity.Burst;

[BurstCompile]
public struct ParallelRayCastingDataCars : IJobParallelFor
{
    [ReadOnly] public float CastingDistance;
    [ReadOnly] public NativeArray<Vector3> Cars_Positions;
    [ReadOnly] public NativeArray<V6> MPC_Array;
    [ReadOnly] public NativeArray<Vector3> MPC_Perpendiculars;

    [WriteOnly] public NativeArray<float> SoA;
    [WriteOnly] public NativeArray<int> SeenIndicator;
    [WriteOnly] public NativeArray<RaycastCommand> commands;
    public void Execute(int index)
    {
        int i_car = Mathf.FloorToInt(index / MPC_Array.Length);
        int i_mpc = index - i_car * MPC_Array.Length;
        Vector3 temp_direction = MPC_Array[i_mpc].Coordinates - Cars_Positions[i_car];

        float cosA = Vector3.Dot(MPC_Array[i_mpc].Normal, -temp_direction.normalized); // NOTE: the sign is negative (upd 24.02.2022: the sign of temp_direction)
        if (cosA > (float)0.1 && temp_direction.magnitude < CastingDistance)
        {
            //SoA[index] = Mathf.Sign(Vector3.Dot(MPC_Perpendiculars[i_mpc], -temp_direction.normalized));
            SoA[index] = Vector3.Dot(MPC_Perpendiculars[i_mpc], -temp_direction.normalized);
            SeenIndicator[index] = 1;
            commands[index] = new RaycastCommand(Cars_Positions[i_car], temp_direction.normalized, temp_direction.magnitude);
        }
        else
        {
            SoA[index] = 0;
            SeenIndicator[index] = 0;
            commands[index] = new RaycastCommand(new Vector3(0, 0, 0), new Vector3(0, 0, 0), 0);
        }

    }
}

[BurstCompile]
public struct ParallelRayCastingDataAntennas : IJobParallelFor
{
    [ReadOnly] public float CastingDistance;
    [ReadOnly] public NativeArray<Vector3> Antennas_Positions;
    [ReadOnly] public NativeArray<V6> MPC_Array;
    [ReadOnly] public NativeArray<Vector3> MPC_Perpendiculars;

    [WriteOnly] public NativeArray<float> MA_SoA;
    [WriteOnly] public NativeArray<int> MA_SeenIndicator;
    [WriteOnly] public NativeArray<RaycastCommand> MA_commands;
    public void Execute(int index)
    {
        int i_ant = Mathf.FloorToInt(index / MPC_Array.Length);
        int i_mpc = index - i_ant * MPC_Array.Length;
        Vector3 temp_direction = MPC_Array[i_mpc].Coordinates - Antennas_Positions[i_ant];

        float cosA = Vector3.Dot(MPC_Array[i_mpc].Normal, -temp_direction.normalized); // NOTE: the sign is negative (upd 24.02.2022: the sign of temp_direction)
        if (cosA > (float)0.1 && temp_direction.magnitude < CastingDistance)
        {
            //SoA[index] = Mathf.Sign(Vector3.Dot(MPC_Perpendiculars[i_mpc], -temp_direction.normalized));
            MA_SoA[index] = Vector3.Dot(MPC_Perpendiculars[i_mpc], -temp_direction.normalized);
            MA_SeenIndicator[index] = 1;
            MA_commands[index] = new RaycastCommand(Antennas_Positions[i_ant], temp_direction.normalized, temp_direction.magnitude);
        }
        else
        {
            MA_SoA[index] = 0;
            MA_SeenIndicator[index] = 0;
            MA_commands[index] = new RaycastCommand(new Vector3(0, 0, 0), new Vector3(0, 0, 0), 0);
        }

    }
}