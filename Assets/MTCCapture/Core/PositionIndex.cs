// PositionIndex.cs
//
// Standalone 24-joint index enum used by the capture + trim pipeline.
//
// In the original "My project" this enum lived inside RealTimeSMPLX.cs, which
// transitively pulled in the third-party SMPL-X package to compile. It is
// extracted here so the capture path (PXR_BodyTrackingFeed -> PXR_24PointMapper
// -> session scripts) and the replay tool build WITHOUT SMPL-X. Values and order
// are identical to the original (matches the PICO BodyTrackerRole / SMPL 24-joint
// ordering used throughout the CSV format).
//
// NOTE: if you later add the optional live SMPL-X avatar (RealTimeSMPLX.cs from
// the original project, which ALSO defines PositionIndex), delete this file to
// avoid a duplicate-definition compile error.

/// <summary>
/// Position index of the 24 body joints (pelvis-rooted SMPL ordering).
/// </summary>
public enum PositionIndex : int
{
    hip = 0,            // pelvis
    left_hip,           // 1
    right_hip,          // 2
    spine1,             // 3
    left_knee,          // 4
    right_knee,         // 5
    spine2,             // 6
    left_ankle,         // 7
    right_ankle,        // 8
    spine3,             // 9
    left_foot_index,    // 10
    right_foot_index,   // 11
    neck,               // 12
    left_collar,        // 13
    right_collar,       // 14
    head,               // 15
    left_shoulder,      // 16
    right_shoulder,     // 17
    left_elbow,         // 18
    right_elbow,        // 19
    left_wrist,         // 20
    right_wrist,        // 21
    left_hand,          // 22
    right_hand,         // 23
    Count
}

public static partial class EnumExtend
{
    public static int Int(this PositionIndex i)
    {
        return (int)i;
    }
}
