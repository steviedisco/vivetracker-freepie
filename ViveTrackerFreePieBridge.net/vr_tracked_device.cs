using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Valve.VR;

internal class vr_tracked_device
{
    protected readonly CVRSystem vr;
    protected readonly uint index;
    protected readonly string device_class;

    public vr_tracked_device(ref CVRSystem vr_obj, uint index, string device_class)
    {
        this.vr = vr_obj;
        this.index = index;
        this.device_class = device_class;
    }

    public string get_serial()
    {
        var error = default(ETrackedPropertyError);
        var sb = new StringBuilder();
        var ret = vr.GetStringTrackedDeviceProperty(index, ETrackedDeviceProperty.Prop_SerialNumber_String, sb, 1024, ref error);

        return sb.ToString();
    }

    public string get_model()
    {
        var error = default(ETrackedPropertyError);
        var sb = new StringBuilder();
        var ret = vr.GetStringTrackedDeviceProperty(index, ETrackedDeviceProperty.Prop_ModelNumber_String, sb, 1024, ref error);

        return sb.ToString();
    }

    public string get_mode()
    {
        var error = default(ETrackedPropertyError);
        var sb = new StringBuilder();
        var ret = vr.GetStringTrackedDeviceProperty(index, ETrackedDeviceProperty.Prop_ModeLabel_String, sb, 1024, ref error);

        return sb.ToString();
    }

    public HmdMatrix34_t get_position()
    {
        var poses = new TrackedDevicePose_t[OpenVR.k_unMaxTrackedDeviceCount];
        vr.GetDeviceToAbsoluteTrackingPose(ETrackingUniverseOrigin.TrackingUniverseRawAndUncalibrated, 0, poses);

        var pose = poses[index];
        return pose.mDeviceToAbsoluteTracking;
    }
}
