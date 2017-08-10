using Valve.VR;
using System.Collections.Generic;
using System;
using System.Threading;
using System.Threading.Tasks;

namespace ViveTrackerFreePieBridge.net
{
    class Program
    {
        static Dictionary<string, List<string>> names = new Dictionary<string, List<string>>();
        static Dictionary<string, vr_tracked_device> devices = new Dictionary<string, vr_tracked_device>();
        static CVRSystem vr;

        static Timer resetTimer;
        static bool timerCalled = false;

        const float rad_to_deg = 57.295779513082320876798154814105f;

        static void Main(string[] args)
        {
            var task = Task.Run(() =>
            {
                var initError = default(EVRInitError);

                while (true)
                {
                    try
                    {
                        vr = OpenVR.Init(ref initError, EVRApplicationType.VRApplication_Background);
                        if (vr == null)
                        {
                            Thread.Sleep(100);
                            continue;
                        }

                        EnumerateDevices();

                        while (true)
                        {
                            var enumerate = !devices.ContainsKey("tracker_1");

                            if (!enumerate)
                            {
                                var mat = devices["tracker_1"].get_position();
                                var trans = new RigidTransform(mat);
                                var pos = trans.pos;
                                var rot = VectorFromQuaternion(trans.rot);
                                var data = new FreepieWriter.FreepieWriter.FreepieData();

                                //data.x = (float)(pos.v0 * -100f);
                                //data.y = (float)(-100f + (pos.v1 * 100f));
                                //data.z = (float)(pos.v2 * -100f);

                                data.x = (mat.m3 * -100f);
                                data.y = -100f + (mat.m7 * 100f);
                                data.z = mat.m11 * -100f;

                                //var forward = new HmdVector3d_t { v0 = 0f, v1 = 0f, v2 = -1f };
                                //var rot = mult(forward, mat);

                                // data.pitch = 180f + (float)vec.v1 * rad_to_deg;
                                data.yaw = 180f + (float)NormalizeAngle((rot.v0 * rad_to_deg));
                                // data.roll = -(float)vec.v2 * rad_to_deg;

                                FreepieWriter.FreepieWriter.WriteData(data, 0);
                            }

                            if (enumerate)
                            {
                                EnumerateDevices();
                                Thread.Sleep(100);
                                continue;
                            }

                            Thread.Sleep(1);
                        }
                    }
                    catch
                    {
                        Thread.Sleep(100);
                    }
                }
            });

            Task.WaitAny(task);
        }

        private static void EnumerateDevices()
        {
            names.Clear();
            devices.Clear();
            names.Add("Tracking Reference", new List<string>());
            names.Add("Controller", new List<string>());
            names.Add("Tracker", new List<string>());
            names.Add("HMD", new List<string>());

            var poses = new TrackedDevicePose_t[OpenVR.k_unMaxTrackedDeviceCount];
            vr.GetDeviceToAbsoluteTrackingPose(ETrackingUniverseOrigin.TrackingUniverseSeated, 0, poses);

            for (uint i = 0; i < poses.Length; i++)
            {
                var pose = poses[i];
                if (pose.bPoseIsValid)
                {
                    var name = string.Empty;

                    var _class = vr.GetTrackedDeviceClass(i);
                    if (_class == ETrackedDeviceClass.Controller)
                    {
                        name = "controller_" + (names["Controller"].Count + 1);
                        names["Controller"].Add(name);
                        devices[name] = new vr_tracked_device(ref vr, i, "Controller");
                    }
                    else if (_class == ETrackedDeviceClass.HMD)
                    {
                        name = "hmd_" + (names["HMD"].Count + 1);
                        names["HMD"].Add(name);
                        devices[name] = new vr_tracked_device(ref vr, i, "HMD");
                    }
                    else if (_class == ETrackedDeviceClass.GenericTracker)
                    {
                        name = "tracker_" + (names["Tracker"].Count + 1);
                        names["Tracker"].Add(name);
                        devices[name] = new vr_tracked_device(ref vr, i, "Tracker");
                    }
                    else if (_class == ETrackedDeviceClass.TrackingReference)
                    {
                        name = "tracking_reference_" + (names["Tracking Reference"].Count + 1);
                        names["Tracking Reference"].Add(name);
                        devices[name] = new vr_tracked_device(ref vr, i, "Tracking Reference");
                    }
                }
            }

            print_discovered_objects();
        }

        private static void print_discovered_objects()
        {
            foreach (var key in names.Keys)
            {
                var plural = key;

                if (names[key].Count != 1)
                    plural += "s";

                Console.WriteLine("Found " + names[key].Count + " " + plural);

                foreach (var device in names[key])
                {
                    if (device == "Tracking Reference")
                    {
                        Console.WriteLine("  " + device + " (" + devices[device].get_serial() +
                            ", Mode " + devices[device].get_mode() +
                            ", " + devices[device].get_model() +
                            ")");
                    }
                    else
                    {
                        Console.WriteLine("  " + device + " (" + devices[device].get_serial() +
                            ", " + devices[device].get_model() +
                            ")");
                    }
                }
            }
        }

        public static HmdVector3d_t VectorFromQuaternion(HmdQuaternion_t q1)
        {
            HmdVector3d_t v;

            var sqw = q1.w * q1.w;
            var sqx = q1.x * q1.x;
            var sqy = q1.y * q1.y;
            var sqz = q1.z * q1.z;
            var unit = sqx + sqy + sqz + sqw; // if normalised is one, otherwise is correction factor
            var test = q1.x * q1.w - q1.y * q1.z;

            if (test > 0.4995f * unit)
            { // singularity at north pole
                v.v0 = Math.PI / 2;
                v.v1 = 2f * Math.Atan2(q1.y, q1.x);
                v.v2 = 0;
                return v;
            }
            if (test < -0.4995f * unit)
            { // singularity at south pole
                v.v0 = -Math.PI / 2;
                v.v1 = -2f * Math.Atan2(q1.y, q1.x);
                v.v2 = 0;
                return v;
            }

            var q = new HmdQuaternion_t { w = q1.w, x = q1.x, y = q1.y, z = q1.z };
            v.v0 = (float)Math.Asin(2f * (q.x * q.z - q.w * q.y));                                            // yaw
            v.v1 = (float)Math.Atan2(2f * q.x * q.w + 2f * q.y * q.z, 1 - 2f * (q.z * q.z + q.w * q.w));      // pitch
            v.v2 = (float)Math.Atan2(2f * q.x * q.y + 2f * q.z * q.w, 1 - 2f * (q.y * q.y + q.z * q.z));      // roll

            return v;
        }

        public static HmdVector3d_t mult(HmdVector3d_t vec, HmdMatrix34_t mat)
        {
            var x = vec.v0;
            var y = vec.v1;
            var z = vec.v2;

            return new HmdVector3d_t
            {
                v0 = mat.m0 * x + mat.m1 * y + mat.m2 * z
                //v0 = mat.m0 * x + mat.m1 * y + mat.m2 * z,
                //v1 = mat.m4 * x + mat.m5 * y + mat.m6 * z,
                //v2 = mat.m8 * x + mat.m9 * y + mat.m10 * z
            };
        }

        static double NormalizeAngle(double angle)
        {
            while (angle > 360)
                angle -= 360;
            while (angle < 0)
                angle += 360;
            return angle;
        }
    }
}
