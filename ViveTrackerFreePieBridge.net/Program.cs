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
                                var pos = devices["tracker_1"].get_position();
                                var data = new FreepieWriter.FreepieWriter.FreepieData();

                                data.x = (pos.m3 * -100f);
                                data.y = -100f + (pos.m7 * 100f);
                                data.z = pos.m11 * -100f;

                                var quat = QuaternionFromRotationMatrix(pos);
                                var vec = VectorFromQuaternion(quat);

                                // data.pitch = 180f + (float)vec.v1 * rad_to_deg;
                                data.yaw = ((float)vec.v0 * rad_to_deg) + 90f;
                                // data.roll = -(float)vec.v2 * rad_to_deg;

                                //data.roll = 180 - rad_to_deg * (float)Math.Atan2(pos.m4, pos.m0);
                                //data.yaw = rad_to_deg * (float)Math.Atan2(pos.m8, Math.Sqrt(Math.Pow(pos.m9, 2) + Math.Pow(pos.m10, 2)));
                                //data.pitch = rad_to_deg * (float)Math.Atan2(pos.m9, pos.m10);

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

        public static HmdQuaternion_t QuaternionFromRotationMatrix(HmdMatrix34_t matrix)
        {
            double num8 = (matrix.m0 + matrix.m5) + matrix.m10;
            HmdQuaternion_t quaternion = new HmdQuaternion_t();
            if (num8 > 0f)
            {
                double num = (double)Math.Sqrt((double)(num8 + 1f));
                quaternion.w = num * 0.5f;
                num = 0.5f / num;
                quaternion.x = (matrix.m6 - matrix.m9) * num;
                quaternion.y = (matrix.m8 - matrix.m2) * num;
                quaternion.z = (matrix.m1 - matrix.m4) * num;
                return quaternion;
            }
            if ((matrix.m0 >= matrix.m5) && (matrix.m0 >= matrix.m10))
            {
                double num7 = (double)Math.Sqrt((double)(((1f + matrix.m0) - matrix.m5) - matrix.m10));
                double num4 = 0.5f / num7;
                quaternion.x = 0.5f * num7;
                quaternion.y = (matrix.m1 + matrix.m4) * num4;
                quaternion.z = (matrix.m2 + matrix.m8) * num4;
                quaternion.w = (matrix.m6 - matrix.m9) * num4;
                return quaternion;
            }
            if (matrix.m5 > matrix.m10)
            {
                double num6 = (double)Math.Sqrt((double)(((1f + matrix.m5) - matrix.m0) - matrix.m10));
                double num3 = 0.5f / num6;
                quaternion.x = (matrix.m4 + matrix.m1) * num3;
                quaternion.y = 0.5f * num6;
                quaternion.z = (matrix.m9 + matrix.m6) * num3;
                quaternion.w = (matrix.m8 - matrix.m2) * num3;
                return quaternion;
            }
            double num5 = (double)Math.Sqrt((double)(((1f + matrix.m10) - matrix.m0) - matrix.m5));
            double num2 = 0.5f / num5;
            quaternion.x = (matrix.m8 + matrix.m2) * num2;
            quaternion.y = (matrix.m9 + matrix.m6) * num2;
            quaternion.z = 0.5f * num5;
            quaternion.w = (matrix.m1 - matrix.m4) * num2;

            //double num8 = (matrix.M11 + matrix.M22) + matrix.M33;
            //HmdQuaternion_t quaternion = new HmdQuaternion_t();
            //if (num8 > 0f)
            //{
            //    double num = (double)Math.Sqrt((double)(num8 + 1f));
            //    quaternion.w = num * 0.5f;
            //    num = 0.5f / num;
            //    quaternion.x = (matrix.M23 - matrix.M32) * num;
            //    quaternion.y = (matrix.M31 - matrix.M13) * num;
            //    quaternion.z = (matrix.M12 - matrix.M21) * num;
            //    return quaternion;
            //}
            //if ((matrix.M11 >= matrix.M22) && (matrix.M11 >= matrix.M33))
            //{
            //    double num7 = (double)Math.Sqrt((double)(((1f + matrix.M11) - matrix.M22) - matrix.M33));
            //    double num4 = 0.5f / num7;
            //    quaternion.x = 0.5f * num7;
            //    quaternion.y = (matrix.M12 + matrix.M21) * num4;
            //    quaternion.z = (matrix.M13 + matrix.M31) * num4;
            //    quaternion.w = (matrix.M23 - matrix.M32) * num4;
            //    return quaternion;
            //}
            //if (matrix.M22 > matrix.M33)
            //{
            //    double num6 = (double)Math.Sqrt((double)(((1f + matrix.M22) - matrix.M11) - matrix.M33));
            //    double num3 = 0.5f / num6;
            //    quaternion.x = (matrix.M21 + matrix.M12) * num3;
            //    quaternion.y = 0.5f * num6;
            //    quaternion.z = (matrix.M32 + matrix.M23) * num3;
            //    quaternion.w = (matrix.M31 - matrix.M13) * num3;
            //    return quaternion;
            //}
            //double num5 = (double)Math.Sqrt((double)(((1f + matrix.M33) - matrix.M11) - matrix.M22));
            //double num2 = 0.5f / num5;
            //quaternion.x = (matrix.M31 + matrix.M13) * num2;
            //quaternion.y = (matrix.M32 + matrix.M23) * num2;
            //quaternion.z = 0.5f * num5;
            //quaternion.w = (matrix.M12 - matrix.M21) * num2;

            return quaternion;

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

            return NormalizeAngles(v);
        }

        static HmdVector3d_t NormalizeAngles(HmdVector3d_t angles)
        {
            angles.v0 = NormalizeAngle(angles.v0);
            angles.v1 = NormalizeAngle(angles.v1);
            angles.v2 = NormalizeAngle(angles.v2);
            return angles;
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
