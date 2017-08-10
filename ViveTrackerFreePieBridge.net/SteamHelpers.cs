using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Valve.VR;

namespace ViveTrackerFreePieBridge.net
{
    public struct Matrix4x4
    {
        public float m00;
        public float m01;
        public float m02;
        public float m03;
        public float m10;
        public float m11;
        public float m12;
        public float m13;
        public float m20;
        public float m21;
        public float m22;
        public float m23;
        public float m30;
        public float m31;
        public float m32;
        public float m33;

        public static HmdQuaternion_t GetRotation(Matrix4x4 matrix)
        {
            HmdQuaternion_t q = new HmdQuaternion_t();
            q.w = Math.Sqrt(Math.Max(0, 1 + matrix.m00 + matrix.m11 + matrix.m22)) / 2;
            q.x = Math.Sqrt(Math.Max(0, 1 + matrix.m00 - matrix.m11 - matrix.m22)) / 2;
            q.y = Math.Sqrt(Math.Max(0, 1 - matrix.m00 + matrix.m11 - matrix.m22)) / 2;
            q.z = Math.Sqrt(Math.Max(0, 1 - matrix.m00 - matrix.m11 + matrix.m22)) / 2;
            q.x = _copysign(q.x, matrix.m21 - matrix.m12);
            q.y = _copysign(q.y, matrix.m02 - matrix.m20);
            q.z = _copysign(q.z, matrix.m10 - matrix.m01);
            return q;
        }

        public static HmdVector3d_t GetPosition( Matrix4x4 matrix)
        {
            var x = matrix.m03;
            var y = matrix.m13;
            var z = matrix.m23;

            return new HmdVector3d_t { v0 = x, v1 = y, v2 = z };
        }

        private static double _copysign(double sizeval, double signval)
        {
            return Math.Sign(signval) == 1 ? Math.Abs(sizeval) : -Math.Abs(sizeval);
        }
    }

    public struct RigidTransform
    {
        public HmdVector3d_t  pos;
        public HmdQuaternion_t rot;

        public RigidTransform(HmdMatrix34_t pose)
        {
            var m = new Matrix4x4();

            m.m00 = pose.m0;
            m.m01 = pose.m1;
            m.m02 = -pose.m2;
            m.m03 = pose.m3;

            m.m10 = pose.m4;
            m.m11 = pose.m5;
            m.m12 = -pose.m6;
            m.m13 = pose.m7;

            m.m20 = -pose.m8;
            m.m21 = -pose.m9;
            m.m22 = pose.m10;
            m.m23 = -pose.m11;

            this.pos = Matrix4x4.GetPosition(m);
            this.rot = Matrix4x4.GetRotation(m);
        }

        //public HmdMatrix44_t ToHmdMatrix44()
        //{
        //    var m = Matrix4x4.TRS(pos, rot, Vector3.one);
        //    var pose = new HmdMatrix44_t();

        //    pose.m0 = m[0, 0];
        //    pose.m1 = m[0, 1];
        //    pose.m2 = -m[0, 2];
        //    pose.m3 = m[0, 3];

        //    pose.m4 = m[1, 0];
        //    pose.m5 = m[1, 1];
        //    pose.m6 = -m[1, 2];
        //    pose.m7 = m[1, 3];

        //    pose.m8 = -m[2, 0];
        //    pose.m9 = -m[2, 1];
        //    pose.m10 = m[2, 2];
        //    pose.m11 = -m[2, 3];

        //    pose.m12 = m[3, 0];
        //    pose.m13 = m[3, 1];
        //    pose.m14 = -m[3, 2];
        //    pose.m15 = m[3, 3];

        //    return pose;
        //}

        //public HmdMatrix34_t ToHmdMatrix34()
        //{
        //    var m = Matrix4x4.TRS(pos, rot, Vector3.one);
        //    var pose = new HmdMatrix34_t();

        //    pose.m0 = m[0, 0];
        //    pose.m1 = m[0, 1];
        //    pose.m2 = -m[0, 2];
        //    pose.m3 = m[0, 3];

        //    pose.m4 = m[1, 0];
        //    pose.m5 = m[1, 1];
        //    pose.m6 = -m[1, 2];
        //    pose.m7 = m[1, 3];

        //    pose.m8 = -m[2, 0];
        //    pose.m9 = -m[2, 1];
        //    pose.m10 = m[2, 2];
        //    pose.m11 = -m[2, 3];

        //    return pose;
        //}

        //public override bool Equals(object o)
        //{
        //    if (o is RigidTransform)
        //    {
        //        RigidTransform t = (RigidTransform)o;
        //        return pos == t.pos && rot == t.rot;
        //    }
        //    return false;
        //}

        //public override int GetHashCode()
        //{
        //    return pos.GetHashCode() ^ rot.GetHashCode();
        //}

        //public static bool operator ==(RigidTransform a, RigidTransform b)
        //{
        //    return a.pos == b.pos && a.rot == b.rot;
        //}

        //public static bool operator !=(RigidTransform a, RigidTransform b)
        //{
        //    return a.pos != b.pos || a.rot != b.rot;
        //}

        //public static RigidTransform operator *(RigidTransform a, RigidTransform b)
        //{
        //    return new RigidTransform
        //    {
        //        rot = a.rot * b.rot,
        //        pos = a.pos + a.rot * b.pos
        //    };
        //}

        //public void Inverse()
        //{
        //    rot = Quaternion.Inverse(rot);
        //    pos = -(rot * pos);
        //}

        //public RigidTransform GetInverse()
        //{
        //    var t = new RigidTransform(pos, rot);
        //    t.Inverse();
        //    return t;
        //}

        //public void Multiply(RigidTransform a, RigidTransform b)
        //{
        //    rot = a.rot * b.rot;
        //    pos = a.pos + a.rot * b.pos;
        //}

        //public Vector3 InverseTransformPoint(Vector3 point)
        //{
        //    return Quaternion.Inverse(rot) * (point - pos);
        //}

        //public Vector3 TransformPoint(Vector3 point)
        //{
        //    return pos + (rot * point);
        //}

        //public static Vector3 operator *(RigidTransform t, Vector3 v)
        //{
        //    return t.TransformPoint(v);
        //}

        //public static RigidTransform Interpolate(RigidTransform a, RigidTransform b, float t)
        //{
        //    return new RigidTransform(Vector3.Lerp(a.pos, b.pos, t), Quaternion.Slerp(a.rot, b.rot, t));
        //}

        //public void Interpolate(RigidTransform to, float t)
        //{
        //    pos = SteamVR_Utils.Lerp(pos, to.pos, t);
        //    rot = SteamVR_Utils.Slerp(rot, to.rot, t);
        //}
    }
}
