using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Physics;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;

namespace ARL.Contracts
{
    internal static class HipJoinContracts
    {
        internal static void Run()
        {
            Check.Run("stable join waits for both geometry and genuine source time", Stable);
            Check.Run("held source and persistent penetration never qualify", Reject);
            Check.Run("join support identity, clock, reset and cancellation", Identity);
            Check.Run("R1 shallow timeout endpoints qualify only for unloaded join", Shallow);
        }
        private static HipJoinPose Pose(float min = -.0013f) => new HipJoinPose
        {
            Geometry = new InitialGeometryPreflight { Valid = true, Calibrated = true, FootprintsInside = true,
                RawMin = -.0027f, ControlMin = min, ActualMin = min },
            Hip = new Vector3(0,1,0), Knee = new Vector3(0,.5f,0), Ankle = new Vector3(0,.1f,0),
            RawHip = new Vector3(0,1,0), RawFoot = new Vector3(0,.1f,0),
            Upper = Quaternion.identity, Lower = Quaternion.identity, Foot = Quaternion.identity,
            RawUpper = Quaternion.identity, RawLower = Quaternion.identity, RawFootRotation = Quaternion.identity
        };
        private static PhysicsStepContext Step(int n, int source)
        { var s = RuntimeContracts.Step(n); s.SourceSequence = source; return s; }
        private static void Feed(HipJoinPreparationWindow w, int n, HipJoinPose p, int hz = 60, int id = 10)
        {
            int seq = (n - 1) * hz / 60 + 1;
            w.Observe(Step(n,seq), new SourceStepContext { Epoch = 1, Sequence = seq, Time = (seq-1)/(float)hz, Dt = 1f/hz },
                id, new Vector3(0,-.15f,0), new Vector3(4,.15f,4), p, p, null);
        }
        private static void Stable()
        {
            foreach (int hz in new[] {20,40,60})
            {
                var w = new HipJoinPreparationWindow(); w.Begin(Step(1,1));
                for (int n=1;n<=45;n++) Feed(w,n,Pose(-.004279f),hz);
                Check.That(w.Pending && !w.Qualified,"logged B-depth fixture cannot pass immediately");
                for (int n=46;n<=100;n++) Feed(w,n,Pose(),hz);
                Check.That(w.Pending && !w.Qualified,"less than one full stable second cannot pass");
                int k=101; while(w.Pending && k<130) { Feed(w,k,Pose(),hz); k++; }
                Check.That(w.Qualified && w.StableSeconds>=1f,"all source cadences qualify after the stable window");
                Check.That(!w.QualifiedFor(Step(k,k)),"qualification cannot authorize a later physics step");
            }
        }
        private static void Reject()
        {
            var w = new HipJoinPreparationWindow(); w.Begin(Step(1,1));
            for(int n=1;n<1000 && w.Pending;n++) Feed(w,n,Pose(-.00301f));
            Check.That(!w.Qualified && w.Reason.StartsWith("TIMEOUT:"),"persistent depth outside the preparation envelope times out");
            w.Begin(Step(1,1));
            var source = new SourceStepContext {Epoch=1,Sequence=1,Time=0,Dt=1f/60f};
            for(int n=1;n<1000 && w.Pending;n++)
                w.Observe(Step(n,1),source,10,new Vector3(0,-.15f,0),new Vector3(4,.15f,4),Pose(),Pose(),null);
            Check.That(!w.Qualified,"Fixed catchup cannot promote one held source to a stable window");
        }
        private static void Shallow()
        {
            foreach(float min in new[] {-.002687f,-.002594f,-.002647f,-.002729f})
            {
                var p=Pose(min); var w=new HipJoinPreparationWindow(); w.Begin(Step(1,1));
                for(int n=1;n<80 && w.Pending;n++) Feed(w,n,p);
                Check.That(w.Qualified && p.Geometry.Issue!=null && p.Geometry.PreparationIssue==null,
                    "logged endpoint passes stable preparation while ordinary geometry remains rejected");
            }
        }
        private static void Identity()
        {
            var w=new HipJoinPreparationWindow(); w.Begin(Step(1,1)); Feed(w,1,Pose()); Feed(w,2,Pose(),60,11);
            Check.That(!w.Pending && !w.Qualified && w.Reason.Contains("SUPPORT_CHANGED"),"floor switch invalidates the fixture");
            w.Begin(Step(1,1)); Feed(w,1,Pose()); Feed(w,3,Pose());
            Check.That(!w.Pending && !w.Qualified,"skipped shared Fixed packet rejected");
            w.Begin(Step(1,1)); Feed(w,1,Pose()); w.Cancel("USER_STOP");
            Check.That(!w.Pending && !w.Qualified,"user cancellation leaves no live readiness");
            w.Reset(); Check.That(w.Reason=="NOT_REQUESTED" && w.Elapsed==0f,"restart does not inherit a hold or timeout");
        }
    }
}
