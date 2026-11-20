using ARL.Contracts;
using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class PelvisDynamicReceiverIsolation
    {
        internal static void RunTradeCompletionContracts() =>
            Check.Run("CAP10 receiver commits split scales once and rejects corrupt receipts before writes", TradeCompletion);

        private static void TradeCompletion()
        {
            HipBoundedTradeContracts.AppliedFixture(out _, out _, out var lc, out var rc);
            foreach (bool trace in new[] { false, true })
            for (int fault = -1; fault < 9; fault++)
            {
                var receiver = new PelvisDynamicReceiverIsolation(); receiver.Produce(lc.Step, trace);
                receiver._hipCommand = lc.ParentInput;
                var l = Pair(lc.ParentInput, lc.Fixed, true); var r = Pair(rc.ParentInput, rc.Fixed, false);
                l.Current = lc; r.Current = rc;
                switch (fault)
                {
                    case 0: r.Current.UpperAngularScale = r.Current.MotionScale; break;
                    case 1: l.Current.UpperAngularScale = r.Current.UpperAngularScale = lc.MotionScale; break;
                    case 2: l.Current.UpperAngularScale = r.Current.UpperAngularScale = lc.MotionScale * .8f; break;
                    case 3: l.Current.ParentAngularReaction = -lc.MotionScale * lc.Angular.RelativeTorque; break;
                    case 4: l.Current.NetHipTorque = lc.MotionScale * lc.PureHipTorque; break;
                    case 5: l.Current.FullFeedbackFeasible = r.Current.FullFeedbackFeasible = false; break;
                    case 6: l.Current.UpperAngularScale = r.Current.UpperAngularScale = float.NaN; break;
                    case 7:
                        receiver._hipCommand.BoundedTradeSelected = false;
                        l.Current.ParentInput = r.Current.ParentInput = receiver._hipCommand;
                        break;
                    case 8: l.Current.UpwardExtraAcceleration = r.Current.UpwardExtraAcceleration = lc.AllocationDecision.SameQExtra; break;
                }
                bool ok = receiver.CompleteHipTranslationReaction(lc.Step, l, r);
                if (fault >= 0)
                {
                    Check.That(!ok && !receiver._hipAttitudeCommand.Committed && receiver._pelvis.ForceCalls == 0 &&
                        receiver._chest.ForceCalls == 0 && receiver._pelvis.TorqueCalls == 0, "corrupt split-scale receipt writes nothing: " + fault);
                    continue;
                }
                var c = receiver._hipAttitudeCommand;
                Check.That(ok && c.Matches(lc.ParentInput) && c.UpperAngularScale == lc.UpperAngularScale &&
                    c.MotionScale == lc.MotionScale && c.UpwardExtraAcceleration == lc.UpwardExtraAcceleration,
                    "completed receipt retains both scales and actual Y");
                Check.That((receiver._pelvis.ForceSum - 4f * (lc.CommonAcceleration - lc.ParentInput.Acceleration)).magnitude < .00001f &&
                    (receiver._chest.ForceSum - 6f * (lc.CommonAcceleration - lc.ParentInput.Acceleration)).magnitude < .00001f,
                    "native delta uses allocated common acceleration exactly once");
                Check.That(receiver._pelvis.TorqueSum.Equals(c.CorrectionTorque) &&
                    c.AngularReaction.Equals(lc.ParentAngularReaction + rc.ParentAngularReaction), "relative reaction included exactly once");
                Check.That(!receiver.CompleteHipTranslationReaction(lc.Step, l, r) && receiver._pelvis.ForceCalls == 1 &&
                    receiver._chest.ForceCalls == 1 && receiver._pelvis.TorqueCalls == 1, "completion cannot submit twice");
            }
        }
    }
}
