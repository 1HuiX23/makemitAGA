using System.Globalization;
using System.Text;
using UnityEngine;
using MakemitAGA.ActiveRagdollLab.Model;
using Command = MakemitAGA.ActiveRagdollLab.Model.HipAnchorPairReport.Command;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class BilateralFootTransitionTrace
    {
        private sealed partial class LeftLandingPath
        {
            private TurnHeightContinuation _heightContinuation;
            private bool _heightSeen;
            internal void CompareTurnHeight(int origin, in Tick l, in Tick r, in ParentHeightProbe.Input input,
                in HipAnchorPairInput left, in HipAnchorPairInput right, in Command lc, in Command rc,
                bool leftFirst, bool pulse)
            {
                _heightSeen = true;
                var row=_rise[_rise.Count-1];
                try
                {
                    row.TurnHeight=_heightContinuation.Compare(origin,l,r,input,row.Input,left,right,lc,rc,leftFirst,pulse,row.Plans.Native);
                    if (row.TurnHeight.State != TurnHeightContinuation.State.None && input.ReceiverReady && input.Step.Matches(row.Step))
                        row.HeightInput = input;
                }
                catch (System.Exception)
                {
                    // Keep the native rise/readback even when the optional alternative is invalid.
                    row.TurnHeight = _heightContinuation.End(TurnHeightContinuation.State.Missing);
                }
            }

            // Preserve the issued reference in round-trip precision, formatting only when stopped.
            private static string HeightNumber(float x) => x.ToString("R", CultureInfo.InvariantCulture);

            private static void HeightVector(StringBuilder b, string name, Vector3 v)
            {
                b.Append('/').Append(name).Append('=').Append(HeightNumber(v.x)).Append('|')
                    .Append(HeightNumber(v.y)).Append('|').Append(HeightNumber(v.z));
            }

            private static void AppendHeightInput(StringBuilder b, int start, int local, in ParentHeightProbe.Input i)
            {
                b.Append("\nH180_INPUT/start-issue=").Append(start).Append('|').Append(local)
                    .Append("/epoch-fixed-source=").Append(i.Step.Epoch).Append('|').Append(i.Step.Sequence).Append('|').Append(i.Step.SourceSequence)
                    .Append("/dt=").Append(HeightNumber(i.Step.Dt)).Append("/feedback=").Append(i.Feedback)
                    .Append("/support-mask-approach=").Append(i.Support.SupportId).Append('|').Append(i.Support.StanceMask).Append('|').Append(i.Support.ApproachMask)
                    .Append("/rawPelvisY-comFloorY=").Append(HeightNumber(i.SampledPelvisY)).Append('|').Append(HeightNumber(i.FloorY))
                    .Append("/positionLimit=").Append(HeightNumber(i.PositionLimit));
                HeightVector(b,"position",i.Position); HeightVector(b,"velocity",i.Velocity);
                HeightVector(b,"goal",i.Goal); HeightVector(b,"goalV",i.GoalV);
                HeightVector(b,"previousRequest",i.PreviousRequest); HeightVector(b,"previousAllocated",i.PreviousAllocated);
                HeightVector(b,"settledV",i.SettledV); HeightVector(b,"committedPosition",i.CommittedPosition);
                HeightVector(b,"committedV",i.CommittedV); HeightVector(b,"actualCom",i.ActualCom);
                HeightVector(b,"positionError",i.PositionError); HeightVector(b,"velocityError",i.VelocityError);
            }

            internal void AppendTurnHeight(StringBuilder b, int start, int strong, int first)
            {
                if (!_heightSeen) return;
                int compared=0, paired=0;
                foreach (var row in _rise)
                {
                    var h=row.TurnHeight;
                    if (h.State==TurnHeightContinuation.State.None) continue;
                    if (row.HeightInput.ReceiverReady) AppendHeightInput(b, start, row.From, row.HeightInput);
                    b.Append("\nH180/start-issue-read=").Append(start).Append('|').Append(row.From).Append('|').Append(row.To)
                        .Append("/state=").Append(h.State).Append("/read=").Append(row.State);
                    if (h.State==TurnHeightContinuation.State.Seed || h.HasPlan)
                        b.Append("/floorY_m=").Append(F(h.Floor));
                    if (!h.HasPlan) continue;
                    b.Append("/deltaRefMm(instantY-chainY-chainXZ)=").Append(F(h.InstantY*1000f)).Append('|')
                        .Append(F(h.ReferenceY*1000f)).Append('|').Append(F(h.ReferenceXZ*1000f))
                        .Append("/deltaRefVy_mps=").Append(F(h.VelocityY));
                    if (h.State!=TurnHeightContinuation.State.Compared) continue;
                    compared++;
                    b.Append("/requestY(base-trial)=").Append(F(h.Native.Request)).Append('|').Append(F(h.Trial.Request))
                        .Append("/allocatedY(base-trial)=").Append(F(h.Native.System.y)).Append('|').Append(F(h.Trial.System.y))
                        .Append("/rightForceDeltaN=").Append(F((h.Trial.RightForce-h.Native.RightForce).magnitude))
                        .Append("/hipVy_mps(base-trial)=").Append(F(row.Predicted(h.Native))).Append('|').Append(F(row.Predicted(h.Trial)));
                    HeightVector(b,"deltaSystemAcceleration",h.Trial.System-h.Native.System);
                    HeightVector(b,"deltaRightForce",h.Trial.RightForce-h.Native.RightForce);
                    if (row.State!=Read.Matched) continue;
                    paired++;
                    b.Append("/actualHipVy_mps=").Append(F(row.PostHipV));
                    int index=row.To-start;
                    if (index>=0 && index<_points.Count && _points[index].Local==row.To && _points[index].Source==row.ReadClock.SourceSequence)
                        b.Append("/actualSole-gapMm=").Append(F(_points[index].Min*1000f)).Append('|').Append(F(_points[index].Gap*1000f));
                }
                b.Append("\nTURN_HEIGHT_CONTINUATION_V041800/start-strong-first=").Append(start).Append('|').Append(strong).Append('|').Append(first)
                    .Append("/compared-nextMatched=").Append(compared).Append('|').Append(paired)
                    .Append("/model=CONDITIONAL_REFERENCE_AND_ONE_SOLVE/plantWrites=0");
            }
        }
    }
}
