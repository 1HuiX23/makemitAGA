using System;
using System.Text;
using UnityEngine;
using Phase = MakemitAGA.ActiveRagdollLab.Physics.FootLandingOpportunityAudit.Phase;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class BilateralFootTransitionTrace
    {
        internal enum Reference : byte { Unavailable, Entry, SelectedPlane }

        internal struct SelectedPose
        {
            internal Vector3 Center, Half;
            internal Quaternion Source, Control;
            internal float PlaneTop;
        }

        // Ankle-relative box geometry is shared by all three poses. Projecting onto a
        // verified selected support does not qualify the five probes or enable a collider.
        internal static bool TryRightMinima(Vector3 sourceAnkle, Quaternion sourceRotation, Vector3 lowerAnkle,
            Quaternion controlRotation, Vector3 center, Vector3 half, float top, out float raw, out float goal)
        {
            raw = goal = float.NaN;
            if (!HipPointPairMobility.V(sourceAnkle) || !HipPointPairMobility.V(lowerAnkle) ||
                !RightFootTorqueForecast.ValidRotation(sourceRotation) || !RightFootTorqueForecast.ValidRotation(controlRotation) ||
                !HipPointPairMobility.V(center) || !HipPointPairMobility.V(half) ||
                half.x <= 0f || half.y <= 0f || half.z <= 0f || !float.IsFinite(top)) return false;
            raw = FootBoxGeometry.Minimum(sourceAnkle, sourceRotation, center, half) - top;
            goal = FootBoxGeometry.Minimum(lowerAnkle, controlRotation, center, half) - top;
            return float.IsFinite(raw) && float.IsFinite(goal);
        }

        private sealed class RightReentry
        {
            // Always retain the opening, actual first decision and resolution even for a long wait.
            // Per-step values and counters stop at the first native decision or gait departure. The frozen landing
            // owner can resolve a pending miss later, on a different gait or a different support policy.
            private const int RowLimit = 32;
            private struct Row
            {
                internal int Local, Shared, Source, Held, PeerLocal, Hits, ProbeId;
                internal float Time, Age, Raw, Goal, Actual, Gap, GapY, LowerMinusSourceY, Tangent, Normal, Clearance;
                internal Reference Ref;
                internal Phase Gait, PeerGait;
                internal bool Fresh, Strong, Identity, Same, Inside, Form, Pre, PeerForm, PeerPre, PriorHit, PeerPriorHit;
                internal Row(in View v)
                {
                    var t = v.Foot;
                    Local = t.Local; Shared = t.Step.Sequence; Source = t.Source.Clock.Sequence; Time = t.Step.FixedTime;
                    Fresh = t.Fresh; Held = t.Held; Age = t.Age;
                    Ref = t.RightReference;
                    Raw = t.SourceMin; Goal = t.ControlMin;
                    if (Ref == Reference.SelectedPlane)
                    {
                        var pose = t.RightPose;
                        if (!TryRightMinima(t.Target, pose.Source, t.Lower, pose.Control, pose.Center, pose.Half,
                            pose.PlaneTop, out Raw, out Goal)) Ref = Reference.Unavailable;
                    }
                    else if (!float.IsFinite(Raw) || !float.IsFinite(Goal) || !float.IsFinite(t.RightEntryMin))
                        Ref = Reference.Unavailable;
                    Actual = Ref == Reference.Entry ? t.RightEntryMin : t.BoxMin;
                    var gap = t.Foot - t.Lower;
                    Gap = gap.magnitude; GapY = gap.y; LowerMinusSourceY = t.Lower.y - t.Target.y;
                    Gait = t.Phase; Strong = t.Gait.Stance; Identity = t.QualifiedIdentity;
                    Tangent = t.Gait.Tangent; Normal = t.Gait.SignedNormal; Clearance = t.Gait.Clearance;
                    Hits = t.Gait.Hits; ProbeId = t.Gait.Surface; Same = t.Gait.Same; Inside = t.Inside;
                    Form = t.Active; Pre = t.Precontact; PriorHit = t.PreviousHit;
                    PeerLocal = v.OtherLocal; PeerGait = v.OtherPhase; PeerForm = v.OtherActive;
                    PeerPre = v.OtherPrecontact; PeerPriorHit = v.OtherPreviousHit;
                }
            }
            private struct Depth
            {
                internal int Local;
                internal string Evidence;
                private bool _observed;
                private int _last;
                    internal void See(int local, bool valid, float value)
                {
                    if (!valid) return;
                    bool deep = OrdinaryFootEntry.TooDeep(value);
                    if (Local == 0 && deep)
                    {
                        Local = local;
                        Evidence = !_observed ? "AlreadyBelowAtFirstRead" :
                            _last + 1 != local ? "BelowAfterGap" : "Crossed";
                    }
                    _last = local; _observed = true;
                }
                internal string Text => Local != 0 ? Local + "|" + Evidence : "0|" + (_observed ? "NoBelowObserved" : "Unavailable");
            }
            private readonly Row _before;
            private readonly Row[] _rows = new Row[RowLimit];
            private Row _last;
            private Depth _sourceDepth, _controlDepth, _actualDepth;
            private int _seen, _missingRefs, _peerFormal, _peerPre, _peerPriorHit, _firstStrong, _firstOffStance;

            internal RightReentry(in View before)
            {
                _before = new Row(before);
                ObserveDepth(_before);
            }
            private void ObserveDepth(in Row r)
            {
                _sourceDepth.See(r.Local, r.Ref != Reference.Unavailable, r.Raw);
                _controlDepth.See(r.Local, r.Ref != Reference.Unavailable, r.Goal);
                _actualDepth.See(r.Local, true, r.Actual);
            }
            internal void Observe(in View point, bool throughFirst)
            {
                if (!throughFirst || _firstOffStance != 0) return;
                var r = new Row(point);
                if (_seen < RowLimit) _rows[_seen] = r;
                _last = r;
                _seen++;
                if (r.Ref == Reference.Unavailable) _missingRefs++;
                if (r.PeerForm) _peerFormal++;
                if (r.PeerPre) _peerPre++;
                if (r.PeerPriorHit) _peerPriorHit++;
                if (_firstStrong == 0 && r.Gait == Phase.Stance && r.Strong) _firstStrong = r.Local;
                if (_firstOffStance == 0 && r.Gait != Phase.Stance) _firstOffStance = r.Local;
                ObserveDepth(r);
            }

            internal void Append(StringBuilder b, Window w)
            {
                var t = w.Origin.Foot;
                bool observedFirst = w.First.Foot.Local != 0;
                b.Append("\nRIGHT_STANCE_REENTRY_V041830/epoch-shared-origin-source-floor-body=").Append(t.Step.Epoch).Append('|')
                    .Append(t.Step.Sequence).Append('|').Append(t.Local).Append('|').Append(t.Source.Clock.Sequence)
                    .Append('|').Append(t.Floor).Append('|').Append(t.Body).Append("/release=").Append(w.Type)
                    .Append("/motion=").Append(t.Motion ?? "NA").Append("/first-outcome-resolved=").Append(w.First.Foot.Local)
                    .Append('|').Append(w.Landing).Append('|').Append(w.Resolved.Foot.Local)
                    .Append("/firstStrong-firstOffStance-depthLatch=").Append(_firstStrong).Append('|').Append(_firstOffStance)
                    .Append('|').Append(w.First.Foot.NativeDepthLatch)
                    .Append("/cohort=").Append(_firstOffStance != 0 ? "LeftStanceBeforeFirst" : observedFirst ? "StanceThroughFirst" : "Censored")
                    .Append("/end=").Append(w.EndReason ?? "Open")
                    .Append("/firstBelowRaw=").Append(_sourceDepth.Text).Append("/firstBelowGoal=").Append(_controlDepth.Text)
                    .Append("/firstBelowActual=").Append(_actualDepth.Text)
                    .Append("/observedThrough=").Append(_firstOffStance != 0 ? "FirstOffStance" : observedFirst ? "FirstDecision" : "WindowEnd")
                    .Append("/observedSteps-peerFormal-peerPre-peerPreviousHit-missingRef=").Append(_seen)
                    .Append('|').Append(_peerFormal).Append('|').Append(_peerPre).Append('|').Append(_peerPriorHit).Append('|').Append(_missingRefs)
                    .Append("/middleRowsOmitted=").Append(_firstOffStance != 0 ? Math.Max(0, _seen-2) : Math.Max(0, _seen - RowLimit - 1));
                Write(b, _before, t.Local, t.Step.FixedTime, "Before");
                if (_firstOffStance != 0)
                {
                    Write(b, _rows[0], t.Local, t.Step.FixedTime, "Release");
                    Write(b, _last, t.Local, t.Step.FixedTime, "FirstOffStance");
                    return;
                }
                int kept = Math.Min(_seen, RowLimit);
                for (int i = 0; i < kept; i++) Write(b, _rows[i], t.Local, t.Step.FixedTime, i == 0 ? "Release" : "Step");
                if (_seen > RowLimit) Write(b, _last, t.Local, t.Step.FixedTime, "LastThroughFirst");
                if (w.Resolved.Foot.Local != 0 && w.Resolved.Foot.Local != _last.Local)
                    Write(b, new Row(w.Resolved), t.Local, t.Step.FixedTime, "Resolution");
            }

            private static void Write(StringBuilder b, in Row r, int origin, float startTime, string at)
            {
                bool refs = r.Ref != Reference.Unavailable;
                b.Append("\nR183/origin-local-shared-source=").Append(origin).Append('|').Append(r.Local).Append('|').Append(r.Shared)
                    .Append('|').Append(r.Source).Append("/at=").Append(at).Append("/fromReleaseMs=").Append(F((r.Time-startTime)*1000f))
                    .Append("/gait-strong-id=").Append(r.Gait).Append('|').Append(Bit(r.Strong)).Append('|').Append(Bit(r.Identity))
                    .Append("/fresh-held-ageMs=").Append(Bit(r.Fresh)).Append('|').Append(r.Held).Append('|').Append(F(r.Age*1000f))
                    .Append("/ref=").Append(r.Ref).Append("/raw-goal-actualMm=").Append(refs ? F(r.Raw*1000f) : "NA").Append('|')
                    .Append(refs ? F(r.Goal*1000f) : "NA").Append('|').Append(F(r.Actual*1000f))
                    .Append("/goalMinusRaw-actualMinusGoalMm=").Append(refs ? F((r.Goal-r.Raw)*1000f) : "NA").Append('|')
                    .Append(refs ? F((r.Actual-r.Goal)*1000f) : "NA")
                    .Append("/sourceToGoalAnkleY-rotationMm=").Append(F(r.LowerMinusSourceY*1000f)).Append('|')
                    .Append(refs ? F((r.Goal-r.Raw-r.LowerMinusSourceY)*1000f) : "NA")
                    .Append("/goalToActualAnkleY-rotationMm=").Append(F(r.GapY*1000f)).Append('|')
                    .Append(refs ? F((r.Actual-r.Goal-r.GapY)*1000f) : "NA")
                    .Append("/ankleGap3D-verticalMm=").Append(F(r.Gap*1000f)).Append('|').Append(F(r.GapY*1000f))
                    .Append("/vt-vn-maxAbsMm=").Append(F(r.Tangent)).Append('|').Append(F(r.Normal)).Append('|').Append(F(r.Clearance*1000f))
                    .Append("/hits-same-probeId-inside=").Append(r.Hits).Append('|').Append(Bit(r.Same)).Append('|').Append(r.ProbeId).Append('|').Append(Bit(r.Inside))
                    .Append("/formal-pre-previousHit=").Append(Bit(r.Form)).Append('|').Append(Bit(r.Pre)).Append('|').Append(Bit(r.PriorHit))
                    .Append("/peerLocal-gait-formal-pre-previousHit=").Append(r.PeerLocal).Append('|').Append(r.PeerGait).Append('|')
                    .Append(Bit(r.PeerForm)).Append('|').Append(Bit(r.PeerPre)).Append('|').Append(Bit(r.PeerPriorHit));
            }
        }
    }
}
