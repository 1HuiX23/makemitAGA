using System;
using System.Globalization;
using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class LandingPreparationResponse
    {
        // Five constant-size accumulators. ALL and APPROACH overlap; the other
        // three are disjoint command kinds. No extra timeline/window is created.
        private readonly Totals _all = new Totals(), _hipOnly = new Totals(), _kneeOnly = new Totals(),
            _both = new Totals(), _approachTotals = new Totals();
        private bool _hasMismatch;
        private Sample _mismatch;
        private sealed class Totals
        {
            internal int Count, Up, Down, Flat, AboveThenUp, BelowThenDown, GeometryCount, GeometryMissing,
                ReferenceBelowActualAbove, FullFootprints, VelocityCount, AboveUpVelocity, ProbeValid;
            private double _before, _after, _change, _gapChange, _reference, _actual, _closurePeak;
            private readonly double[] _delta = new double[7], _absoluteDelta = new double[7], _probeGain = new double[3];
            private readonly int[] _probeInside = new int[3], _probeUp = new int[3], _probeDown = new int[3], _probeFlat = new int[3];
            internal void Reset()
            {
                Count = Up = Down = Flat = AboveThenUp = BelowThenDown = GeometryCount = GeometryMissing =
                    ReferenceBelowActualAbove = FullFootprints = VelocityCount = AboveUpVelocity = ProbeValid = 0;
                _before = _after = _change = _gapChange = _reference = _actual = _closurePeak = 0;
                Array.Clear(_delta, 0, 7); Array.Clear(_absoluteDelta, 0, 7); Array.Clear(_probeGain, 0, 3);
                Array.Clear(_probeInside, 0, 3); Array.Clear(_probeUp, 0, 3); Array.Clear(_probeDown, 0, 3); Array.Clear(_probeFlat, 0, 3);
            }
            internal void Add(Sample s)
            {
                Count++; float d = s.AfterMin - s.BeforeMin;
                _before += s.BeforeMin; _after += s.AfterMin; _change += d; _gapChange += s.AfterGap - s.BeforeGap;
                if (d > .0001f) Up++; else if (d < -.0001f) Down++; else Flat++;
                if (s.BeforeMin > .0001f && d > .0001f) AboveThenUp++;
                if (s.BeforeMin < -.0001f && d < -.0001f) BelowThenDown++;
                var g = s.Geometry;
                if (!g.Valid) { GeometryMissing++; return; }
                GeometryCount++; _reference += g.Reference; _actual += g.Actual;
                if (g.Reference < -.0001f && g.Actual > .0001f) ReferenceBelowActualAbove++;
                if (g.FootprintMask == 255) FullFootprints++;
                AddDelta(0, g.ParentDelta); AddDelta(1, g.HipRotationDelta); AddDelta(2, g.KneeRotationDelta);
                AddDelta(3, g.HipGapDelta); AddDelta(4, g.KneeGapDelta); AddDelta(5, g.AnkleGapDelta); AddDelta(6, g.FootRotationDelta);
                _closurePeak = Math.Max(_closurePeak, Math.Abs(g.Closure));
                if (g.VelocityValid) { VelocityCount++; if (g.Actual > .0001f && g.CornerNormalSpeed > .006f) AboveUpVelocity++; }
                if (!g.ProbeValid) return;
                ProbeValid++;
                // Out-of-footprint candidates remain visible in the denominator,
                // never counted as useful in-surface displacement.
                AddProbe(0, g.HipProbe - g.Actual, (g.ProbeFootprintMask & 3) == 3);
                AddProbe(1, g.KneeProbe - g.Actual, (g.ProbeFootprintMask & 5) == 5);
                AddProbe(2, g.BothProbe - g.Actual, (g.ProbeFootprintMask & 9) == 9);
            }
            private void AddDelta(int i, float d) { _delta[i] += d; _absoluteDelta[i] += Math.Abs(d); }
            private void AddProbe(int i, float gain, bool inside)
            {
                if (!inside) return;
                _probeInside[i]++; _probeGain[i] += gain;
                if (gain > .0001f) _probeUp[i]++; else if (gain < -.0001f) _probeDown[i]++; else _probeFlat[i]++;
            }
            internal string Text(string name) => "\nLANDING_RESPONSE_TOTALS_V041410/group=" + name + "/resolved=" + Count +
                "/actualUp-down-flat=" + Up + "|" + Down + "|" + Flat + "/aboveThenUp-belowThenDown=" + AboveThenUp + "|" + BelowThenDown +
                "/meanMm(before-after-change-gapChange)=" + Mean(_before, Count) + "|" + Mean(_after, Count) + "|" + Mean(_change, Count) + "|" + Mean(_gapChange, Count) +
                "/geometry(valid-missing-allFootprintsInside-refBelowActualAbove)=" + GeometryCount + "|" + GeometryMissing + "|" + FullFootprints + "|" + ReferenceBelowActualAbove +
                "/geometryMeanMm(ref-actual)=" + Mean(_reference, GeometryCount) + "|" + Mean(_actual, GeometryCount) +
                "/deltaMeanMm(parent-hipR-kneeR-hipGap-kneeGap-ankleGap-footR)=" + Values(_delta, GeometryCount) +
                "/deltaAbsMeanMm(sameOrder)=" + Values(_absoluteDelta, GeometryCount) + "/closurePeakMm=" + (GeometryCount > 0 ? Number(_closurePeak*1000) : "NA") +
                "/preCornerVelocity(valid-aboveUp)=" + VelocityCount + "|" + AboveUpVelocity +
                "/probeValid=" + ProbeValid + ProbeText(0, "hip") + ProbeText(1, "knee") + ProbeText(2, "both");
            private string ProbeText(int i, string name) => "/probe_" + name + "(inside-up-down-flat-meanMm)=" +
                _probeInside[i] + "|" + _probeUp[i] + "|" + _probeDown[i] + "|" + _probeFlat[i] + "|" + Mean(_probeGain[i], _probeInside[i]);
            private static string Values(double[] a, int n) => Mean(a[0],n) + "|" + Mean(a[1],n) + "|" + Mean(a[2],n) + "|" + Mean(a[3],n) + "|" + Mean(a[4],n) + "|" + Mean(a[5],n) + "|" + Mean(a[6],n);
            private static string Mean(double v, int n) => n > 0 ? Number(v * 1000 / n) : "NA";
            private static string Number(double v) => v.ToString("G9", CultureInfo.InvariantCulture);
        }
        private void ResetGeometry()
        {
            _all.Reset(); _hipOnly.Reset(); _kneeOnly.Reset(); _both.Reset(); _approachTotals.Reset();
            _hasMismatch = false; _mismatch = default;
        }
        private void ObserveGeometry(LandingPreparationCommand c, Sample s)
        {
            _all.Add(s);
            if (c.HipApplied && c.KneeApplied) _both.Add(s); else if (c.HipApplied) _hipOnly.Add(s); else _kneeOnly.Add(s);
            if (c.Approach) _approachTotals.Add(s);
            if (s.Geometry.Valid && (!_hasMismatch || Mathf.Abs(s.Geometry.Actual-s.Geometry.Reference) >
                Mathf.Abs(_mismatch.Geometry.Actual-_mismatch.Geometry.Reference))) { _hasMismatch = true; _mismatch = s; }
        }
        private string GeometrySummary() => "\nLANDING_RESPONSE_SCOPE_V041410=ALL_RESOLVED_ADOPTED_COMMANDS" +
            "/groups=ALL_PLUS_DISJOINT_HIP_ONLY_KNEE_ONLY_BOTH_PLUS_APPROACH_SUBSET" +
            "/heightDeadbandMm=0.1/preCornerSpeedDeadbandMps=0.006/upIsNotLandingSuccess=True/causalIsolation=False/diagnosticFeedback=False" +
            _all.Text("ALL") + _hipOnly.Text("HIP_ONLY") + _kneeOnly.Text("KNEE_ONLY") + _both.Text("BOTH") + _approachTotals.Text("APPROACH") +
            (_hasMismatch ? Text("MAX_REFERENCE_ACTUAL_GAP", _mismatch) : "\nLANDING_GEOMETRY_WITNESS_V041410=NONE");
    }
}
