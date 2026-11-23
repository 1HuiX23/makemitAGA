using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class BilateralFootTransitionTrace
    {
        private sealed partial class Window
        {
            internal readonly Curve RenderAnkle = new Curve(), RenderSole = new Curve();
            internal int RenderQuiet, RenderCount;
            internal string RenderEnd;
            private int _nextRender;
            private bool _renderSlowed;
            private Vector3 _renderOrigin;

            // Consume the real LateUpdate packets through this accepted physics boundary.
            // At higher render rates this includes packets never held by any FixedUpdate.
            internal void ReadRenders(FootTransitionSource history, in FootTransitionSource.Sample end, int through)
            {
                if (RenderEnd != null || through < _nextRender) return;
                if (_nextRender <= 0 || end.Clock.Sequence-_nextRender >= FootTransitionSource.RenderCapacity)
                { RenderEnd="HistoryGap"; return; }
                for (; _nextRender <= through; _nextRender++)
                {
                    if (!history.TryRender(end,_nextRender,out var sample))
                    { RenderEnd="MissingRender"; return; }
                    var visual=sample.Visual;
                    if (RenderCount==0)
                    {
                        bool hasBaseline = history.TryRender(end, _nextRender - 1, out var prev);
                        RenderAnkle.BaselineKnown = RenderSole.BaselineKnown = hasBaseline;
                        var baseline = hasBaseline ? prev.Visual : visual;
                        RenderAnkle.Baseline=Along(baseline.Foot-baseline.Target);
                        RenderSole.Baseline=Along(baseline.Sole-baseline.TargetSole);
                        _renderOrigin=baseline.Target;
                    }
                    float speed=FootTransitionSource.Planar(sample.World).magnitude;
                    float resume=RenderQuiet!=0 ? FootGaitEvidence.StanceTangent : FootGaitEvidence.SwingTangent;
                    if (_renderSlowed && speed>=resume) { RenderEnd="MotionResumed"; return; }
                    RenderCount++;
                    _renderSlowed |= speed<=FootGaitEvidence.StanceTangent;
                    bool firstQuiet=RenderQuiet==0 && speed<=FootContactGeometryLimits.RecoveryMaxPlanarSpeed;
                    if (firstQuiet) RenderQuiet=_nextRender;
                    if (RenderQuiet==0) continue;
                    RenderAnkle.Update(_nextRender,Along(visual.Foot-_renderOrigin),Along(visual.Target-_renderOrigin),firstQuiet);
                    RenderSole.Update(_nextRender,Along(visual.Sole-_renderOrigin),Along(visual.TargetSole-_renderOrigin),firstQuiet);
                }
            }
        }
    }
}
