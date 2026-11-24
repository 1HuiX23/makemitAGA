using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using Window = MakemitAGA.ActiveRagdollLab.Physics.RightFootPrecontact;

namespace ARL.Contracts
{
    internal static partial class RightFootPrecontactContracts
    {
        private static void RunEntryAuditContracts()
        {
            RunPhysicalEntryContracts();
            Check.Run("Entry audit preserves actual window, reopening lock and cleared formal link", EntryWindows);
            Check.Run("Entry audit isolates floor, trace and formal-active boundaries", EntryBoundaries);
            Check.Run("Entry audit joins ordinary, recovery and unrecovered receipts to the original landing", EntryOutcomeJoin);
            Check.Run("Entry audit counts through bounded retention without per-solve allocations", EntryCapacity);
            Check.Run("Captured 179/180 right steady approaches include normal control landings", EntryFixture);
        }

        private static Window Audited(Window.Mode mode = Window.Mode.PreviousStepExpiryReview)
        {
            var w = new Window(); w.Select(mode); w.ResetTrace(); return w;
        }

        // Drive the real selector and receipts in lifecycle order; native callbacks are synthetic here.
        private static void EntryStep(Window w, in Window.Sample s, bool ordinary = false, bool recovery = false,
            bool miss = false, int depth = 0, bool held = false, bool issueMiss = false, bool ok = true,
            Window.End close = Window.End.None, bool trace = true, bool previousMissCounted = false)
        {
            w.BeginEntryAudit(s.Local, trace, previousMissCounted);
            var end = w.Evaluate(s, issueMiss, out bool request);
            if (ok && (ordinary || recovery)) { w.BeginFormal(recovery); w.Close(Window.End.Handoff, s.Local); }
            else if (ok && (miss || held)) w.Close(Window.End.Miss, s.Local);
            else if (close != Window.End.None || end != Window.End.None) w.Close(close != Window.End.None ? close : end, s.Local);
            else if (ok && request) w.Open();
            if (ok && w.Tracking) { w.CountSolve(); w.CheckUnloaded(s.Local, false); }
            int link = w.LinkedOrigin;
            if (ok && (ordinary || recovery || miss)) w.AfterFormalEvent();
            if (!ok) w.Close(Window.End.Fault, s.Local);
            w.CompleteEntryAudit(ok, ordinary, recovery, miss, depth, held, link, ordinary || recovery);
        }

        private static void EntryWindows()
        {
            var w = Audited();
            EntryStep(w, S(1, .032f)); EntryStep(w, S(2, .021f));
            Check.That(!w.Tracking && w.LinkedOrigin == 0, "35/45 never open a physical window");
            EntryStep(w, S(3, .013f));
            Check.That(w.Physical && w.LinkedOrigin == 3, "native 25 opens on the next pair");
            EntryStep(w, S(4, .010f), close: Window.End.Rising);
            EntryStep(w, S(5, .007f)); EntryStep(w, S(6, .034f)); EntryStep(w, S(7, .020f));
            EntryStep(w, S(8, -.004f), held: true);
            EntryStep(w, S(9, -.005f), miss: true, depth: 8, issueMiss: true, held: true);
            string log = w.Summary();
            Check.That(log.Contains("/depthLocal=8/") &&
                log.Contains("/nativeStarts-first-last=1|3|3/lastCloseOrigin-local-end=3|4|Rising/formalLinkedOrigin=0"),
                "an early close remains attributed even though the formal link has expired");
            Check.That(log.Contains("/nativeRequest-open-commitMismatch=1|1|0/"), "native accounting closes");
        }

        private static void EntryBoundaries()
        {
            var w=Audited(); EntryStep(w,S(1,.020f)); EntryStep(w,S(2,.010f));
            var other=S(3,.009f); other.Support=44; EntryStep(w,other,ordinary:true);
            Check.That(w.Summary().Contains("/start-local=Continuity|3/") &&
                w.Summary().Contains("/nativeStarts-first-last=0|0|0/lastCloseOrigin-local-end=0|0|None"),
                "a physical window on the old support cannot own a new-floor landing");
            w.BeginEntryAudit(4, true, false); w.ObserveExistingFormal(); w.BreakContinuity();
            w.CompleteEntryAudit(true,false,false,false,0,false,0,false);
            EntryStep(w,S(5,.033f)); EntryStep(w,S(6,.014f),ordinary:true);
            Check.That(w.Summary().Contains("/fixed-evaluated-formalBypass-fault=6|5|1|0/") &&
                w.Summary().Contains("/start-local=AfterFormal|5/"),"formal contact, including release, is a boundary");
            var prior=Audited(First35); EntryStep(prior,S(1,.038f),miss:true,held:true,depth:1);
            EntryStep(prior,S(2,.014f),issueMiss:true,held:true,previousMissCounted:true);
            EntryStep(prior,S(3,.032f),issueMiss:true,previousMissCounted:true);
            EntryStep(prior,S(4,.014f)); EntryStep(prior,S(5,0f),ordinary:true);
            Check.That(prior.Summary().Contains("/start-local=AfterFormal|3/") &&
                prior.Summary().Contains("/nativeStarts-first-last=1|4|4/"), "old missed hold is excluded");
            var carrying=Audited(); EntryStep(carrying,S(1,.020f)); EntryStep(carrying,S(2,.012f));
            carrying.ResetTrace(); EntryStep(carrying,S(3,.010f),ordinary:true);
            Check.That(carrying.Summary().Contains("/carriedOrigin=2/nativeStarts-first-last=0|0|0/") &&
                carrying.Summary().Contains("/lastCloseOrigin-local-end=2|3|Handoff/formalLinkedOrigin=2"),"live native contact survives trace restart");
            var off=Audited(); EntryStep(off,S(1,.031f),trace:false); EntryStep(off,S(2,.011f),trace:false);
            Check.That(off.Summary().Contains("/fixed-evaluated-formalBypass-fault=0|0|0|0/"),"trace off collects no steps");
            off.Select(Window.Mode.Off); off.ResetTrace(); Check.That(off.Summary()=="","unselected foot has no entry audit");
        }

        private static void EntryOutcomeJoin()
        {
            foreach (string outcome in new[] {"Enabled", "Recovered", "HoldEnded", "TraceEnded"})
            {
                var w = Audited();
                var outcomes = new MakemitAGA.ActiveRagdollLab.Physics.LandingWindowOutcomes();
                bool missed = outcome != "Enabled";
                for (int n = 1; n <= 4; n++)
                {
                    var s = S(n, n == 1 ? .033f : n == 2 ? .013f : -.004f);
                    bool first = n == 4;
                    EntryStep(w, s, ordinary: first && !missed, miss: first && missed,
                        depth: first && missed ? 3 : 0, held: missed && n >= 3);
                    var result = new MakemitAGA.ActiveRagdollLab.Physics.FootLandingOpportunityAudit.Sample {
                        Step = s.Step, Source = s.Source, Fixed = n, SupportId = s.Support, FloorHalf = s.FloorHalf, FootSide = "RIGHT",
                        GeometryValid = true, MotionValid = true, TrackingValid = true, Min = s.Min, SourcePhase = "STEADY",
                        FirstMiss = first && missed, OrdinaryEnabled = first && !missed, NowActive = first && !missed,
                        DepthLatchFixed = first && missed ? 3 : 0, MissInProgress = missed && n >= 3 };
                    outcomes.Observe(result, false, true, false, true);
                }
                if (missed && outcome != "TraceEnded")
                {
                    var s = S(5, 0f); bool recovered = outcome == "Recovered";
                    EntryStep(w, s, recovery: recovered, previousMissCounted: true);
                    var result = new MakemitAGA.ActiveRagdollLab.Physics.FootLandingOpportunityAudit.Sample {
                        Step = s.Step, Source = s.Source, Fixed = 5, SupportId = s.Support, FloorHalf = s.FloorHalf, FootSide = "RIGHT",
                        GeometryValid = true, MotionValid = true, TrackingValid = true, Min = s.Min,
                        NowActive = recovered, RecoveryEnabled = recovered };
                    outcomes.Observe(result, false, true, false, true);
                }
                if (outcome == "TraceEnded")
                    outcomes.Cut(MakemitAGA.ActiveRagdollLab.Physics.LandingEventHistory.MissEnd.TraceEnded);
                string kind = missed ? "FirstMiss" : "Ordinary";
                Check.That(w.Summary().Contains("RIGHT_PRECONTACT_LANDING_V041830/epoch-shared-local-source-support=3|24|4|4|42/kind="+kind)
                    && outcomes.Summary().Contains("LANDING_HISTORY_EVENT_V041440/kind="+kind+"/outcome="+outcome+"/epoch-shared-local-source=3|24|4|4/support=42"),
                    "join by side, source, support and both physics identities: " + outcome);
                Check.That(w.Summary().Contains("/landingTotal-saved-omitted=1|1|0"), "recovery is not a second first landing");
            }
        }

        private static void EntryCapacity()
        {
            var w=Audited();
            for(int n=1;n<=20;n++) EntryStep(w,S(n,(n&1)==1?.032f:.014f));
            long before=GC.GetAllocatedBytesForCurrentThread();
            for(int n=21;n<=900;n++) EntryStep(w,S(n,(n&1)==1?.032f:.014f));
            long bytes=GC.GetAllocatedBytesForCurrentThread()-before;
            Check.That(bytes==0,"no managed allocation per examined pair during repeated solving");
            Check.That(w.Summary().Contains("/fixed-evaluated-formalBypass-fault=900|900|0|0/"),
                "no speculative pairs retained on the +25 mm baseline");
            for(int n=901;n<=1030;n++) EntryStep(w,S(n,0),miss:true,depth:n,held:true);
            Check.That(w.Summary().Contains("/landingTotal-saved-omitted=130|128|2"),"landing retention is independent of pair retention");
        }

        private static float EntryFloat(string value) => value=="NA" ? float.NaN : float.Parse(value,CultureInfo.InvariantCulture)*.001f;
        private static void EntryFixture()
        {
            using var stream=typeof(RightFootPrecontactContracts).Assembly.GetManifestResourceStream("ARL.Contracts.RightPrecontactEntry179180");
            using var reader=new StreamReader(stream);
            var counts=new int[2,2,2]; var affectedMisses=new List<string>();
            int version=0, eventLocal=0, group=0, cutoff=0, first35=0;
            Window.Sample previous=default;
            var wide = new Window(); wide.Select(First35);
            void EndEvent()
            {
                if (version==0) return;
                int vi=version-179; counts[vi,group,0]++;
                if(first35!=0) counts[vi,group,1]++;
                if(group==1 && first35!=0) affectedMisses.Add(version+"/"+eventLocal+"/"+first35);
            }
            string line;
            while((line=reader.ReadLine())!=null)
            {
                if(line.StartsWith("#")) continue;
                var t=line.Split('\t');
                if(t[0]=="E")
                {
                    EndEvent(); version=int.Parse(t[1]); eventLocal=int.Parse(t[2]); group=t[3]=="Enabled"?0:1;
                    cutoff=int.Parse(t[4]); if(t[5]!="0") cutoff=Math.Min(cutoff,int.Parse(t[5].Split(',')[0]));
                    previous=default; first35=0; continue;
                }
                int n=int.Parse(t[1]), src=int.Parse(t[2]), flags=int.Parse(t[4]);
                var s=S(n,EntryFloat(t[5])); s.Predicted=EntryFloat(t[6]); s.CornerV=EntryFloat(t[7]);
                s.Age=EntryFloat(t[8]); s.Gap=EntryFloat(t[9]); s.AnkleY=EntryFloat(t[10]);
                // H144 has the footprint and fallback bits, not both native five-ray receipts
                // or the rearm lock. Identity and monotone timestamps here are explicit assumptions.
                s.ExactFloor=(flags & 8)!=0 && (flags & 32)==0; s.PhaseKnown=t[3] is "Swing" or "Stance" or "Approach";
                s.Step.SourceSequence=s.Source.Sequence=src; s.Source.Time=src*.016f;
                wide.BreakContinuity();
                wide.Evaluate(previous, false, out _); wide.Evaluate(s, false, out bool opens);
                if(n<cutoff && first35==0 && opens && !Window.Candidate(previous,s)) first35=n;
                previous=s;
            }
            EndEvent();
            Check.That(string.Join(",",affectedMisses)=="179/2778/2765,179/2835/2822,180/2795/2783,180/5969/5956",
                "same four geometric miss candidates with the +35 mm first sample");
            for(int v=0;v<2;v++)
            {
                Check.That(counts[v,0,0]==10+v && counts[v,1,0]==7-v,"all 17 steady landings per run");
                Check.That(counts[v,0,1]==(v==0?8:2) &&
                    counts[v,1,1]==2,"normal and missed landings at each alternative");
            }
        }
    }
}
