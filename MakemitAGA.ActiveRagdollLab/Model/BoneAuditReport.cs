using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Model
{
    internal sealed class BoneAuditReport
    {
        public bool Completed;
        public bool Success;
        public string Error;
        public string SceneName;
        public string TargetName;
        public string TargetPath;
        public int ExactMatches;
        public int TransformCount;
        public int AnimatorCount;
        public int RigidbodyCount;
        public int JointCount;
        public int ColliderCount;
        public int DuplicateBoneNameCount;
        public string PelvisPath;
        public string SpinePath;
        public string ChestPath;
        public Transform TargetRoot;
        public Transform Pelvis;
        public Transform Spine;
        public Transform Chest;
        public float PelvisToChestDistance;
        public float PelvisToSpineDistance;
        public Vector3 CharacterUp;
        public Vector3 CharacterForward;
        public readonly List<string> Lines = new List<string>();

        public string GetConsoleSummary()
        {
            if (!Completed)
                return "ARL bone audit | completed=False";
            if (!Success)
                return "ARL bone audit | completed=True | success=False | error=" + (Error ?? "unknown");

            return
                "ARL bone audit | completed=True | success=True" +
                " | scene=" + SceneName +
                " | target=" + TargetName +
                " | matches=" + ExactMatches +
                " | transforms=" + TransformCount +
                " | animator=" + AnimatorCount +
                " | rigidbody=" + RigidbodyCount +
                " | joints=" + JointCount +
                " | colliders=" + ColliderCount +
                " | pelvis=" + PelvisPath +
                " | spine=" + SpinePath +
                " | chest=" + ChestPath +
                " | torsoLength=" + F(PelvisToChestDistance) +
                " | duplicateNames=" + DuplicateBoneNameCount;
        }

        public string GetFullText()
        {
            StringBuilder sb = new StringBuilder(4096);
            sb.AppendLine("MakemitAGA Active Ragdoll Lab 0.4.2.1 - Bone Audit");
            sb.AppendLine("completed=" + Completed);
            sb.AppendLine("success=" + Success);
            sb.AppendLine("error=" + (Error ?? "<none>"));
            sb.AppendLine("scene=" + (SceneName ?? "<none>"));
            sb.AppendLine("target=" + (TargetName ?? "<none>"));
            sb.AppendLine("targetPath=" + (TargetPath ?? "<none>"));
            sb.AppendLine("exactMatches=" + ExactMatches);
            sb.AppendLine("transformCount=" + TransformCount);
            sb.AppendLine("animatorCount=" + AnimatorCount);
            sb.AppendLine("rigidbodyCount=" + RigidbodyCount);
            sb.AppendLine("jointCount=" + JointCount);
            sb.AppendLine("colliderCount=" + ColliderCount);
            sb.AppendLine("duplicateBoneNameCount=" + DuplicateBoneNameCount);
            sb.AppendLine("pelvisPath=" + (PelvisPath ?? "<none>"));
            sb.AppendLine("spinePath=" + (SpinePath ?? "<none>"));
            sb.AppendLine("chestPath=" + (ChestPath ?? "<none>"));
            sb.AppendLine("pelvisToChestDistance=" + F(PelvisToChestDistance));
            sb.AppendLine("pelvisToSpineDistance=" + F(PelvisToSpineDistance));
            sb.AppendLine("characterUp=" + V(CharacterUp));
            sb.AppendLine("characterForward=" + V(CharacterForward));
            sb.AppendLine();
            sb.AppendLine("Hierarchy:");
            for (int i = 0; i < Lines.Count; i++)
                sb.AppendLine(Lines[i]);
            return sb.ToString();
        }

        private static string F(float value) => value.ToString("G9", CultureInfo.InvariantCulture);
        private static string V(Vector3 value) => "(" + F(value.x) + ", " + F(value.y) + ", " + F(value.z) + ")";
    }
}
