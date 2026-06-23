/*
 * SeatVlmDebugVisuals.cs
 * 只负责黄色搜索射线、青色候选框和最终命中标记。
 * 正式项目默认隐藏；debug_svt 才允许显示。
 */
using System.Collections.Generic;
using UnityEngine;

using MakemitAGA.World;
namespace MakemitAGA.Mita_self.Mita_tools
{
    internal static class SeatVlmDebugVisuals
    {
        private const string Prefix = "VT_Debug_";
        private const int DebugLayer = 2;

        private static readonly List<GameObject> RegionObjects = new List<GameObject>();
        private static readonly List<GameObject> SelectionObjects = new List<GameObject>();
        private static readonly List<GameObject> HitObjects = new List<GameObject>();

        private static Material _yellow;
        private static Material _cyan;
        private static Material _red;
        private static Material _green;

        public static bool Visible { get; private set; } = false;

        public static bool IsDebugTransform(Transform t)
        {
            while (t != null)
            {
                if ((t.name ?? "").StartsWith(Prefix)) return true;
                t = t.parent;
            }
            return false;
        }

        public static void DrawSearchFrustum(Camera cam, Rect rect)
        {
            ClearList(RegionObjects);
            if (cam == null) return;
            EnsureMaterials();

            float d = 3f;
            Vector3 o = cam.transform.position;
            Vector3 bl = cam.ViewportPointToRay(new Vector3(rect.xMin, rect.yMin, 0)).GetPoint(d);
            Vector3 br = cam.ViewportPointToRay(new Vector3(rect.xMax, rect.yMin, 0)).GetPoint(d);
            Vector3 tl = cam.ViewportPointToRay(new Vector3(rect.xMin, rect.yMax, 0)).GetPoint(d);
            Vector3 tr = cam.ViewportPointToRay(new Vector3(rect.xMax, rect.yMax, 0)).GetPoint(d);

            AddLine(RegionObjects, "OriginBL", o, bl, Color.yellow, _yellow, .006f);
            AddLine(RegionObjects, "OriginBR", o, br, Color.yellow, _yellow, .006f);
            AddLine(RegionObjects, "OriginTL", o, tl, Color.yellow, _yellow, .006f);
            AddLine(RegionObjects, "OriginTR", o, tr, Color.yellow, _yellow, .006f);
            AddLine(RegionObjects, "Bottom", bl, br, Color.yellow, _yellow, .008f);
            AddLine(RegionObjects, "Right", br, tr, Color.yellow, _yellow, .008f);
            AddLine(RegionObjects, "Top", tr, tl, Color.yellow, _yellow, .008f);
            AddLine(RegionObjects, "Left", tl, bl, Color.yellow, _yellow, .008f);
            ApplyVisibility(RegionObjects);
        }

        public static void DrawSelectedBounds(Bounds b)
        {
            ClearList(SelectionObjects);
            EnsureMaterials();
            Vector3 min = b.min, max = b.max;
            Vector3 p000 = new Vector3(min.x,min.y,min.z), p100 = new Vector3(max.x,min.y,min.z);
            Vector3 p010 = new Vector3(min.x,max.y,min.z), p110 = new Vector3(max.x,max.y,min.z);
            Vector3 p001 = new Vector3(min.x,min.y,max.z), p101 = new Vector3(max.x,min.y,max.z);
            Vector3 p011 = new Vector3(min.x,max.y,max.z), p111 = new Vector3(max.x,max.y,max.z);
            Vector3[,] e = {
                {p000,p100},{p100,p110},{p110,p010},{p010,p000},
                {p001,p101},{p101,p111},{p111,p011},{p011,p001},
                {p000,p001},{p100,p101},{p010,p011},{p110,p111}
            };
            for (int i=0;i<12;i++) AddLine(SelectionObjects,"Bounds"+i,e[i,0],e[i,1],Color.cyan,_cyan,.009f);
            ApplyVisibility(SelectionObjects);
        }

        public static void DrawFinalHit(Vector3 origin, Vector3 point, Vector3 normal, bool hasFloor, Vector3 floor)
        {
            ClearList(HitObjects);
            EnsureMaterials();
            AddLine(HitObjects,"Ray",origin,point,Color.red,_red,.012f);
            AddSphere(HitObjects,"Hit",point,.045f,_red);
            AddLine(HitObjects,"Normal",point,point+normal.normalized*.30f,Color.red,_red,.008f);
            if (hasFloor)
            {
                AddSphere(HitObjects,"Floor",floor,.035f,_green);
                AddLine(HitObjects,"Height",floor,point,Color.green,_green,.008f);
            }
            ApplyVisibility(HitObjects);
        }

        public static void SetVisible(bool visible)
        {
            Visible = visible;
            ApplyVisibility(RegionObjects); ApplyVisibility(SelectionObjects); ApplyVisibility(HitObjects);
        }

        public static void ClearAll()
        {
            ClearList(RegionObjects); ClearList(SelectionObjects); ClearList(HitObjects);
        }

        private static void AddLine(List<GameObject> owner,string name,Vector3 a,Vector3 b,Color c,Material m,float w)
        {
            GameObject go = new GameObject(Prefix+name); go.layer = DebugLayer;
            LineRenderer lr = go.AddComponent<LineRenderer>();
            lr.useWorldSpace=true; lr.positionCount=2; lr.SetPosition(0,a); lr.SetPosition(1,b);
            lr.startWidth=w; lr.endWidth=w; lr.startColor=c; lr.endColor=c; if(m!=null) lr.material=m;
            owner.Add(go);
        }

        private static void AddSphere(List<GameObject> owner,string name,Vector3 p,float size,Material m)
        {
            GameObject go=GameObject.CreatePrimitive(PrimitiveType.Sphere); go.name=Prefix+name; go.layer=DebugLayer;
            go.transform.position=p; go.transform.localScale=Vector3.one*size;
            Collider col=go.GetComponent<Collider>(); if(col!=null) col.enabled=false;
            Renderer r=go.GetComponent<Renderer>(); if(r!=null && m!=null) r.material=m;
            owner.Add(go);
        }

        private static void EnsureMaterials()
        {
            if(_yellow==null) _yellow=CreateMaterial(Color.yellow);
            if(_cyan==null) _cyan=CreateMaterial(Color.cyan);
            if(_red==null) _red=CreateMaterial(Color.red);
            if(_green==null) _green=CreateMaterial(Color.green);
        }

        private static Material CreateMaterial(Color c)
        {
            Shader s=Shader.Find("Sprites/Default"); if(s==null) s=Shader.Find("Unlit/Color");
            if(s==null) return null; Material m=new Material(s); m.color=c; return m;
        }

        private static void ClearList(List<GameObject> list)
        {
            for(int i=0;i<list.Count;i++) if(list[i]!=null) UnityEngine.Object.Destroy(list[i]);
            list.Clear();
        }

        private static void ApplyVisibility(List<GameObject> list)
        {
            for(int i=0;i<list.Count;i++) if(list[i]!=null) list[i].SetActive(Visible);
        }
    }
}
