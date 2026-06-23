/*
 * SeatVlmObjectDetector.cs
 * 把模型圈选的二维区域转换成真实 Unity GameObject 候选列表。
 * 黄色射线只属于 debug_svt，可见性默认关闭。
 */
using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Text;
using UnityEngine;

using MakemitAGA.World;
namespace MakemitAGA.Mita_self.Mita_tools
{
    internal sealed class DetectedObjectCandidate
    {
        public int Id;
        public GameObject Object;
        public Bounds Bounds;
        public Rect ViewportRect;
        public Rect TopLeftRect;
        public float Distance;
        public float Score;
        public string Name;
        public string Path;
        public bool IsAlive => Object != null && Object.transform != null;

        public string ToJson()
        {
            return "{"+
                "\"id\":"+Id+","+
                "\"name\":\""+SeatVlmObjectDetector.JsonEscape(Name)+"\","+
                "\"path\":\""+SeatVlmObjectDetector.JsonEscape(Path)+"\","+
                "\"rect\":["+F(TopLeftRect.x)+","+F(TopLeftRect.y)+","+F(TopLeftRect.width)+","+F(TopLeftRect.height)+"],"+
                "\"distance\":"+F(Distance)+"}";
        }
        private static string F(float v)=>v.ToString("0.####",CultureInfo.InvariantCulture);
    }

    internal static class SeatVlmObjectDetector
    {
        private static Transform _cachedHouseRoot;
        public static void ClearSceneCache()=>_cachedHouseRoot=null;

        public static List<DetectedObjectCandidate> FindObjectsInRegion(float left,float top,float width,float height,out Rect searchRect)
        {
            searchRect=Rect.zero;
            Camera cam=SeatVlmVisionManager.SnapshotCamera;
            if(cam==null) return new List<DetectedObjectCandidate>();

            left=Mathf.Clamp01(left); top=Mathf.Clamp01(top); width=Mathf.Clamp01(width); height=Mathf.Clamp01(height);
            float right=Mathf.Clamp01(left+width), bottom=Mathf.Clamp01(top+height);
            searchRect=Rect.MinMaxRect(left,1f-bottom,right,1f-top);
            Vector2 center=searchRect.center;
            SeatVlmDebugVisuals.DrawSearchFrustum(cam,searchRect);

            List<Transform> roots=CollectCandidateRoots();
            List<DetectedObjectCandidate> result=new List<DetectedObjectCandidate>();
            for(int i=0;i<roots.Count;i++)
            {
                Transform prop=roots[i];
                if(prop==null || prop.gameObject==null || !prop.gameObject.activeInHierarchy || ShouldSkip(prop)) continue;
                Bounds bounds; if(!TryGetCombinedRendererBounds(prop,out bounds)) continue;
                Rect rect=GetScreenRect(cam,bounds); if(rect.width<=0 || rect.height<=0 || !searchRect.Overlaps(rect)) continue;
                Rect inter=Intersect(searchRect,rect);
                float overlap=Mathf.Max(.0001f,inter.width*inter.height);
                float dist=Vector3.Distance(cam.transform.position,bounds.center);
                float score=Vector2.Distance(rect.center,center)*4f + dist*.2f - overlap*6f;
                result.Add(new DetectedObjectCandidate{
                    Object=prop.gameObject,Bounds=bounds,ViewportRect=rect,TopLeftRect=ToTopLeftRect(rect),
                    Distance=dist,Score=score,Name=prop.name,Path=GetTransformPath(prop)});
            }

            List<DetectedObjectCandidate> sorted=result.OrderBy(c=>c.Score).Take(SeatVlmConfig.MaxCandidates).ToList();
            for(int i=0;i<sorted.Count;i++) sorted[i].Id=i;
            Plugin.Logger?.LogInfo("[SeatVlmObjectDetector] candidates="+sorted.Count);
            return sorted;
        }

        public static string BuildCandidatesJson(Rect search,List<DetectedObjectCandidate> candidates)
        {
            Rect r=ToTopLeftRect(search); StringBuilder sb=new StringBuilder();
            sb.Append("{\"region\":[").Append(F(r.x)).Append(",").Append(F(r.y)).Append(",").Append(F(r.width)).Append(",").Append(F(r.height)).Append("],");
            sb.Append("\"count\":").Append(candidates==null?0:candidates.Count).Append(",\"objects\":[");
            if(candidates!=null) for(int i=0;i<candidates.Count;i++){if(i>0)sb.Append(",");sb.Append(candidates[i].ToJson());}
            sb.Append("]}"); return sb.ToString();
        }

        public static bool TryGetCombinedRendererBounds(Transform root,out Bounds bounds)
        {
            bounds=new Bounds(); if(root==null) return false; bool has=false; Renderer[] rs;
            try{rs=root.GetComponentsInChildren<Renderer>();}catch{return false;}
            for(int i=0;i<rs.Length;i++)
            {
                Renderer r=rs[i]; if(r==null || !r.enabled || SeatVlmDebugVisuals.IsDebugTransform(r.transform)) continue;
                if(!has){bounds=r.bounds;has=true;}else bounds.Encapsulate(r.bounds);
            }
            return has;
        }

        public static Rect GetScreenRect(Camera cam,Bounds b)
        {
            Vector3 c=b.center,e=b.extents;
            Vector3[] p={
                new Vector3(c.x-e.x,c.y-e.y,c.z-e.z),new Vector3(c.x+e.x,c.y-e.y,c.z-e.z),
                new Vector3(c.x-e.x,c.y-e.y,c.z+e.z),new Vector3(c.x+e.x,c.y-e.y,c.z+e.z),
                new Vector3(c.x-e.x,c.y+e.y,c.z-e.z),new Vector3(c.x+e.x,c.y+e.y,c.z-e.z),
                new Vector3(c.x-e.x,c.y+e.y,c.z+e.z),new Vector3(c.x+e.x,c.y+e.y,c.z+e.z)};
            float minX=1,maxX=0,minY=1,maxY=0; bool any=false;
            for(int i=0;i<p.Length;i++)
            {
                Vector3 v=cam.WorldToViewportPoint(p[i]); if(v.z<=0)continue; any=true;
                minX=Mathf.Min(minX,v.x);maxX=Mathf.Max(maxX,v.x);minY=Mathf.Min(minY,v.y);maxY=Mathf.Max(maxY,v.y);
            }
            if(!any)return Rect.zero; minX=Mathf.Clamp01(minX);maxX=Mathf.Clamp01(maxX);minY=Mathf.Clamp01(minY);maxY=Mathf.Clamp01(maxY);
            if(maxX<=minX || maxY<=minY)return Rect.zero; return Rect.MinMaxRect(minX,minY,maxX,maxY);
        }

        public static string JsonEscape(string v)=>(v??"").Replace("\\","\\\\").Replace("\"","\\\"").Replace("\r","\\r").Replace("\n","\\n").Replace("\t","\\t");

        private static List<Transform> CollectCandidateRoots()
        {
            List<Transform> list=new List<Transform>(); Transform house=GetHouseRoot();
            if(house!=null)
            {
                for(int i=0;i<house.childCount;i++)
                {
                    Transform room=house.GetChild(i); if(room==null || !room.gameObject.activeInHierarchy)continue;
                    for(int j=0;j<room.childCount;j++){Transform prop=room.GetChild(j);if(prop!=null&&prop.gameObject.activeInHierarchy)list.Add(prop);}
                }
                return list;
            }
            Renderer[] rs=UnityEngine.Object.FindObjectsOfType<Renderer>(); HashSet<int> seen=new HashSet<int>();
            for(int i=0;i<rs.Length;i++)
            {
                Renderer r=rs[i]; if(r==null||!r.enabled)continue; Transform t=r.transform.parent!=null?r.transform.parent:r.transform;
                int id=t.GetInstanceID(); if(seen.Add(id))list.Add(t);
            }
            return list;
        }

        private static Transform GetHouseRoot()
        {
            if(_cachedHouseRoot!=null)
            {
                try{if(_cachedHouseRoot.gameObject!=null&&_cachedHouseRoot.gameObject.activeInHierarchy)return _cachedHouseRoot;}catch{}
                _cachedHouseRoot=null;
            }
            GameObject[] all=UnityEngine.Object.FindObjectsOfType<GameObject>();
            for(int i=0;i<all.Length;i++)
            {
                GameObject go=all[i]; if(go==null||go.name!="HouseGame Tamagotchi")continue;
                Transform house=go.transform.Find("House"); if(house!=null){_cachedHouseRoot=house;return house;}
            }
            return null;
        }

        private static bool ShouldSkip(Transform t)
        {
            string n=(t.name??"").ToLowerInvariant();
            return n.Contains("blink")||n.Contains("shadow")||n.Contains("particle")||n.Contains("beam")||n.Contains("vt_debug")||n.Contains("mita_internal_eye")||n.Contains("mita_snapshot_ghost")||n.Contains("ai_3dtext_instance");
        }

        private static Rect Intersect(Rect a,Rect b)
        {
            float xmin=Mathf.Max(a.xMin,b.xMin),xmax=Mathf.Min(a.xMax,b.xMax),ymin=Mathf.Max(a.yMin,b.yMin),ymax=Mathf.Min(a.yMax,b.yMax);
            return xmax<=xmin||ymax<=ymin?Rect.zero:Rect.MinMaxRect(xmin,ymin,xmax,ymax);
        }
        private static Rect ToTopLeftRect(Rect r)=>new Rect(r.xMin,1f-r.yMax,r.width,r.height);
        private static string GetTransformPath(Transform t){if(t==null)return"<null>";string p=t.name;for(Transform q=t.parent;q!=null;q=q.parent)p=q.name+"/"+p;return p;}
        private static string F(float v)=>v.ToString("0.####",CultureInfo.InvariantCulture);
    }
}
