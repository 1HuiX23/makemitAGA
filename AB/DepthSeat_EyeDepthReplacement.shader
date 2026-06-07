Shader "Hidden/DepthSeat/EyeDepthReplacement"
{
    SubShader
    {
        Tags { "RenderType"="Opaque" }

        Pass
        {
            Cull Off
            ZWrite On
            ZTest LEqual

            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #include "UnityCG.cginc"

            struct appdata
            {
                float4 vertex : POSITION;
            };

            struct v2f
            {
                float4 pos : SV_POSITION;
                float eyeDepth : TEXCOORD0;
            };

            v2f vert(appdata v)
            {
                v2f o;
                o.pos = UnityObjectToClipPos(v.vertex);

                // 当前 camera view-space 下的正 eye depth，单位约等于世界单位。
                float3 viewPos = UnityObjectToViewPos(v.vertex);
                o.eyeDepth = -viewPos.z;

                return o;
            }

            float4 frag(v2f i) : SV_Target
            {
                return float4(i.eyeDepth, 0, 0, 1);
            }
            ENDCG
        }
    }

    Fallback Off
}