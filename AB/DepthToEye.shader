Shader "Hidden/DepthSeat/DepthToEye"
{
    SubShader
    {
        Cull Off ZWrite Off ZTest Always

        Pass
        {
            CGPROGRAM
            #pragma vertex vert_img
            #pragma fragment frag
            #include "UnityCG.cginc"

            sampler2D_float _CameraDepthTexture;

            fixed4 frag(v2f_img i) : SV_Target
            {
                float raw = SAMPLE_DEPTH_TEXTURE(_CameraDepthTexture, i.uv);

                // 输出 eye depth，单位接近世界单位。
                // C# 侧会用 camera ray + eye depth 反投影。
                float eye = LinearEyeDepth(raw);

                return float4(eye, eye, eye, 1.0);
            }
            ENDCG
        }
    }

    Fallback Off
}