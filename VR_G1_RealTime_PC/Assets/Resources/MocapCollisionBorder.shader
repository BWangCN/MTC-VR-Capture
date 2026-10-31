Shader "MTC/Mocap Collision Border"
{
    Properties
    {
        _Color ("Border Color", Color) = (1, 0, 0, 0.35)
        _BorderWidth ("Border Width", Range(0.01, 0.5)) = 0.1
    }
    SubShader
    {
        Tags { "Queue" = "Overlay+1000" "RenderType" = "Transparent" "IgnoreProjector" = "True" "DisableBatching" = "True" }
        Cull Off
        ZWrite Off
        ZTest Always
        Blend SrcAlpha OneMinusSrcAlpha
        ColorMask RGB

        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #pragma multi_compile_instancing
            #include "UnityCG.cginc"

            struct appdata
            {
                float4 vertex : POSITION;
                float2 uv : TEXCOORD0;
                UNITY_VERTEX_INPUT_INSTANCE_ID
            };

            struct v2f
            {
                float4 position : SV_POSITION;
                float2 uv : TEXCOORD0;
                UNITY_VERTEX_OUTPUT_STEREO
            };

            fixed4 _Color;
            float _BorderWidth;

            v2f vert(appdata input)
            {
                v2f output;
                UNITY_SETUP_INSTANCE_ID(input);
                UNITY_INITIALIZE_OUTPUT(v2f, output);
                UNITY_INITIALIZE_VERTEX_OUTPUT_STEREO(output);

                // Fill each eye's viewport directly: no world-space panel or projection sizing.
                output.position = float4(input.vertex.xy, UNITY_NEAR_CLIP_VALUE, 1.0);
                output.uv = input.uv;
                return output;
            }

            fixed4 frag(v2f input) : SV_Target
            {
                UNITY_SETUP_STEREO_EYE_INDEX_POST_VERTEX(input);
                float2 edgeDistances = min(input.uv, 1.0 - input.uv);
                float edgeDistance = min(edgeDistances.x, edgeDistances.y);
                float opacity = 1.0 - smoothstep(0.0, _BorderWidth, edgeDistance);
                return fixed4(_Color.rgb, _Color.a * opacity);
            }
            ENDCG
        }
    }
    Fallback Off
}
