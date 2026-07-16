// OscBodyReceiver.cs
// -----------------------------------------------------------------------------
// Receives ALVR's "VRChat Body OSC" tracker stream over LOCALHOST UDP (default 9000),
// parses the OSC messages, draws a sphere at each tracker (visible in the headset),
// and logs every OSC address it receives. Pure C# — no OpenVR, no extra packages.
//
// SETUP (ALVR side): in the ALVR Dashboard, set Body Tracking output/sink to
// "VRChat Body OSC" (port 9000). ALVR (on this PC) then sends the body trackers to
// 127.0.0.1:9000, which this script reads. VRChat itself does NOT need to be running.
//
// VRChat OSC trackers = up to 8 (hip, chest, 2x feet, 2x knees, 2x elbows) + a head
// reference, addresses like  /tracking/trackers/<n>/position  and  /rotation .
//
// This is the transport-verification + foundation: run it, confirm spheres appear and
// the Console lists the addresses, then I finalize the joint->capsule mapping +
// collision + haptics from the confirmed format.  Add via  MTC > Add OSC Body Receiver.
// -----------------------------------------------------------------------------
using System;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;

public class OscBodyReceiver : MonoBehaviour
{
    [Header("Network")]
    public int port = 9000;

    [Header("Alignment")]
    [Tooltip("Space the OSC tracker coords are expressed in (the play-space origin). " +
             "Usually the XR Origin. Auto-found by name if left empty. If null, used as-is (world).")]
    public Transform alignTo;

    [Header("Look")]
    public float markerRadius = 0.05f;
    public Color color = new Color(1f, 0.2f, 0.9f, 1f);

    [Header("Status (read-only)")]
    public int packets;
    public int addressCount;

    // address -> latest float args (filled on rx thread, read on main thread)
    readonly Dictionary<string, float[]> _latest = new Dictionary<string, float[]>();
    readonly object _lock = new object();

    UdpClient _udp;
    Thread _thread;
    volatile bool _running;

    readonly Dictionary<string, Transform> _markers = new Dictionary<string, Transform>();
    Material _mat;
    float _logAt;
    string _onScreen = "(waiting for OSC on 9000…)";

    void Start()
    {
        if (alignTo == null)
        {
            var xo = GameObject.Find("XR Origin");
            if (xo != null) alignTo = xo.transform;
        }
        _mat = MakeMat(color);

        try
        {
            _udp = new UdpClient(port);
            _running = true;
            _thread = new Thread(RxLoop) { IsBackground = true };
            _thread.Start();
            Debug.Log($"[OscBody] Listening for VRChat-OSC body trackers on udp/{port}. " +
                      "Set ALVR Body Tracking output = 'VRChat Body OSC'.");
        }
        catch (Exception e)
        {
            Debug.LogError($"[OscBody] Could not bind udp/{port}: {e.Message} (is something else using it?)");
            enabled = false;
        }
    }

    void RxLoop()
    {
        var ep = new IPEndPoint(IPAddress.Any, port);
        while (_running)
        {
            try
            {
                byte[] data = _udp.Receive(ref ep);
                packets++;
                ParsePacket(data, 0, data.Length);
            }
            catch (Exception e) { if (_running) Debug.LogWarning($"[OscBody] rx error: {e.Message}"); }
        }
    }

    // OSC packet = message ("/...") or bundle ("#bundle").
    void ParsePacket(byte[] b, int start, int len)
    {
        if (len < 4) return;
        if (b[start] == (byte)'#') // "#bundle\0" + timetag(8) + [int32 size + element]*
        {
            int p = start + 16;
            while (p + 4 <= start + len)
            {
                int sz = ReadInt32(b, ref p);
                if (sz <= 0 || p + sz > start + len) break;
                ParsePacket(b, p, sz);
                p += sz;
            }
            return;
        }

        int pos = start;
        string addr = ReadOscString(b, ref pos, start);
        if (string.IsNullOrEmpty(addr) || addr[0] != '/') return;
        if (pos >= start + len) return;
        string tags = ReadOscString(b, ref pos, start);
        if (tags == null || tags.Length == 0 || tags[0] != ',') return;

        var args = new List<float>(tags.Length);
        for (int i = 1; i < tags.Length; i++)
        {
            char t = tags[i];
            if (t == 'f') args.Add(ReadFloat32(b, ref pos));
            else if (t == 'i') args.Add(ReadInt32(b, ref pos));
            else if (t == 's') ReadOscString(b, ref pos, start);
            else break;
            if (pos > start + len) return;
        }
        lock (_lock) { _latest[addr] = args.ToArray(); }
    }

    void Update()
    {
        Dictionary<string, float[]> snap;
        lock (_lock) { snap = new Dictionary<string, float[]>(_latest); }
        addressCount = snap.Count;

        foreach (var kv in snap)
        {
            if (kv.Value != null && kv.Value.Length >= 3 &&
                kv.Key.EndsWith("position", StringComparison.OrdinalIgnoreCase))
            {
                Vector3 p = new Vector3(kv.Value[0], kv.Value[1], kv.Value[2]);
                if (alignTo != null) p = alignTo.TransformPoint(p);
                GetMarker(kv.Key).position = p;
            }
        }

        if (Time.time >= _logAt)
        {
            _logAt = Time.time + 2f;
            var sb = new StringBuilder($"[OscBody] packets={packets} addresses={snap.Count}\n");
            foreach (var kv in snap)
                sb.AppendLine($"  {kv.Key} = [{(kv.Value == null ? "" : string.Join(", ", Array.ConvertAll(kv.Value, x => x.ToString("F3"))))}]");
            _onScreen = sb.ToString();
            Debug.Log(_onScreen);
        }
    }

    /// World positions of the body trackers (excludes the head reference). Updated each frame.
    public List<Vector3> GetBodyTrackerWorldPositions()
    {
        var list = new List<Vector3>();
        foreach (var kv in _markers)
        {
            if (kv.Value == null) continue;
            if (kv.Key.IndexOf("head", StringComparison.OrdinalIgnoreCase) >= 0) continue; // head comes from the camera
            if (!kv.Key.EndsWith("position", StringComparison.OrdinalIgnoreCase)) continue;
            list.Add(kv.Value.position);
        }
        return list;
    }

    Transform GetMarker(string addr)
    {
        if (_markers.TryGetValue(addr, out var t) && t) return t;
        var s = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        s.name = "osc:" + addr;
        var c = s.GetComponent<Collider>(); if (c) Destroy(c);
        s.transform.localScale = Vector3.one * (markerRadius * 2f);
        var r = s.GetComponent<Renderer>(); if (r) r.sharedMaterial = _mat;
        s.transform.SetParent(transform, true);
        _markers[addr] = s.transform;
        return s.transform;
    }

    void OnGUI()
    {
        var st = new GUIStyle(GUI.skin.label) { fontSize = 12 };
        st.normal.textColor = Color.white;
        GUI.Label(new Rect(10, 300, 1000, 500), _onScreen, st);
    }

    void OnDestroy()
    {
        _running = false;
        _udp?.Close();
        if (_thread != null && _thread.IsAlive) _thread.Join(200);
    }

    // ---- OSC primitives (big-endian; strings null-terminated, padded to 4 bytes) ----
    static int ReadInt32(byte[] b, ref int p)
    {
        int v = (b[p] << 24) | (b[p + 1] << 16) | (b[p + 2] << 8) | b[p + 3];
        p += 4; return v;
    }
    static float ReadFloat32(byte[] b, ref int p)
    {
        byte[] t = { b[p + 3], b[p + 2], b[p + 1], b[p] };
        p += 4; return BitConverter.ToSingle(t, 0);
    }
    // pad alignment is relative to the OSC packet/element start
    static string ReadOscString(byte[] b, ref int p, int start)
    {
        int s = p;
        while (p < b.Length && b[p] != 0) p++;
        string str = Encoding.ASCII.GetString(b, s, p - s);
        p++;                                   // consume null
        while (((p - start) % 4) != 0) p++;    // 4-byte align
        return str;
    }

    static Material MakeMat(Color c)
    {
        Shader sh = Shader.Find("Unlit/Color") ?? Shader.Find("Standard");
        var m = new Material(sh) { color = c };
        if (m.HasProperty("_BaseColor")) m.SetColor("_BaseColor", c);
        return m;
    }

#if UNITY_EDITOR
    [UnityEditor.MenuItem("MTC/Add OSC Body Receiver to Scene")]
    static void AddToScene()
    {
        var go = new GameObject("OscBodyReceiver");
        go.AddComponent<OscBodyReceiver>();
        UnityEditor.Undo.RegisterCreatedObjectUndo(go, "Add OscBodyReceiver");
        UnityEditor.Selection.activeGameObject = go;
    }
#endif
}
