// MtcCaptureRoot.cs  (Editor)
// -----------------------------------------------------------------------------
// Where an environment's folder lives under MTC_CAPTURE_ROOT.
//
// Two layouts coexist by design:
//   <root>/<env_id>/                    env with no capture yet
//   <root>/MM-DD-YYYY/<env_id>/         env filed under the day it was recorded
//
// The date folder is what makes a session traceable to whoever ran it, so every
// tool that walks the capture root has to look one level deeper. Resolving it in
// one place keeps TrimCaptureMode and AtomicEnvSet from drifting apart.
// -----------------------------------------------------------------------------
using System.Collections.Generic;
using System.IO;
using System.Text.RegularExpressions;

public static class MtcCaptureRoot
{
    static readonly Regex DateDir = new Regex(@"^\d{2}-\d{2}-\d{4}$");

    public static string Path_ =>
        System.Environment.GetEnvironmentVariable("MTC_CAPTURE_ROOT") ?? @"E:\MTC_data\capture";

    /// env_id -> folder, across both layouts. First match wins, so an env that
    /// somehow exists in two places resolves to the shallower one rather than
    /// silently picking a random day.
    public static Dictionary<string, string> EnvDirs()
    {
        var map = new Dictionary<string, string>();
        if (!Directory.Exists(Path_)) return map;
        foreach (var p in Directory.GetDirectories(Path_))
        {
            string name = System.IO.Path.GetFileName(p);
            if (name.StartsWith("_")) continue;
            if (DateDir.IsMatch(name))
            {
                foreach (var q in Directory.GetDirectories(p))
                {
                    string inner = System.IO.Path.GetFileName(q);
                    if (!map.ContainsKey(inner)) map[inner] = q;
                }
            }
            else if (!map.ContainsKey(name))
            {
                map[name] = p;
            }
        }
        return map;
    }

    public static string EnvDir(string envId)
    {
        return EnvDirs().TryGetValue(envId, out var d)
            ? d
            : System.IO.Path.Combine(Path_, envId);
    }

    /// The MM-DD-YYYY folder an env sits in, or "" when it has no capture yet.
    public static string CaptureDay(string envId)
    {
        string parent = System.IO.Path.GetFileName(System.IO.Path.GetDirectoryName(EnvDir(envId)) ?? "");
        return DateDir.IsMatch(parent) ? parent : "";
    }
}
