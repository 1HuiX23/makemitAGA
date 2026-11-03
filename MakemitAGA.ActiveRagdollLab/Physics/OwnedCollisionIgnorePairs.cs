using System.Collections.Generic;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    /// <summary>
    /// Restoration receipts for pairs whose originally unignored state this owner may change.
    /// Callers decide when isolation is needed, preserve pre-existing ignores, and remember each
    /// pair before the native write. Activation, readback policy and diagnostics remain with them.
    /// </summary>
    internal sealed class OwnedCollisionIgnorePairs
    {
        private sealed class Pair
        {
            internal Collider A;
            internal Collider B;
        }

        private readonly List<Pair> _pairs = new List<Pair>();

        internal int Count => _pairs.Count;

        internal bool Contains(Collider a, Collider b)
        {
            for (int i = 0; i < _pairs.Count; i++)
            {
                Pair pair = _pairs[i];
                if ((pair.A == a && pair.B == b) || (pair.A == b && pair.B == a)) return true;
            }
            return false;
        }

        // Bookkeeping only: a failed or partially applied native write still needs this receipt.
        internal bool Remember(Collider a, Collider b)
        {
            if (Contains(a, b)) return false;
            _pairs.Add(new Pair { A = a, B = b });
            return true;
        }

        internal int RestoreAll()
        {
            int failures = 0;
            for (int i = _pairs.Count - 1; i >= 0; i--)
            {
                Pair pair = _pairs[i];
                try
                {
                    if (pair.A != null && pair.B != null)
                    {
                        UnityEngine.Physics.IgnoreCollision(pair.A, pair.B, false);
                        if (UnityEngine.Physics.GetIgnoreCollision(pair.A, pair.B))
                        {
                            failures++;
                            continue;
                        }
                    }
                    // Only confirmed restoration or a destroyed endpoint releases ownership.
                    _pairs.RemoveAt(i);
                }
                catch { failures++; } // Retain this receipt and continue independent pairs.
            }
            return failures;
        }
    }
}
