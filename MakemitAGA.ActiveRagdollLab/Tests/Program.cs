using System;

namespace ARL.Contracts
{
    internal static class Check
    {
        private static int _assertions, _groups, _failedGroups;
        internal static void That(bool condition, string description)
        {
            _assertions++;
            if (!condition) throw new InvalidOperationException(description);
        }
        internal static void Near(float actual, float expected, float tolerance, string description) =>
            That(float.IsFinite(actual) && MathF.Abs(actual - expected) <= tolerance,
                description + " (actual=" + actual + ", expected=" + expected + ")");
        internal static void Throws<T>(Action action, string description) where T : Exception
        {
            try { action(); }
            catch (T) { That(true, description); return; }
            That(false, description);
        }
        internal static void Run(string name, Action test)
        {
            _groups++;
            try { test(); Console.WriteLine("PASS " + name); }
            catch (Exception e) { _failedGroups++; Console.Error.WriteLine("FAIL " + name + ": " + e); }
        }
        internal static int Finish()
        {
            Console.WriteLine("CONTRACTS groups=" + _groups + " assertions=" + _assertions + " failedGroups=" + _failedGroups);
            return _failedGroups == 0 ? 0 : 1;
        }
    }

    internal static class Program
    {
        public static int Main()
        {
            Console.WriteLine("ARL managed contracts; no Unity/IL2CPP/PhysX certification.");
            RuntimeContracts.Run();
            CoordinatorContracts.Run();
            TrajectoryContracts.Run();
            SampleTimingContracts.Run();
            FootEvidenceContracts.Run();
            return Check.Finish();
        }
    }
}
