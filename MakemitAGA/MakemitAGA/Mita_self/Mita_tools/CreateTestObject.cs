using UnityEngine;

namespace MakemitAGA.Mita_self.Mita_tools
{
    /// <summary>
    /// 只负责创建用于验证 sit(...) 的两个实验座椅。
    /// 真实环境中的座椅、床、沙发等物体不依赖这里；Mita_sit 会按名称在场景中查找目标。
    /// </summary>
    public static class CreateTestObject
    {
        public const string HighChairName = "TestChair_High";
        public const string LowChairName = "TestChair_Low";

        private static GameObject _highChair;
        private static GameObject _lowChair;

        /// <summary>
        /// 控制台命令 create_test_cube 的入口。
        /// 重复执行时不会不断创建新 cube，而是复用旧对象并重置到标准测试位置。
        /// </summary>
        public static void CreateTestCubes()
        {
            _highChair = CreateOrResetCube(
                _highChair,
                HighChairName,
                new Vector3(-7f, 0.25f, 0.5f),
                new Vector3(0.5f, 0.5f, 0.5f),
                Quaternion.Euler(0f, 90f, 0f));

            _lowChair = CreateOrResetCube(
                _lowChair,
                LowChairName,
                new Vector3(-7f, 0.05f, 1f),
                new Vector3(0.5f, 0.1f, 0.5f),
                Quaternion.Euler(0f, 90f, 0f));

            ConsoleMain.ConsolePrintGame("已创建测试座椅：TestChair_High / TestChair_Low。可输入 sit(TestChair_High) 或 sit(TestChair_Low)。");
        }

        private static GameObject CreateOrResetCube(GameObject cached, string name, Vector3 position, Vector3 scale, Quaternion rotation)
        {
            GameObject cube = cached;

            // 如果静态引用丢失，尝试按名称找回，避免重复创建。
            if (cube == null)
                cube = GameObject.Find(name);

            if (cube == null)
            {
                cube = GameObject.CreatePrimitive(PrimitiveType.Cube);
                cube.name = name;
            }

            cube.transform.position = position;
            cube.transform.localScale = scale;
            cube.transform.rotation = rotation;
            cube.SetActive(true);

            Collider col = cube.GetComponent<Collider>();
            if (col != null) col.isTrigger = false;

            return cube;
        }
    }
}
