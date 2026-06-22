using System;

namespace TechCosmos.PhysicsSystem.Runtime
{
    /// <summary>
    /// 物理世界注册中心，便于项目层与 Unity 适配层共享默认实例。
    /// </summary>
    public static class PhysicsHub
    {
        private static PhysicsWorld _defaultWorld;

        public static PhysicsWorld DefaultWorld
        {
            get => _defaultWorld ??= PhysicsWorld.Create(PhysicsSettings.Default);
            set => _defaultWorld = value ?? throw new ArgumentNullException(nameof(value));
        }

        public static void Reset()
        {
            _defaultWorld = null;
        }
    }
}
