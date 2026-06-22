# Tech-Cosmos Physics System

> **版本**: 1.0.0 | **命名空间**: `TechCosmos.PhysicsSystem.Runtime` / `TechCosmos.PhysicsSystem.Unity` | **Unity**: 2022.3+

---

## 1. 概述

**纯 C# 物理框架**，可在 Runtime 层完全脱离 `UnityEngine.Physics` 运行；Unity 层提供与 Unity API 对齐的替代入口，便于迁移现有项目。

```
Runtime (无引擎依赖)          Unity (可选适配)
────────────────────         ─────────────────────────────
PhysicsWorld                 CosmosPhysicsWorld (FixedUpdate)
PhysicsBody                  CosmosRigidbody / CosmosStaticBody
Raycast / Overlap            CosmosPhysics.Raycast / OverlapSphere
Collision Events             CosmosCollisionRelay
```

---

## 2. 设计原则

| 原则 | 说明 |
|------|------|
| **引擎无关核心** | `Runtime/` 使用 `noEngineReferences`，仅依赖 `com.techcosmos.core` |
| **API 可读** | 命名对齐 Unity：`Raycast`、`OverlapSphere`、`Rigidbody`、`IsTrigger` |
| **单一入口** | `PhysicsWorld` 负责模拟 + 查询 + 事件 |
| **渐进迁移** | 可只用 Runtime 做服务端/单元测试，也可在 Unity 中用 `CosmosPhysics` 替换 `Physics` |

---

## 3. 快速开始

### 3.1 Unity 场景

1. 创建空物体，添加 **`CosmosPhysicsWorld`**
2. 动态物体：`CosmosRigidbody` + `CosmosSphereCollider`（或 Box / Capsule）
3. 静态地面：`CosmosStaticBody` + `CosmosBoxCollider`
4. 查询代码将 `Physics.Raycast` 改为 `CosmosPhysics.Raycast`

```csharp
using TechCosmos.PhysicsSystem.Unity;
using UnityEngine;

if (CosmosPhysics.Raycast(origin, direction, out var hit, 100f))
{
    Debug.Log(hit.collider.name);
}
```

### 3.2 纯 Runtime（无 Unity）

```csharp
using TechCosmos.Core.Runtime;
using TechCosmos.PhysicsSystem.Runtime;

var world = PhysicsWorld.Create(PhysicsSettings.Default);

var ground = world.CreateBody(PhysicsBodyDescriptor.StaticBox(new Float3(0f, -1f, 0f), new Float3(20f, 1f, 20f)));
var ball = world.CreateBody(PhysicsBodyDescriptor.DynamicSphere(new Float3(0f, 5f, 0f), 0.5f, mass: 1f));

for (int i = 0; i < 120; i++)
    world.Simulate(1f / 60f);

if (world.Raycast(new Float3(0f, 10f, 0f), new Float3(0f, -1f, 0f), out var hit, 20f))
    System.Console.WriteLine($"Hit distance: {hit.distance}");
```

---

## 4. 核心 API

### PhysicsWorld

| 方法 | 说明 |
|------|------|
| `CreateBody(descriptor)` | 创建刚体 |
| `Simulate(deltaTime)` | 推进一帧物理 |
| `Raycast(origin, dir, out hit, maxDistance, layerMask)` | 射线检测 |
| `SphereCast(...)` | 球形投射 |
| `OverlapSphere(...)` | 球形重叠 |
| `OverlapBox(...)` | 盒形重叠 |

### 事件

- `CollisionEnter` / `CollisionStay` / `CollisionExit`
- Trigger 与 Collider 共用事件流，`IsTrigger` 区分

---

## 5. 支持范围（v1.0）

- **形状**: Sphere / Box / Capsule
- **刚体**: Static / Kinematic / Dynamic
- **求解**: 冲量法 + 空间哈希粗检测
- **查询**: Raycast、SphereCast、OverlapSphere、OverlapBox

> 复杂网格 (MeshCollider)、关节 (Joint)、CCD 等高级特性可在后续版本扩展。

---

## 6. 与 TargetingSystem 集成

TargetingSystem 的 `UnityPhysicsTargetingBackend` 可替换为基于 `CosmosPhysics.Raycast` 的后端，实现全链路脱离 Unity Physics。

---

## 7. 依赖

- `com.techcosmos.core` >= 1.1.0

---

## License

MIT — Tech-Cosmos
