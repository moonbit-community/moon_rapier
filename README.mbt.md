# Milky2018/moon_rapier

`moon_rapier` is a MoonBit rigid-body physics library based on Rapier.

It provides:
- 2D physics APIs in `core`, `collision`, `dynamics`, `pipeline`, `control`.
- 3D physics APIs with `3D`/`3DReal` types in the same packages (for example `RigidBodySet3D`, `PhysicsPipeline3DReal`).

## Add to your project

In your `moon.pkg`:

```moonbit
import {
  "Milky2018/moon_rapier/core",
  "Milky2018/moon_rapier/collision",
  "Milky2018/moon_rapier/dynamics",
  "Milky2018/moon_rapier/pipeline",
}
```

If you use character controller / vehicle controller helpers, also import:

```moonbit
import {
  "Milky2018/moon_rapier/control",
  "Milky2018/moon_rapier/rapier3d",
}
```

## Quick start (2D)

```moonbit
let gravity = @core.Vec2::new(0.0F, -9.81F)
let params = @dynamics.IntegrationParameters::default()
let pipeline = @pipeline.PhysicsPipeline::new()
let islands = @dynamics.IslandManager::new()
let broad_phase = @collision.BroadPhaseBvh::new()
let narrow_phase = @collision.NarrowPhase::new()
let bodies = @dynamics.RigidBodySet::new()
let colliders = @collision.ColliderSet::new()
let impulse_joints = @dynamics.ImpulseJointSet::new()
let multibody_joints = @dynamics.MultibodyJointSet::new()
let ccd_solver = @dynamics.CCDSolver::new()
let hooks = @pipeline.PhysicsHooks::new()
let events = @pipeline.EventHandler::new()

let ground = bodies.insert(@dynamics.RigidBodyBuilder::fixed().build())
colliders.insert_with_parent(
  @collision.ColliderBuilder::cuboid(10.0F, 0.1F).build(),
  ground,
  bodies,
) |> ignore

let ball = bodies.insert(
  @dynamics.RigidBodyBuilder::dynamic()
  .translation(@core.Vec2::new(0.0F, 3.0F))
  .build(),
)
colliders.insert_with_parent(
  @collision.ColliderBuilder::ball(0.5F).build(),
  ball,
  bodies,
) |> ignore

pipeline.step(
  gravity, params, islands, broad_phase, narrow_phase, bodies, colliders,
  impulse_joints, multibody_joints, ccd_solver, hooks, events,
)
```

## Quick start (3D)

```moonbit
let gravity = @core.Vec3::new(0.0F, -9.81F, 0.0F)
let params = @dynamics.IntegrationParameters::default()
let pipeline = @pipeline.PhysicsPipeline3DReal::new()
let islands = @dynamics.IslandManager3D::new()
let broad_phase = @collision.BroadPhase3D::new()
let narrow_phase = @collision.NarrowPhase3D::new()
let bodies = @dynamics.RigidBodySet3D::new()
let colliders = @collision.ColliderSet3D::new()

let ground = bodies.insert(@dynamics.RigidBodyBuilder3D::fixed().build())
colliders.insert_with_parent(
  @collision.ColliderBuilder3D::cuboid(10.0F, 0.1F, 10.0F).build(),
  ground,
  bodies,
) |> ignore

let cube = bodies.insert(
  @dynamics.RigidBodyBuilder3D::dynamic()
  .translation(@core.Vec3::new(0.0F, 3.0F, 0.0F))
  .build(),
)
colliders.insert_with_parent(
  @collision.ColliderBuilder3D::cuboid(0.5F, 0.5F, 0.5F).build(),
  cube,
  bodies,
) |> ignore

pipeline.step(
  gravity, params, islands, broad_phase, narrow_phase, bodies, colliders,
)
```

## Main packages

- `Milky2018/moon_rapier/core`: math types and shared fundamentals.
- `Milky2018/moon_rapier/collision`: colliders, broad phase, narrow phase, queries.
- `Milky2018/moon_rapier/dynamics`: rigid bodies, joints, integration parameters.
- `Milky2018/moon_rapier/pipeline`: simulation stepping and event/hook interfaces.
- `Milky2018/moon_rapier/control`: character and vehicle controllers.
