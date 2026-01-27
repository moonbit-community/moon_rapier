// Copyright 2025 International Digital Economy Academy
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

///|
pub struct PhysicsPipeline {
  prev_collisions : Array[
    (
      @collision.ColliderHandle,
      @collision.ColliderHandle,
      @collision.CollisionEventFlags,
      Bool,
    ),
  ]
  contact_solver_cache : Array[ContactImpulseCacheEntry]
}

///|
pub fn PhysicsPipeline::new() -> PhysicsPipeline {
  { prev_collisions: [], contact_solver_cache: [] }
}

///|
fn min_value(a : @core.Real, b : @core.Real) -> @core.Real {
  if a < b {
    a
  } else {
    b
  }
}

///|
fn max_value(a : @core.Real, b : @core.Real) -> @core.Real {
  if a > b {
    a
  } else {
    b
  }
}

///|
fn clamp_value(
  value : @core.Real,
  min_val : @core.Real,
  max_val : @core.Real,
) -> @core.Real {
  max_value(min_val, min_value(max_val, value))
}

///|
fn separation_ball_ball(
  pos1 : @core.Vec2,
  r1 : @core.Real,
  pos2 : @core.Vec2,
  r2 : @core.Real,
) -> @core.Vec2? {
  let delta = pos1.sub(pos2)
  let dist = delta.length()
  let radius = r1 + r2
  if dist > radius {
    None
  } else {
    let dir = if dist <= 1.0e-6F {
      @core.Vec2::new(1.0F, 0.0F)
    } else {
      delta.normalize()
    }
    let depth = radius - dist
    Some(@core.Vec2::new(dir.x * depth, dir.y * depth))
  }
}

///|
fn vec2_scale(v : @core.Vec2, s : @core.Real) -> @core.Vec2 {
  @core.Vec2::new(v.x * s, v.y * s)
}

///|
fn cuboid_axes(rotation : @core.Real) -> (@core.Vec2, @core.Vec2) {
  let rot = @core.Rot2::from_angle(rotation)
  (@core.Vec2::new(rot.cos, rot.sin), @core.Vec2::new(-rot.sin, rot.cos))
}

///|
fn projection_radius(
  axis : @core.Vec2,
  cuboid_axis_x : @core.Vec2,
  cuboid_axis_y : @core.Vec2,
  half_width : @core.Real,
  half_height : @core.Real,
) -> @core.Real {
  @core.abs(axis.dot(cuboid_axis_x)) * half_width +
  @core.abs(axis.dot(cuboid_axis_y)) * half_height
}

///|
fn separation_cuboid_cuboid(
  pos1 : @core.Vec2,
  rot1 : @core.Real,
  hw1 : @core.Real,
  hh1 : @core.Real,
  pos2 : @core.Vec2,
  rot2 : @core.Real,
  hw2 : @core.Real,
  hh2 : @core.Real,
) -> @core.Vec2? {
  let (ax1, ay1) = cuboid_axes(rot1)
  let (ax2, ay2) = cuboid_axes(rot2)
  let delta = pos1.sub(pos2)
  let mut best_overlap = 0.0F
  let mut best_axis = @core.Vec2::new(1.0F, 0.0F)
  let mut best_sign = 1.0F
  let axes = [ax1, ay1, ax2, ay2]
  for i in 0..<axes.length() {
    let axis = axes[i]
    let proj = delta.dot(axis)
    let dist = @core.abs(proj)
    let r1 = projection_radius(axis, ax1, ay1, hw1, hh1)
    let r2 = projection_radius(axis, ax2, ay2, hw2, hh2)
    let overlap = r1 + r2 - dist
    if overlap <= 0.0F {
      return None
    }
    if i == 0 || overlap < best_overlap {
      best_overlap = overlap
      best_axis = axis
      best_sign = if proj < 0.0F { -1.0F } else { 1.0F }
    }
  }
  Some(vec2_scale(best_axis, best_overlap * best_sign))
}

///|
fn separation_ball_cuboid(
  ball_pos : @core.Vec2,
  radius : @core.Real,
  cuboid_pos : @core.Vec2,
  cuboid_rot : @core.Real,
  half_width : @core.Real,
  half_height : @core.Real,
) -> @core.Vec2? {
  let rot = @core.Rot2::from_angle(cuboid_rot)
  let inv = rot.inverse()
  let local_ball = inv.rotate_vec2(ball_pos.sub(cuboid_pos))
  let clamped = @core.Vec2::new(
    clamp_value(local_ball.x, -half_width, half_width),
    clamp_value(local_ball.y, -half_height, half_height),
  )
  let local_delta = local_ball.sub(clamped)
  let dist = local_delta.length()
  if dist > radius {
    return None
  }
  if dist > 1.0e-6F {
    let dir_world = rot.rotate_vec2(local_delta.normalize())
    let depth = radius - dist
    Some(vec2_scale(dir_world, depth))
  } else {
    // Circle center is inside (or extremely close to) the cuboid. Pick the closest face.
    let dx = half_width - @core.abs(local_ball.x)
    let dy = half_height - @core.abs(local_ball.y)
    if dx < dy {
      let sign = if local_ball.x < 0.0F { -1.0F } else { 1.0F }
      let dir_world = rot.rotate_vec2(@core.Vec2::new(sign, 0.0F))
      Some(vec2_scale(dir_world, radius + dx))
    } else {
      let sign = if local_ball.y < 0.0F { -1.0F } else { 1.0F }
      let dir_world = rot.rotate_vec2(@core.Vec2::new(0.0F, sign))
      Some(vec2_scale(dir_world, radius + dy))
    }
  }
}

///|
fn separation_vector(
  shape1 : @collision.Shape,
  pos1 : @core.Vec2,
  rot1 : @core.Real,
  shape2 : @collision.Shape,
  pos2 : @core.Vec2,
  rot2 : @core.Real,
) -> @core.Vec2? {
  fn segment_world_endpoints(
    center : @core.Vec2,
    rotation : @core.Real,
    a : @core.Vec2,
    b : @core.Vec2,
  ) -> (@core.Vec2, @core.Vec2) {
    let rot = @core.Rot2::from_angle(rotation)
    (center.add(rot.rotate_vec2(a)), center.add(rot.rotate_vec2(b)))
  }

  fn closest_point_on_segment(
    a : @core.Vec2,
    b : @core.Vec2,
    p : @core.Vec2,
  ) -> @core.Vec2 {
    let ab = b.sub(a)
    let denom = ab.dot(ab)
    if denom <= 1.0e-12F {
      a
    } else {
      let t = clamp_value(p.sub(a).dot(ab) / denom, 0.0F, 1.0F)
      a.add(vec2_scale(ab, t))
    }
  }

  fn segment_aabb_intersect(
    a : @core.Vec2,
    b : @core.Vec2,
    hw : @core.Real,
    hh : @core.Real,
  ) -> Bool {
    let d = b.sub(a)
    let mut t0 = 0.0F
    let mut t1 = 1.0F
    let p = [-d.x, d.x, -d.y, d.y]
    let q = [a.x + hw, hw - a.x, a.y + hh, hh - a.y]
    for i in 0..<4 {
      let pi = p[i]
      let qi = q[i]
      if @core.abs(pi) <= 1.0e-12F {
        if qi < 0.0F {
          return false
        }
      } else {
        let r = qi / pi
        if pi < 0.0F {
          if r > t1 {
            return false
          }
          if r > t0 {
            t0 = r
          }
        } else {
          if r < t0 {
            return false
          }
          if r < t1 {
            t1 = r
          }
        }
      }
    }
    true
  }

  fn separation_ball_segment(
    ball_center : @core.Vec2,
    radius : @core.Real,
    seg_center : @core.Vec2,
    seg_rot : @core.Real,
    a : @core.Vec2,
    b : @core.Vec2,
  ) -> @core.Vec2? {
    let (sa, sb) = segment_world_endpoints(seg_center, seg_rot, a, b)
    let q = closest_point_on_segment(sa, sb, ball_center)
    let delta = ball_center.sub(q)
    let dist = delta.length()
    if dist >= radius {
      None
    } else {
      let dir = if dist <= 1.0e-6F {
        // Fallback: use segment normal.
        let d = sb.sub(sa)
        let n = @core.Vec2::new(-d.y, d.x)
        if n.length_squared() <= 1.0e-12F {
          @core.Vec2::new(0.0F, 1.0F)
        } else {
          n.normalize()
        }
      } else {
        vec2_scale(delta, 1.0F / dist)
      }
      let depth = radius - dist
      Some(vec2_scale(dir, depth))
    }
  }

  fn separation_cuboid_segment(
    cuboid_center : @core.Vec2,
    cuboid_rot : @core.Real,
    half_width : @core.Real,
    half_height : @core.Real,
    seg_center : @core.Vec2,
    seg_rot : @core.Real,
    a : @core.Vec2,
    b : @core.Vec2,
  ) -> @core.Vec2? {
    let (sa, sb) = segment_world_endpoints(seg_center, seg_rot, a, b)
    // Early-out if they don't intersect.
    let rot = @core.Rot2::from_angle(cuboid_rot)
    let inv = rot.inverse()
    let la = inv.rotate_vec2(sa.sub(cuboid_center))
    let lb = inv.rotate_vec2(sb.sub(cuboid_center))
    if !segment_aabb_intersect(la, lb, half_width, half_height) {
      return None
    }
    fn project_cuboid(
      center : @core.Vec2,
      rot : @core.Real,
      hw : @core.Real,
      hh : @core.Real,
      axis : @core.Vec2,
    ) -> (@core.Real, @core.Real) {
      let (ax, ay) = cuboid_axes(rot)
      let c = center.dot(axis)
      let r = @core.abs(axis.dot(ax)) * hw + @core.abs(axis.dot(ay)) * hh
      (c - r, c + r)
    }

    fn project_segment(
      a : @core.Vec2,
      b : @core.Vec2,
      axis : @core.Vec2,
    ) -> (@core.Real, @core.Real) {
      let da = a.dot(axis)
      let db = b.dot(axis)
      if da < db {
        (da, db)
      } else {
        (db, da)
      }
    }

    fn support_point_cuboid_toward(
      center : @core.Vec2,
      rot : @core.Real,
      hw : @core.Real,
      hh : @core.Real,
      dir : @core.Vec2,
    ) -> @core.Vec2 {
      let (ax, ay) = cuboid_axes(rot)
      let sx = if dir.dot(ax) >= 0.0F { 1.0F } else { -1.0F }
      let sy = if dir.dot(ay) >= 0.0F { 1.0F } else { -1.0F }
      center.add(vec2_scale(ax, sx * hw)).add(vec2_scale(ay, sy * hh))
    }

    // SAT-style minimum translation vector (MTV) to separate the cuboid from the segment.
    //
    // Note: a segment is degenerate. Projecting it on its own normal produces a single point,
    // yielding zero overlap even when the segment crosses the cuboid. For that axis we use the
    // "point + normal" separation test (parry's segment_cuboid_find_local_separating_normal_oneway).
    let margin = 1.0e-6F
    let mut best : @core.Vec2? = None
    let mut best_len2 = 1.0e30F
    let (cub_ax, cub_ay) = cuboid_axes(cuboid_rot)
    let cub_axes = [cub_ax, cub_ay]
    for i in 0..<cub_axes.length() {
      let axis = cub_axes[i]
      let (min1, max1) = project_cuboid(
        cuboid_center, cuboid_rot, half_width, half_height, axis,
      )
      let (min2, max2) = project_segment(sa, sb, axis)
      let overlap = min_value(max1, max2) - max_value(min1, min2)
      if overlap <= 0.0F {
        return None
      }
      let shift_pos = max2 + margin - min1
      let shift_neg = min2 - margin - max1
      let shift = if shift_pos < -shift_neg { shift_pos } else { shift_neg }
      let cand = vec2_scale(axis, shift)
      let l2 = cand.length_squared()
      if l2 < best_len2 {
        best_len2 = l2
        best = Some(cand)
      }
    }
    let seg_dir = sb.sub(sa)
    let seg_n_raw = @core.Vec2::new(-seg_dir.y, seg_dir.x)
    if seg_n_raw.length_squared() > 1.0e-12F {
      let seg_n = seg_n_raw.normalize()
      let axis = if cuboid_center.sub(sa).dot(seg_n) >= 0.0F {
        seg_n
      } else {
        @core.Vec2::new(-seg_n.x, -seg_n.y)
      }
      let support = support_point_cuboid_toward(
        cuboid_center,
        cuboid_rot,
        half_width,
        half_height,
        @core.Vec2::new(-axis.x, -axis.y),
      )
      let separation = support.sub(sa).dot(axis)
      if separation > 0.0F {
        return None
      }
      let cand = vec2_scale(axis, -separation + margin)
      let l2 = cand.length_squared()
      if l2 < best_len2 {
        best_len2 = l2
        best = Some(cand)
      }
    }
    best
  }

  fn separation_basic(
    s1 : @collision.Shape,
    p1 : @core.Vec2,
    r1 : @core.Real,
    s2 : @collision.Shape,
    p2 : @core.Vec2,
    r2 : @core.Real,
  ) -> @core.Vec2? {
    match (s1, s2) {
      (@collision.Shape::Ball(br1), @collision.Shape::Ball(br2)) =>
        separation_ball_ball(p1, br1, p2, br2)
      (@collision.Shape::Cuboid(hw1, hh1), @collision.Shape::Cuboid(hw2, hh2)) =>
        separation_cuboid_cuboid(p1, r1, hw1, hh1, p2, r2, hw2, hh2)
      (@collision.Shape::Ball(r), @collision.Shape::Cuboid(hw, hh)) =>
        separation_ball_cuboid(p1, r, p2, r2, hw, hh)
      (@collision.Shape::Cuboid(hw, hh), @collision.Shape::Ball(r)) =>
        if separation_ball_cuboid(p2, r, p1, r1, hw, hh) is Some(sep) {
          Some(@core.Vec2::new(-sep.x, -sep.y))
        } else {
          None
        }
      (@collision.Shape::Ball(r), @collision.Shape::Segment(a, b)) =>
        separation_ball_segment(p1, r, p2, r2, a, b)
      (@collision.Shape::Segment(a, b), @collision.Shape::Ball(r)) =>
        if separation_ball_segment(p2, r, p1, r1, a, b) is Some(sep) {
          Some(@core.Vec2::new(-sep.x, -sep.y))
        } else {
          None
        }
      (@collision.Shape::Cuboid(hw, hh), @collision.Shape::Segment(a, b)) =>
        separation_cuboid_segment(p1, r1, hw, hh, p2, r2, a, b)
      (@collision.Shape::Segment(a, b), @collision.Shape::Cuboid(hw, hh)) =>
        if separation_cuboid_segment(p2, r2, hw, hh, p1, r1, a, b) is Some(sep) {
          Some(@core.Vec2::new(-sep.x, -sep.y))
        } else {
          None
        }
      _ => None
    }
  }

  fn capsule_components(
    s : @collision.Shape,
    center : @core.Vec2,
    rotation : @core.Real,
  ) -> Array[(@collision.Shape, @core.Vec2, @core.Real)] {
    match s {
      @collision.Shape::CapsuleX(half_height, radius) => {
        let rot = @core.Rot2::from_angle(rotation)
        let offset = rot.rotate_vec2(@core.Vec2::new(half_height, 0.0F))
        let a = center.add(offset)
        let b = center.sub(offset)
        [
          (@collision.Shape::Ball(radius), a, 0.0F),
          (@collision.Shape::Ball(radius), b, 0.0F),
          (@collision.Shape::Cuboid(half_height, radius), center, rotation),
        ]
      }
      @collision.Shape::CapsuleY(half_height, radius) => {
        let rot = @core.Rot2::from_angle(rotation)
        let offset = rot.rotate_vec2(@core.Vec2::new(0.0F, half_height))
        let a = center.add(offset)
        let b = center.sub(offset)
        [
          (@collision.Shape::Ball(radius), a, 0.0F),
          (@collision.Shape::Ball(radius), b, 0.0F),
          (@collision.Shape::Cuboid(radius, half_height), center, rotation),
        ]
      }
      _ => []
    }
  }

  fn best_sep(a : @core.Vec2?, b : @core.Vec2?) -> @core.Vec2? {
    if b is None {
      return a
    }
    if a is None {
      return b
    }
    let av = a.unwrap()
    let bv = b.unwrap()
    if bv.length_squared() > av.length_squared() {
      b
    } else {
      a
    }
  }

  match (shape1, shape2) {
    (@collision.Shape::Ball(_), @collision.Shape::Ball(_))
    | (@collision.Shape::Ball(_), @collision.Shape::Cuboid(_, _))
    | (@collision.Shape::Cuboid(_, _), @collision.Shape::Ball(_))
    | (@collision.Shape::Cuboid(_, _), @collision.Shape::Cuboid(_, _))
    | (@collision.Shape::Ball(_), @collision.Shape::Segment(_, _))
    | (@collision.Shape::Segment(_, _), @collision.Shape::Ball(_))
    | (@collision.Shape::Cuboid(_, _), @collision.Shape::Segment(_, _))
    | (@collision.Shape::Segment(_, _), @collision.Shape::Cuboid(_, _))
    | (@collision.Shape::Segment(_, _), @collision.Shape::Segment(_, _)) =>
      separation_basic(shape1, pos1, rot1, shape2, pos2, rot2)
    (@collision.Shape::Ball(_), @collision.Shape::CapsuleX(_, _))
    | (@collision.Shape::Ball(_), @collision.Shape::CapsuleY(_, _))
    | (@collision.Shape::Cuboid(_, _), @collision.Shape::CapsuleX(_, _))
    | (@collision.Shape::Cuboid(_, _), @collision.Shape::CapsuleY(_, _))
    | (@collision.Shape::Segment(_, _), @collision.Shape::CapsuleX(_, _))
    | (@collision.Shape::Segment(_, _), @collision.Shape::CapsuleY(_, _)) => {
      let comps2 = capsule_components(shape2, pos2, rot2)
      let mut best : @core.Vec2? = None
      for i in 0..<comps2.length() {
        let c2 = comps2[i]
        best = best_sep(
          best,
          separation_basic(shape1, pos1, rot1, c2.0, c2.1, c2.2),
        )
      }
      best
    }
    (@collision.Shape::CapsuleX(_, _), _)
    | (@collision.Shape::CapsuleY(_, _), _) => {
      let comps1 = capsule_components(shape1, pos1, rot1)
      let mut best : @core.Vec2? = None
      for i in 0..<comps1.length() {
        let c1 = comps1[i]
        let cand = match shape2 {
          @collision.Shape::CapsuleX(_, _) | @collision.Shape::CapsuleY(_, _) => {
            let comps2 = capsule_components(shape2, pos2, rot2)
            let mut best2 : @core.Vec2? = None
            for j in 0..<comps2.length() {
              let c2 = comps2[j]
              best2 = best_sep(
                best2,
                separation_basic(c1.0, c1.1, c1.2, c2.0, c2.1, c2.2),
              )
            }
            best2
          }
          _ => separation_basic(c1.0, c1.1, c1.2, shape2, pos2, rot2)
        }
        best = best_sep(best, cand)
      }
      best
    }
  }
}

///|
fn handle_user_changes_to_bodies(
  islands : @dynamics.IslandManager,
  bodies : @dynamics.RigidBodySet,
  colliders : @collision.ColliderSet,
  impulse_joints : @dynamics.ImpulseJointSet,
  multibody_joints : @dynamics.MultibodyJointSet,
  modified_bodies : Array[@dynamics.RigidBodyHandle],
) -> Unit {
  for i in 0..<modified_bodies.length() {
    let handle = modified_bodies[i]
    if bodies.get(handle) is Some(body) {
      if body.is_enabled() {
        islands.rigid_body_updated(handle, bodies)
      } else {
        islands.rigid_body_removed_or_disabled(handle, bodies)
      }
      colliders.set_parent_enabled(handle, body.is_enabled())
      impulse_joints.map_attached_joints_mut(handle, (
        parent,
        child,
        joint_handle,
        joint,
      ) => {
        parent |> ignore
        child |> ignore
        joint_handle |> ignore
        joint.set_enabled_by_attached_body(body.is_enabled()) |> ignore
      })
      multibody_joints |> ignore
    }
  }
}

///|
fn update_body_mass_properties_from_colliders(
  bodies : @dynamics.RigidBodySet,
  colliders : @collision.ColliderSet,
  modified_bodies : Array[@dynamics.RigidBodyHandle],
) -> Unit {
  for i in 0..<modified_bodies.length() {
    let handle = modified_bodies[i]
    if bodies.get_mut_internal(handle) is Some(body) {
      // Recompute like Rapier: sum all enabled colliders mass properties transformed by their
      // local pose relative to the rigid-body, then apply `additional_mass` as a mass scaling.
      let attached = colliders.colliders_with_parent(handle)
      let mut mprops = @core.MassProperties::default()
      let default_ccd = @dynamics.RigidBodyCcd::default()
      let mut ccd_thickness = default_ccd.ccd_thickness
      let mut ccd_max_dist = default_ccd.ccd_max_dist
      for j in 0..<attached.length() {
        let co_handle = attached[j]
        if colliders.get(co_handle) is Some(collider) {
          if collider.is_enabled() {
            let local_pos = @core.Isometry2::new(
              collider.local_translation,
              @core.Rot2::from_angle(collider.local_rotation),
            )
            mprops = mprops.add(
              collider.mass_properties().transform_by(local_pos),
            )
            let thickness = shape_ccd_thickness(collider.shape)
            if thickness < ccd_thickness {
              ccd_thickness = thickness
            }
            let radius = shape_bounding_radius(collider.shape)
            let dist = collider.local_translation.length() + radius
            if dist > ccd_max_dist {
              ccd_max_dist = dist
            }
          }
        }
      }
      let added_mass = body.additional_mass()
      if added_mass != 0.0F {
        mprops = mprops.set_mass(mprops.mass + added_mass, true)
      }
      body.set_local_mass_properties(mprops) |> ignore
      body.set_ccd_properties(ccd_thickness, ccd_max_dist) |> ignore
    }
  }
}

///|
fn clear_modified_bodies(
  bodies : @dynamics.RigidBodySet,
  modified_bodies : Array[@dynamics.RigidBodyHandle],
) -> Unit {
  for i in 0..<modified_bodies.length() {
    let handle = modified_bodies[i]
    if bodies.get_mut_internal(handle) is Some(body) {
      body.clear_changes() |> ignore
    }
  }
}

///|
fn clear_modified_colliders(
  colliders : @collision.ColliderSet,
  modified_colliders : Array[@collision.ColliderHandle],
) -> Unit {
  colliders.clear_changes_for(modified_colliders)
}

///|
fn collider_handle_equals(
  a : @collision.ColliderHandle,
  b : @collision.ColliderHandle,
) -> Bool {
  @collision.ColliderHandle::equals(a, b)
}

///|
fn collision_pair_equals(
  a : (@collision.ColliderHandle, @collision.ColliderHandle),
  b : (@collision.ColliderHandle, @collision.ColliderHandle),
) -> Bool {
  collider_handle_equals(a.0, b.0) && collider_handle_equals(a.1, b.1)
}

///|
fn collision_pair_in_list(
  pair : (@collision.ColliderHandle, @collision.ColliderHandle),
  list : Array[(@collision.ColliderHandle, @collision.ColliderHandle)],
) -> Bool {
  for i in 0..<list.length() {
    if collision_pair_equals(pair, list[i]) {
      return true
    }
  }
  false
}

///|
fn solve_joints_and_contacts_interleaved(
  parameters : IntegrationParameters,
  bodies : @dynamics.RigidBodySet,
  colliders : @collision.ColliderSet,
  narrow_phase : @collision.NarrowPhase,
  impulse_joints : @dynamics.ImpulseJointSet,
  dt : @core.Real,
  event_handler : EventHandler,
  contact_cache : Array[ContactImpulseCacheEntry],
) -> Unit {
  let joint_constraints = build_joint_constraints(
    parameters, bodies, impulse_joints, dt,
  )
  let contact_constraints = build_constraints(
    parameters,
    bodies,
    colliders,
    narrow_phase,
    dt,
    contact_cache,
    parameters.warmstart_coefficient,
  )
  if joint_constraints.length() > 0 {
    warmstart_joints(bodies, joint_constraints)
  }
  if contact_constraints.length() > 0 {
    warmstart(bodies, contact_constraints)
  }
  let base_iters = if parameters.num_solver_iterations <= 0 {
    1
  } else {
    parameters.num_solver_iterations
  }
  // Our contact solver is simpler than Rapier's (no block solver, no SIMD),
  // so we give contacts more iterations to improve stacking stability.
  let joint_iters = base_iters
  let contact_iters = base_iters
  let iters = if contact_iters > joint_iters {
    contact_iters
  } else {
    joint_iters
  }
  for i in 0..<iters {
    if joint_constraints.length() > 0 && i < joint_iters {
      solve_joint_velocity_constraints(bodies, joint_constraints, dt, 1)
    }
    if contact_constraints.length() > 0 {
      solve_velocity_constraints(bodies, dt, contact_constraints, 1)
    }
  }
  if contact_constraints.length() > 0 {
    solve_restitution_constraints(bodies, contact_constraints)
    position_correction(
      bodies,
      colliders,
      contact_constraints,
      parameters.allowed_linear_error(),
    )
  }
  if joint_constraints.length() > 0 {
    position_correction_joints(bodies, joint_constraints)
    writeback_joint_impulses(impulse_joints, joint_constraints)
  }
  if contact_constraints.length() > 0 {
    emit_contact_force_events(
      bodies, colliders, dt, event_handler, contact_constraints,
    )
    update_cache(contact_cache, contact_constraints)
  }
}

///|
fn compute_collision_event_flags(
  colliders : @collision.ColliderSet,
  h1 : @collision.ColliderHandle,
  h2 : @collision.ColliderHandle,
) -> @collision.CollisionEventFlags {
  let mut flags = @collision.CollisionEventFlags::empty()
  let mut sensor = false
  if colliders.get(h1) is Some(c1) {
    sensor = sensor || c1.is_sensor()
  } else {
    flags = flags.insert(@collision.CollisionEventFlags::removed())
  }
  if colliders.get(h2) is Some(c2) {
    sensor = sensor || c2.is_sensor()
  } else {
    flags = flags.insert(@collision.CollisionEventFlags::removed())
  }
  if sensor {
    flags = flags.insert(@collision.CollisionEventFlags::sensor())
  }
  flags
}

///|
fn should_emit_collision_events(
  colliders : @collision.ColliderSet,
  h1 : @collision.ColliderHandle,
  h2 : @collision.ColliderHandle,
) -> Bool {
  fn has_collision_events(collider : @collision.Collider) -> Bool {
    collider
    .active_events()
    .contains(@collision.ActiveEvents::collision_events())
  }

  let mut enabled = false
  if colliders.get(h1) is Some(c1) {
    enabled = enabled || has_collision_events(c1)
  }
  if colliders.get(h2) is Some(c2) {
    enabled = enabled || has_collision_events(c2)
  }
  enabled
}

///|
fn collect_current_collisions(
  narrow_phase : @collision.NarrowPhase,
  colliders : @collision.ColliderSet,
) -> Array[
  (
    @collision.ColliderHandle,
    @collision.ColliderHandle,
    @collision.CollisionEventFlags,
    Bool,
  ),
] {
  let result : Array[
    (
      @collision.ColliderHandle,
      @collision.ColliderHandle,
      @collision.CollisionEventFlags,
      Bool,
    ),
  ] = []
  let intersections = narrow_phase.intersection_pairs()
  for i in 0..<intersections.length() {
    let pair = intersections[i]
    if pair.intersecting {
      let flags = compute_collision_event_flags(
        colliders,
        pair.collider1,
        pair.collider2,
      )
      let emit = should_emit_collision_events(
        colliders,
        pair.collider1,
        pair.collider2,
      )
      result.push((pair.collider1, pair.collider2, flags, emit))
    }
  }
  let contacts = narrow_phase.contact_pairs()
  for i in 0..<contacts.length() {
    let pair = contacts[i]
    let flags = compute_collision_event_flags(colliders, pair.0, pair.1)
    let emit = should_emit_collision_events(colliders, pair.0, pair.1)
    result.push((pair.0, pair.1, flags, emit))
  }
  result
}

///|
fn PhysicsPipeline::emit_collision_events(
  self : PhysicsPipeline,
  bodies : @dynamics.RigidBodySet,
  colliders : @collision.ColliderSet,
  narrow_phase : @collision.NarrowPhase,
  event_handler : EventHandler,
) -> Unit {
  let current = collect_current_collisions(narrow_phase, colliders)
  let prev_pairs : Array[(@collision.ColliderHandle, @collision.ColliderHandle)] = []
  for i in 0..<self.prev_collisions.length() {
    let p = self.prev_collisions[i]
    prev_pairs.push((p.0, p.1))
  }
  let cur_pairs : Array[(@collision.ColliderHandle, @collision.ColliderHandle)] = []
  for i in 0..<current.length() {
    let c = current[i]
    cur_pairs.push((c.0, c.1))
  }
  for i in 0..<current.length() {
    let c = current[i]
    if !collision_pair_in_list((c.0, c.1), prev_pairs) && c.3 {
      event_handler.handle_collision_event(
        bodies,
        colliders,
        @collision.CollisionEvent::Started(c.0, c.1, c.2),
      )
    }
  }
  for i in 0..<self.prev_collisions.length() {
    let p = self.prev_collisions[i]
    if !collision_pair_in_list((p.0, p.1), cur_pairs) && p.3 {
      let mut flags = p.2
      if colliders.get(p.0) is None || colliders.get(p.1) is None {
        flags = flags.insert(@collision.CollisionEventFlags::removed())
      }
      event_handler.handle_collision_event(
        bodies,
        colliders,
        @collision.CollisionEvent::Stopped(p.0, p.1, flags),
      )
    }
  }
  self.prev_collisions.clear()
  for i in 0..<current.length() {
    self.prev_collisions.push(current[i])
  }
}

///|
pub fn PhysicsPipeline::step(
  self : PhysicsPipeline,
  gravity : @core.Vec2,
  parameters : IntegrationParameters,
  islands : @dynamics.IslandManager,
  broad_phase : @collision.BroadPhaseBvh,
  narrow_phase : @collision.NarrowPhase,
  bodies : @dynamics.RigidBodySet,
  colliders : @collision.ColliderSet,
  impulse_joints : @dynamics.ImpulseJointSet,
  multibody_joints : @dynamics.MultibodyJointSet,
  ccd_solver : @dynamics.CCDSolver,
  physics_hooks : PhysicsHooks,
  event_handler : EventHandler,
) -> Unit {
  let wake_from_impulse = impulse_joints.take_wake_up()
  for i in 0..<wake_from_impulse.length() {
    islands.wake_up(bodies, wake_from_impulse[i], true)
  }
  let wake_from_multibody = multibody_joints.take_wake_up()
  for i in 0..<wake_from_multibody.length() {
    islands.wake_up(bodies, wake_from_multibody[i], true)
  }
  let modified_colliders_before_step = colliders.take_modified()
  @collision.handle_user_changes_to_colliders(
    bodies, colliders, modified_colliders_before_step,
  )
  let modified_bodies_before_step = bodies.take_modified()
  handle_user_changes_to_bodies(
    islands, bodies, colliders, impulse_joints, multibody_joints, modified_bodies_before_step,
  )
  update_body_mass_properties_from_colliders(
    bodies, colliders, modified_bodies_before_step,
  )
  let join_from_impulse = impulse_joints.take_to_join()
  for i in 0..<join_from_impulse.length() {
    let pair = join_from_impulse[i]
    islands.interaction_started_or_stopped(
      bodies,
      Some(pair.0),
      Some(pair.1),
      true,
      false,
    )
  }
  let join_from_multibody = multibody_joints.take_to_join()
  for i in 0..<join_from_multibody.length() {
    let pair = join_from_multibody[i]
    islands.interaction_started_or_stopped(
      bodies,
      Some(pair.0),
      Some(pair.1),
      true,
      false,
    )
  }
  let prediction_distance = parameters.prediction_distance()
  let mut remaining_time = parameters.dt
  let mut remaining_substeps = parameters.max_ccd_substeps
  let ccd_is_enabled = remaining_substeps > 0
  if !ccd_is_enabled {
    remaining_substeps = 1
  }
  let min_ccd_dt = parameters.min_ccd_dt
  let collision_pipeline = CollisionPipeline::new()
  while remaining_substeps > 0 && remaining_time > 0.0F {
    let mut dt = remaining_time
    if ccd_is_enabled && remaining_substeps > 1 {
      // Take forces (gravity) into account for the initial TOI estimate.
      // NOTE: CCD activation uses `RigidBodyForces.force/torque`, so compute them first.
      for i in 0..<bodies.bodies.length() {
        if bodies.bodies[i] is Some(body) {
          if !body.enabled || body.is_sleeping() {
            continue
          }
          if body.body_type is @dynamics.RigidBodyType::Dynamic {
            body.update_effective_force_and_torque(gravity) |> ignore
          }
        }
      }
      let ccd_active = ccd_solver.update_ccd_active_flags(
        islands, bodies, remaining_time, true,
      )
      let first_impact = if ccd_active {
        find_first_ccd_impact(
          gravity, remaining_time, islands, bodies, colliders,
        )
      } else {
        None
      }
      if first_impact is Some(toi) {
        let steps_real = Float::from_double(remaining_substeps.to_double())
        let original_interval = remaining_time / steps_real
        dt = if toi < original_interval {
          original_interval
        } else {
          toi + (remaining_time - toi) / steps_real
        }
        remaining_substeps = remaining_substeps - 1
      } else {
        dt = remaining_time
        remaining_substeps = 0
      }
      remaining_time = remaining_time - dt
      if remaining_time <= min_ccd_dt {
        dt = dt + remaining_time
        remaining_time = 0.0F
        remaining_substeps = 0
      }
    } else {
      dt = remaining_time
      remaining_time = 0.0F
      remaining_substeps = 0
    }
    // Keep collider world poses synced with the current rigid-body positions before we
    // do any CCD shape casts (user may have teleported bodies without touching colliders).
    broad_phase.update(prediction_distance, bodies, colliders)
    for i in 0..<bodies.bodies.length() {
      if bodies.bodies[i] is Some(body) {
        if !body.enabled || body.is_sleeping() {
          continue
        }
        match body.body_type {
          @dynamics.RigidBodyType::Dynamic => {
            body.update_effective_force_and_torque(gravity) |> ignore
            let forces = body.forces()
            let raw_vels = forces.integrate(dt, body.vels(), body.mass_props())
            let damped_vels = raw_vels.apply_damping(dt, body.damping())
            let linvel = if body.locked_translations {
              @core.Vec2::zero()
            } else {
              damped_vels.linvel
            }
            let angvel = if body.locked_rotations {
              0.0F
            } else {
              damped_vels.angvel
            }
            body.set_linvel(linvel, false) |> ignore
            body.set_angvel(angvel, false) |> ignore
            let next_pos = body
              .vels()
              .integrate(dt, body.position(), body.center_of_mass())
            body.set_next_position_internal(next_pos) |> ignore
          }
          @dynamics.RigidBodyType::KinematicPositionBased => {
            // Rapier parity: derive velocities from position -> next_position using the
            // current substep dt.
            let inv_dt = 1.0F / dt
            let derived_vels = @dynamics.RigidBodyPosition::new(
              body.position(),
              body.next_position(),
            ).interpolate_velocity(inv_dt, body.center_of_mass())
            body.set_kinematic_position_based_vels(derived_vels) |> ignore
          }
          @dynamics.RigidBodyType::KinematicVelocityBased => {
            let next_pos = body
              .vels()
              .integrate(dt, body.position(), body.center_of_mass())
            body.set_next_position_internal(next_pos) |> ignore
          }
          _ => ()
        }
      }
    }
    if ccd_is_enabled {
      // Clamp motions of CCD-active bodies before advancing to their final positions.
      let ccd_active = ccd_solver.update_ccd_active_flags(
        islands, bodies, dt, false,
      )
      if ccd_active {
        let impacts = predict_ccd_impacts_at_next_positions(
          dt, islands, bodies, colliders,
        )
        ccd_solver.clamp_motions(dt, bodies, impacts)
      }
    }

    // Advance rigid-bodies to their final position for this substep.
    for i in 0..<bodies.bodies.length() {
      if bodies.bodies[i] is Some(body) {
        if !body.enabled || body.is_sleeping() {
          continue
        }
        if body.body_type.is_dynamic_or_kinematic() {
          body.set_position(body.next_position(), false) |> ignore
        }
      }
    }
    collision_pipeline.step(
      prediction_distance, islands, broad_phase, narrow_phase, bodies, colliders,
      impulse_joints, multibody_joints, physics_hooks, event_handler,
    )
    solve_joints_and_contacts_interleaved(
      parameters,
      bodies,
      colliders,
      narrow_phase,
      impulse_joints,
      dt,
      event_handler,
      self.contact_solver_cache,
    )
    collision_pipeline.step(
      prediction_distance, islands, broad_phase, narrow_phase, bodies, colliders,
      impulse_joints, multibody_joints, physics_hooks, event_handler,
    )
  }
  // Only emit collision events after the final collision detection pass.
  PhysicsPipeline::emit_collision_events(
    self, bodies, colliders, narrow_phase, event_handler,
  )
  let contacts = @dynamics.ContactGraph::new()
  for i in 0..<narrow_phase.contacts.length() {
    let contact = narrow_phase.contacts[i]
    if colliders.get(contact.0) is Some(collider1) &&
      colliders.get(contact.1) is Some(collider2) {
      if collider1.parent is Some(parent1) && collider2.parent is Some(parent2) {
        if !@dynamics.RigidBodyHandle::equals(parent1, parent2) {
          contacts.add_pair(parent1, parent2)
        }
      }
    }
  }
  islands.update_islands(
    parameters.dt,
    1.0F,
    bodies,
    contacts,
    impulse_joints,
    multibody_joints,
  )
  let modified_colliders_after_step = colliders.take_modified()
  let modified_bodies_after_step = bodies.take_modified()
  clear_modified_colliders(colliders, modified_colliders_before_step)
  clear_modified_colliders(colliders, modified_colliders_after_step)
  clear_modified_bodies(bodies, modified_bodies_before_step)
  clear_modified_bodies(bodies, modified_bodies_after_step)
}
