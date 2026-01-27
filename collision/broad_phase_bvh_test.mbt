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
test "broad phase bvh pairs basic" {
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = ColliderSet::new()
  let body1 = @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::zero())
    .build()
  let body2 = @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::zero())
    .build()
  let h1 = bodies.insert(body1)
  let h2 = bodies.insert(body2)
  let c1 = ColliderBuilder::ball(0.5F).build()
  let c2 = ColliderBuilder::ball(0.5F).build()
  let ch1 = colliders.insert_with_parent(c1, h1, bodies)
  let ch2 = colliders.insert_with_parent(c2, h2, bodies)
  let broad_phase = BroadPhaseBvh::new()
  broad_phase.update(0.0F, bodies, colliders)
  let pairs = broad_phase.pairs()
  inspect(pairs.length() == 1, content="true")
  let pair = pairs[0]
  let matches = ColliderHandle::equals(pair.0, ch1) &&
    ColliderHandle::equals(pair.1, ch2)
  let matches_swapped = ColliderHandle::equals(pair.0, ch2) &&
    ColliderHandle::equals(pair.1, ch1)
  inspect(matches || matches_swapped, content="true")
}

///|
test "broad phase bvh skips disabled colliders" {
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = ColliderSet::new()
  let h1 = bodies.insert(@dynamics.RigidBodyBuilder::dynamic().build())
  let h2 = bodies.insert(@dynamics.RigidBodyBuilder::dynamic().build())
  let c1 = ColliderBuilder::ball(0.5F).build()
  let c2 = ColliderBuilder::ball(0.5F).enabled(false).build()
  colliders.insert_with_parent(c1, h1, bodies) |> ignore
  colliders.insert_with_parent(c2, h2, bodies) |> ignore
  let broad_phase = BroadPhaseBvh::new()
  broad_phase.update(0.0F, bodies, colliders)
  inspect(broad_phase.pairs().length() == 0, content="true")
}

///|
test "broad phase bvh prediction distance expands aabb pairs" {
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = ColliderSet::new()
  let h1 = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::zero())
    .build(),
  )
  let h2 = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(1.01F, 0.0F))
    .build(),
  )
  colliders.insert_with_parent(ColliderBuilder::ball(0.5F).build(), h1, bodies)
  |> ignore
  colliders.insert_with_parent(ColliderBuilder::ball(0.5F).build(), h2, bodies)
  |> ignore
  let broad_phase = BroadPhaseBvh::new()
  broad_phase.update(0.0F, bodies, colliders)
  inspect(broad_phase.pairs().length() == 0, content="true")
  broad_phase.update(0.01F, bodies, colliders)
  inspect(broad_phase.pairs().length() == 1, content="true")
}

///|
fn rng_next(seed : Int) -> Int {
  // Deterministic LCG, 31-bit output.
  (seed * 1103515245 + 12345) & 0x7fffffff
}

///|
fn rng_float01(seed : Int) -> (@core.Real, Int) {
  let next = rng_next(seed)
  let value = Float::from_double(next.to_double() / 2147483647.0)
  (value, next)
}

///|
fn compute_aabb_for_test(
  shape : Shape,
  center : @core.Vec2,
  rotation : @core.Real,
  prediction_distance : @core.Real,
) -> (@core.Vec2, @core.Vec2) {
  let pd = if prediction_distance < 0.0F { 0.0F } else { prediction_distance }
  match shape {
    Shape::Ball(radius) => {
      let r = radius + pd
      (
        @core.Vec2::new(center.x - r, center.y - r),
        @core.Vec2::new(center.x + r, center.y + r),
      )
    }
    Shape::Cuboid(half_width, half_height) => {
      let rot = @core.Rot2::from_angle(rotation)
      let abs_cos = @core.abs(rot.cos)
      let abs_sin = @core.abs(rot.sin)
      let hw = abs_cos * half_width + abs_sin * half_height + pd
      let hh = abs_sin * half_width + abs_cos * half_height + pd
      (
        @core.Vec2::new(center.x - hw, center.y - hh),
        @core.Vec2::new(center.x + hw, center.y + hh),
      )
    }
    Shape::CapsuleX(half_height, radius) => {
      let rot = @core.Rot2::from_angle(rotation)
      let offset = rot.rotate_vec2(@core.Vec2::new(half_height, 0.0F))
      let a = center.add(offset)
      let b = center.sub(offset)
      let r = radius + pd
      let min_x = if a.x < b.x { a.x } else { b.x }
      let max_x = if a.x > b.x { a.x } else { b.x }
      let min_y = if a.y < b.y { a.y } else { b.y }
      let max_y = if a.y > b.y { a.y } else { b.y }
      (
        @core.Vec2::new(min_x - r, min_y - r),
        @core.Vec2::new(max_x + r, max_y + r),
      )
    }
    Shape::CapsuleY(half_height, radius) => {
      let rot = @core.Rot2::from_angle(rotation)
      let offset = rot.rotate_vec2(@core.Vec2::new(0.0F, half_height))
      let a = center.add(offset)
      let b = center.sub(offset)
      let r = radius + pd
      let min_x = if a.x < b.x { a.x } else { b.x }
      let max_x = if a.x > b.x { a.x } else { b.x }
      let min_y = if a.y < b.y { a.y } else { b.y }
      let max_y = if a.y > b.y { a.y } else { b.y }
      (
        @core.Vec2::new(min_x - r, min_y - r),
        @core.Vec2::new(max_x + r, max_y + r),
      )
    }
    Shape::Segment(a, b) => {
      let rot = @core.Rot2::from_angle(rotation)
      let wa = center.add(rot.rotate_vec2(a))
      let wb = center.add(rot.rotate_vec2(b))
      let min_x = if wa.x < wb.x { wa.x } else { wb.x }
      let max_x = if wa.x > wb.x { wa.x } else { wb.x }
      let min_y = if wa.y < wb.y { wa.y } else { wb.y }
      let max_y = if wa.y > wb.y { wa.y } else { wb.y }
      (
        @core.Vec2::new(min_x - pd, min_y - pd),
        @core.Vec2::new(max_x + pd, max_y + pd),
      )
    }
    Shape::Polyline(vertices, _) =>
      if vertices.length() == 0 {
        (center, center)
      } else {
        let rot = @core.Rot2::from_angle(rotation)
        let mut min_x = 1.0e30F
        let mut max_x = -1.0e30F
        let mut min_y = 1.0e30F
        let mut max_y = -1.0e30F
        for i in 0..<vertices.length() {
          let wp = center.add(rot.rotate_vec2(vertices[i]))
          if wp.x < min_x {
            min_x = wp.x
          }
          if wp.x > max_x {
            max_x = wp.x
          }
          if wp.y < min_y {
            min_y = wp.y
          }
          if wp.y > max_y {
            max_y = wp.y
          }
        }
        (
          @core.Vec2::new(min_x - pd, min_y - pd),
          @core.Vec2::new(max_x + pd, max_y + pd),
        )
      }
  }
}

///|
fn aabb_intersects_for_test(
  min1 : @core.Vec2,
  max1 : @core.Vec2,
  min2 : @core.Vec2,
  max2 : @core.Vec2,
) -> Bool {
  !(max1.x < min2.x || min1.x > max2.x || max1.y < min2.y || min1.y > max2.y)
}

///|
fn handle_less_for_test(left : ColliderHandle, right : ColliderHandle) -> Bool {
  if left.id < right.id {
    true
  } else if left.id > right.id {
    false
  } else {
    left.generation < right.generation
  }
}

///|
fn pair_less_for_test(
  left : (ColliderHandle, ColliderHandle),
  right : (ColliderHandle, ColliderHandle),
) -> Bool {
  if handle_less_for_test(left.0, right.0) {
    return true
  }
  if handle_less_for_test(right.0, left.0) {
    return false
  }
  handle_less_for_test(left.1, right.1)
}

///|
fn sort_pairs_for_test(pairs : Array[(ColliderHandle, ColliderHandle)]) -> Unit {
  for i in 1..<pairs.length() {
    let key = pairs[i]
    let mut j = i
    while j > 0 && pair_less_for_test(key, pairs[j - 1]) {
      pairs[j] = pairs[j - 1]
      j = j - 1
    }
    pairs[j] = key
  }
}

///|
test "broad phase bvh matches naive aabb pairing" {
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = ColliderSet::new()
  let handles : Array[ColliderHandle] = []
  let mut seed = 1234567
  for _ in 0..<50 {
    let (shape_roll, seed1) = rng_float01(seed)
    let (px, seed2) = rng_float01(seed1)
    let (py, seed3) = rng_float01(seed2)
    let (rot_roll, seed4) = rng_float01(seed3)
    let (enabled_roll, seed5) = rng_float01(seed4)
    seed = seed5
    let translation = @core.Vec2::new(px * 20.0F - 10.0F, py * 20.0F - 10.0F)
    let rotation = rot_roll * @core.two_pi()
    let enabled = enabled_roll > 0.1F
    let collider = if shape_roll < 0.5F {
      ColliderBuilder::ball(0.2F + 0.8F * px)
      .translation(translation)
      .rotation(rotation)
      .enabled(enabled)
      .build()
    } else {
      ColliderBuilder::cuboid(0.2F + 1.0F * px, 0.2F + 1.0F * py)
      .translation(translation)
      .rotation(rotation)
      .enabled(enabled)
      .build()
    }
    handles.push(colliders.insert(collider))
  }
  let prediction_distance = 0.05F
  let broad_phase = BroadPhaseBvh::new()
  broad_phase.update(prediction_distance, bodies, colliders)
  let bvh_pairs = broad_phase.pairs()
  let naive_pairs : Array[(ColliderHandle, ColliderHandle)] = []
  let mins : Array[@core.Vec2?] = []
  let maxs : Array[@core.Vec2?] = []
  for i in 0..<handles.length() {
    let handle = handles[i]
    if colliders.get(handle) is Some(collider) && collider.is_enabled() {
      let (min_v, max_v) = compute_aabb_for_test(
        collider.shape(),
        collider.translation(),
        collider.rotation(),
        prediction_distance,
      )
      mins.push(Some(min_v))
      maxs.push(Some(max_v))
    } else {
      mins.push(None)
      maxs.push(None)
    }
  }
  for i in 0..<mins.length() {
    if mins[i] is Some(min1) && maxs[i] is Some(max1) {
      let h1 = handles[i]
      for j in (i + 1)..<mins.length() {
        if mins[j] is Some(min2) &&
          maxs[j] is Some(max2) &&
          aabb_intersects_for_test(min1, max1, min2, max2) {
          let h2 = handles[j]
          if handle_less_for_test(h1, h2) {
            naive_pairs.push((h1, h2))
          } else {
            naive_pairs.push((h2, h1))
          }
        }
      }
    }
  }
  sort_pairs_for_test(naive_pairs)
  sort_pairs_for_test(bvh_pairs)
  inspect(bvh_pairs.length() == naive_pairs.length(), content="true")
  for i in 0..<bvh_pairs.length() {
    inspect(
      ColliderHandle::equals(bvh_pairs[i].0, naive_pairs[i].0) &&
      ColliderHandle::equals(bvh_pairs[i].1, naive_pairs[i].1),
      content="true",
    )
  }
}
