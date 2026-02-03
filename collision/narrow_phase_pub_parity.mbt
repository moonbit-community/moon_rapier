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
/// Rapier public-surface parity shims for the narrow-phase and contact-pair APIs.
///
/// Notes:
/// - This file focuses on making all Rapier `pub` items (default features) reachable
///   from this MoonBit port. Some advanced low-level fields (solver warmstart data,
///   SIMD contact layout, etc.) are currently exposed for API parity but are not
///   yet fully integrated with the solver in this port.

///|
/// Equivalent to Rapier's `geometry::Contact` (`TrackedContact<ContactData>`).
///
/// This port's 2D narrow-phase currently exposes `ContactManifoldPoint` as the
/// primary per-contact record, so we alias `Contact` to that type.
pub type Contact = ContactManifoldPoint

///|
/// Equivalent to Rapier's `geometry::contact_pair::ContactData`.
pub struct ContactData {
  impulse : @core.Real
  tangent_impulse : @core.Vec2
  warmstart_impulse : @core.Real
  warmstart_tangent_impulse : @core.Vec2
  // Rapier's dim3-only `warmstart_twist_impulse`.
  warmstart_twist_impulse : @core.Real
}

///|
pub fn ContactData::default() -> ContactData {
  {
    impulse: 0.0F,
    tangent_impulse: @core.Vec2::zero(),
    warmstart_impulse: 0.0F,
    warmstart_tangent_impulse: @core.Vec2::zero(),
    warmstart_twist_impulse: 0.0F,
  }
}

///|
/// Equivalent to Rapier's `geometry::contact_pair::SolverContactGeneric`.
///
/// This is exposed primarily for pub-surface parity. The solver warmstart fields
/// are currently not written by the narrow-phase in this port.
pub struct SolverContactGeneric {
  point : @core.Vec2
  dist : @core.Real
  friction : @core.Real
  restitution : @core.Real
  tangent_velocity : @core.Vec2
  warmstart_impulse : @core.Real
  warmstart_tangent_impulse : @core.Vec2
  warmstart_twist_impulse : @core.Real
  // Rapier uses a scalar for SIMD friendliness; we keep it as a scalar too.
  is_new : @core.Real
  // Rapier stores a fixed-size lane array; this port uses a dynamic array.
  contact_id : Array[Int]
}

///|
pub fn SolverContactGeneric::default() -> SolverContactGeneric {
  {
    point: @core.Vec2::zero(),
    dist: 0.0F,
    friction: 0.0F,
    restitution: 0.0F,
    tangent_velocity: @core.Vec2::zero(),
    warmstart_impulse: 0.0F,
    warmstart_tangent_impulse: @core.Vec2::zero(),
    warmstart_twist_impulse: 0.0F,
    is_new: 1.0F,
    contact_id: [],
  }
}

///|
/// Equivalent to Rapier's SIMD solver-contact alias.
///
/// This port does not currently implement SIMD storage; we use the same record type.
pub type SimdSolverContact = SolverContactGeneric

///|
/// Equivalent to Rapier's `geometry::contact_pair::ContactManifoldData`.
pub struct ContactManifoldData {
  solver_flags : SolverFlags
  solver_contacts : Array[SolverContactGeneric]
  relative_dominance : Int
}

///|
pub fn ContactManifoldData::default() -> ContactManifoldData {
  {
    solver_flags: SolverFlags::default(),
    solver_contacts: [],
    relative_dominance: 0,
  }
}

///|
pub fn ContactManifoldData::num_active_contacts(
  self : ContactManifoldData,
) -> Int {
  self.solver_contacts.length()
}

///|
pub fn ContactPair::has_any_active_contact(self : ContactPair) -> Bool {
  // This port currently doesn't build solver contacts, so we treat any manifold point
  // as an "active" contact.
  for i in 0..<self.manifolds.length() {
    if self.manifolds[i].points.length() > 0 {
      return true
    }
  }
  false
}

///|
pub fn ContactPair::total_impulse(self : ContactPair) -> @core.Vec2 {
  // TODO: integrate with the constraints solver to populate impulses.
  self |> ignore
  @core.Vec2::zero()
}

///|
pub fn ContactPair::total_impulse_magnitude(self : ContactPair) -> @core.Real {
  // TODO: integrate with the constraints solver to populate impulses.
  self |> ignore
  0.0F
}

///|
pub fn ContactPair::max_impulse(self : ContactPair) -> (@core.Real, @core.Vec2) {
  // TODO: integrate with the constraints solver to populate impulses.
  self |> ignore
  (0.0F, @core.Vec2::zero())
}

///|
pub fn ContactPair::find_deepest_contact(
  self : ContactPair,
) -> (ContactManifold, Contact)? {
  let mut found = false
  let mut best_m = ContactManifold::{ normal: @core.Vec2::zero(), points: [] }
  let mut best_c = ContactManifoldPoint::{
    local_p1: @core.Vec2::zero(),
    local_p2: @core.Vec2::zero(),
    dist: 0.0F,
    fid1: FeatureId::Unknown,
    fid2: FeatureId::Unknown,
  }
  let mut best_dist = 0.0F
  for mi in 0..<self.manifolds.length() {
    let m = self.manifolds[mi]
    for ci in 0..<m.points.length() {
      let c = m.points[ci]
      if !found || c.dist < best_dist {
        found = true
        best_dist = c.dist
        best_m = m
        best_c = c
      }
    }
  }
  if found {
    Some((best_m, best_c))
  } else {
    None
  }
}

///|
/// Build a contact force event from a contact pair.
///
/// Rapier's signature is `(dt, &ContactPair, total_force_magnitude)`.
/// This port's `ContactPair` does not store its collider handles, so we pass them explicitly.
pub fn ContactForceEvent::from_contact_pair(
  dt : @core.Real,
  collider1 : ColliderHandle,
  collider2 : ColliderHandle,
  pair : ContactPair,
  total_force_magnitude : @core.Real,
) -> ContactForceEvent {
  // TODO: convert impulses to forces once impulses are available in this port.
  dt |> ignore
  pair |> ignore
  ContactForceEvent::new(
    collider1,
    collider2,
    @core.Vec2::zero(),
    total_force_magnitude,
    @core.Vec2::zero(),
    0.0F,
  )
}

///|
/// Equivalent broad-phase type alias for Rapier's `DefaultBroadPhase`.
pub type DefaultBroadPhase = BroadPhaseBvh

///|
/// Index types used by Rapier's interaction graphs.
pub type ColliderGraphIndex = Int

///|
pub type RigidBodyGraphIndex = Int

///|
pub type TemporaryInteractionIndex = Int

///|
/// Minimal interaction-graph surface for pub parity with Rapier.
pub struct InteractionGraph[T] {
  interactions : Array[(ColliderHandle, ColliderHandle, T)]
}

///|
pub fn[T] InteractionGraph::new() -> InteractionGraph[T] {
  { interactions: [] }
}

///|
pub fn[T] InteractionGraph::raw_graph(
  self : InteractionGraph[T],
) -> Array[(ColliderHandle, ColliderHandle, T)] {
  let out : Array[(ColliderHandle, ColliderHandle, T)] = []
  for i in 0..<self.interactions.length() {
    out.push(self.interactions[i])
  }
  out
}

///|
pub fn[T] InteractionGraph::interactions(
  self : InteractionGraph[T],
) -> Array[(ColliderHandle, ColliderHandle, T)] {
  self.raw_graph()
}

///|
pub fn[T] InteractionGraph::index_interaction(
  self : InteractionGraph[T],
  a : ColliderHandle,
  b : ColliderHandle,
) -> Int? {
  for i in 0..<self.interactions.length() {
    let (c1, c2, _) = self.interactions[i]
    if (collider_handle_equals(c1, a) && collider_handle_equals(c2, b)) ||
      (collider_handle_equals(c1, b) && collider_handle_equals(c2, a)) {
      return Some(i)
    }
  }
  None
}

///|
pub fn[T] InteractionGraph::interaction_pair(
  self : InteractionGraph[T],
  a : ColliderHandle,
  b : ColliderHandle,
) -> T? {
  match self.index_interaction(a, b) {
    None => None
    Some(i) => Some(self.interactions[i].2)
  }
}

///|
/// MoonBit does not currently expose `&mut` references; this returns a copy for parity.
pub fn[T] InteractionGraph::interaction_pair_mut(
  self : InteractionGraph[T],
  a : ColliderHandle,
  b : ColliderHandle,
) -> T? {
  self.interaction_pair(a, b)
}

///|
pub fn[T] InteractionGraph::interactions_with(
  self : InteractionGraph[T],
  collider : ColliderHandle,
) -> Array[(ColliderHandle, ColliderHandle, T)] {
  let out : Array[(ColliderHandle, ColliderHandle, T)] = []
  for i in 0..<self.interactions.length() {
    let it = self.interactions[i]
    if collider_handle_equals(it.0, collider) ||
      collider_handle_equals(it.1, collider) {
      out.push(it)
    }
  }
  out
}

///|
pub fn[T] InteractionGraph::interactions_with_endpoints(
  self : InteractionGraph[T],
  collider1 : ColliderHandle,
  collider2 : ColliderHandle,
) -> Array[(ColliderHandle, ColliderHandle, T)] {
  let out : Array[(ColliderHandle, ColliderHandle, T)] = []
  for i in 0..<self.interactions.length() {
    let it = self.interactions[i]
    if (
        collider_handle_equals(it.0, collider1) &&
        collider_handle_equals(it.1, collider2)
      ) ||
      (
        collider_handle_equals(it.0, collider2) &&
        collider_handle_equals(it.1, collider1)
      ) {
      out.push(it)
    }
  }
  out
}

///|
pub fn[T] InteractionGraph::interactions_with_mut(
  self : InteractionGraph[T],
  collider : ColliderHandle,
) -> Array[(ColliderHandle, ColliderHandle, T)] {
  // See `interaction_pair_mut` note.
  self.interactions_with(collider)
}

///|
fn handle_id_equals(a : ColliderHandle, b : ColliderHandle) -> Bool {
  a.id == b.id
}

///|
pub fn NarrowPhase::contact_graph(
  self : NarrowPhase,
) -> InteractionGraph[ContactPair] {
  { interactions: self.contacts }
}

///|
pub fn NarrowPhase::intersection_graph(
  self : NarrowPhase,
) -> InteractionGraph[IntersectionPair] {
  let out : Array[(ColliderHandle, ColliderHandle, IntersectionPair)] = []
  for i in 0..<self.intersections.length() {
    let p = self.intersections[i]
    out.push((p.collider1, p.collider2, p))
  }
  { interactions: out }
}

///|
pub fn NarrowPhase::contact_pair_at_index(
  self : NarrowPhase,
  index : Int,
) -> (ColliderHandle, ColliderHandle, ContactPair)? {
  if index < 0 || index >= self.contacts.length() {
    None
  } else {
    Some(self.contacts[index])
  }
}

///|
pub fn NarrowPhase::contact_pair_unknown_gen(
  self : NarrowPhase,
  first : ColliderHandle,
  second : ColliderHandle,
) -> ContactPair? {
  for i in 0..<self.contacts.length() {
    let pair = self.contacts[i]
    let matches = handle_id_equals(pair.0, first) &&
      handle_id_equals(pair.1, second)
    let matches_swapped = handle_id_equals(pair.0, second) &&
      handle_id_equals(pair.1, first)
    if matches || matches_swapped {
      return Some(pair.2)
    }
  }
  None
}

///|
pub fn NarrowPhase::contact_pairs_with_unknown_gen(
  self : NarrowPhase,
  collider : ColliderHandle,
) -> Array[(ColliderHandle, ColliderHandle, ContactPair)] {
  let out : Array[(ColliderHandle, ColliderHandle, ContactPair)] = []
  for i in 0..<self.contacts.length() {
    let pair = self.contacts[i]
    if handle_id_equals(pair.0, collider) || handle_id_equals(pair.1, collider) {
      out.push(pair)
    }
  }
  out
}

///|
pub fn NarrowPhase::intersection_pair_unknown_gen(
  self : NarrowPhase,
  first : ColliderHandle,
  second : ColliderHandle,
) -> IntersectionPair? {
  for i in 0..<self.intersections.length() {
    let pair = self.intersections[i]
    let matches = handle_id_equals(pair.collider1, first) &&
      handle_id_equals(pair.collider2, second)
    let matches_swapped = handle_id_equals(pair.collider1, second) &&
      handle_id_equals(pair.collider2, first)
    if matches || matches_swapped {
      return Some(pair)
    }
  }
  None
}

///|
pub fn NarrowPhase::intersection_pairs_with_unknown_gen(
  self : NarrowPhase,
  collider : ColliderHandle,
) -> Array[IntersectionPair] {
  let out : Array[IntersectionPair] = []
  for i in 0..<self.intersections.length() {
    let pair = self.intersections[i]
    if handle_id_equals(pair.collider1, collider) ||
      handle_id_equals(pair.collider2, collider) {
      out.push(pair)
    }
  }
  out
}

///|
pub fn NarrowPhase::handle_user_changes(
  self : NarrowPhase,
  bodies : @dynamics.RigidBodySet,
  colliders : ColliderSet,
  modified_colliders : Array[ColliderHandle],
) -> Unit {
  handle_user_changes_to_colliders(bodies, colliders, modified_colliders)
  // This simplified narrow-phase rebuilds pairs on `update`, so we don't need to
  // invalidate any persistent caches here yet.
  self |> ignore
}

///|
pub fn NarrowPhase::query_dispatcher(self : NarrowPhase) -> Unit {
  // Placeholder for pub parity with Rapier's dispatcher-based narrow-phase.
  self |> ignore
}

///|
pub fn NarrowPhase::with_query_dispatcher(
  self : NarrowPhase,
  _dispatcher : Unit,
) -> NarrowPhase {
  // Placeholder for pub parity with Rapier's dispatcher-based narrow-phase.
  self
}
