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
#declaration_only
pub type ColliderHandle

///|
#declaration_only
pub fn ColliderHandle::equals(
  self : ColliderHandle,
  other : ColliderHandle,
) -> Bool {
  ...
}

///|
#declaration_only
pub fn ColliderHandle::from_raw_parts(
  id : Int,
  generation : Int,
) -> ColliderHandle {
  ...
}

///|
#declaration_only
pub fn ColliderHandle::into_raw_parts(self : ColliderHandle) -> (Int, Int) {
  ...
}

///|
#declaration_only
pub fn ColliderHandle::invalid() -> ColliderHandle {
  ...
}

///|
#declaration_only
pub type ActiveCollisionTypes

///|
#declaration_only
pub fn ActiveCollisionTypes::all() -> ActiveCollisionTypes {
  ...
}

///|
#declaration_only
pub fn ActiveCollisionTypes::default() -> ActiveCollisionTypes {
  ...
}

///|
#declaration_only
pub fn ActiveCollisionTypes::equals(
  self : ActiveCollisionTypes,
  other : ActiveCollisionTypes,
) -> Bool {
  ...
}

///|
#declaration_only
pub fn ActiveCollisionTypes::test_body_types(
  self : ActiveCollisionTypes,
  body_type1 : @dynamics.RigidBodyType,
  body_type2 : @dynamics.RigidBodyType,
) -> Bool {
  ...
}

///|
#declaration_only
pub type ActiveEvents

///|
#declaration_only
pub fn ActiveEvents::empty() -> ActiveEvents {
  ...
}

///|
#declaration_only
pub fn ActiveEvents::collision_events() -> ActiveEvents {
  ...
}

///|
#declaration_only
pub fn ActiveEvents::contact_force_events() -> ActiveEvents {
  ...
}

///|
#declaration_only
pub fn ActiveEvents::intersection_events() -> ActiveEvents {
  ...
}

///|
#declaration_only
pub fn ActiveEvents::contains(
  self : ActiveEvents,
  other : ActiveEvents,
) -> Bool {
  ...
}

///|
#declaration_only
pub fn ActiveEvents::insert(
  self : ActiveEvents,
  other : ActiveEvents,
) -> ActiveEvents {
  ...
}

///|
#declaration_only
pub type CollisionEventFlags

///|
#declaration_only
pub fn CollisionEventFlags::empty() -> CollisionEventFlags {
  ...
}

///|
#declaration_only
pub fn CollisionEventFlags::sensor() -> CollisionEventFlags {
  ...
}

///|
#declaration_only
pub fn CollisionEventFlags::removed() -> CollisionEventFlags {
  ...
}

///|
#declaration_only
pub fn CollisionEventFlags::contains(
  self : CollisionEventFlags,
  other : CollisionEventFlags,
) -> Bool {
  ...
}

///|
#declaration_only
pub fn CollisionEventFlags::insert(
  self : CollisionEventFlags,
  other : CollisionEventFlags,
) -> CollisionEventFlags {
  ...
}

///|
pub(all) enum CollisionEvent {
  Started(ColliderHandle, ColliderHandle, CollisionEventFlags)
  Stopped(ColliderHandle, ColliderHandle, CollisionEventFlags)
}

///|
#declaration_only
pub fn CollisionEvent::started(self : CollisionEvent) -> Bool {
  ...
}

///|
#declaration_only
pub fn CollisionEvent::stopped(self : CollisionEvent) -> Bool {
  ...
}

///|
#declaration_only
pub fn CollisionEvent::collider1(self : CollisionEvent) -> ColliderHandle {
  ...
}

///|
#declaration_only
pub fn CollisionEvent::collider2(self : CollisionEvent) -> ColliderHandle {
  ...
}

///|
#declaration_only
pub fn CollisionEvent::sensor(self : CollisionEvent) -> Bool {
  ...
}

///|
#declaration_only
pub fn CollisionEvent::removed(self : CollisionEvent) -> Bool {
  ...
}

///|
#declaration_only
pub type IntersectionEvent

///|
#declaration_only
pub fn IntersectionEvent::new(
  collider1 : ColliderHandle,
  collider2 : ColliderHandle,
  intersecting : Bool,
) -> IntersectionEvent {
  ...
}

///|
#declaration_only
pub fn IntersectionEvent::collider1(self : IntersectionEvent) -> ColliderHandle {
  ...
}

///|
#declaration_only
pub fn IntersectionEvent::collider2(self : IntersectionEvent) -> ColliderHandle {
  ...
}

///|
#declaration_only
pub fn IntersectionEvent::intersecting(self : IntersectionEvent) -> Bool {
  ...
}

///|
#declaration_only
pub type ContactForceEvent

///|
#declaration_only
pub fn ContactForceEvent::new(
  collider1 : ColliderHandle,
  collider2 : ColliderHandle,
  total_force : @core.Vec2,
  total_force_magnitude : @core.Real,
  max_force_direction : @core.Vec2,
  max_force_magnitude : @core.Real,
) -> ContactForceEvent {
  ...
}

///|
#declaration_only
pub fn ContactForceEvent::collider1(self : ContactForceEvent) -> ColliderHandle {
  ...
}

///|
#declaration_only
pub fn ContactForceEvent::collider2(self : ContactForceEvent) -> ColliderHandle {
  ...
}

///|
#declaration_only
pub fn ContactForceEvent::total_force(self : ContactForceEvent) -> @core.Vec2 {
  ...
}

///|
#declaration_only
pub fn ContactForceEvent::total_force_magnitude(
  self : ContactForceEvent,
) -> @core.Real {
  ...
}

///|
#declaration_only
pub fn ContactForceEvent::max_force_direction(
  self : ContactForceEvent,
) -> @core.Vec2 {
  ...
}

///|
#declaration_only
pub fn ContactForceEvent::max_force_magnitude(
  self : ContactForceEvent,
) -> @core.Real {
  ...
}

///|
#declaration_only
pub type ActiveHooks

///|
#declaration_only
pub fn ActiveHooks::empty() -> ActiveHooks {
  ...
}

///|
#declaration_only
pub fn ActiveHooks::all() -> ActiveHooks {
  ...
}

///|
#declaration_only
pub fn ActiveHooks::filter_contact_pairs() -> ActiveHooks {
  ...
}

///|
#declaration_only
pub fn ActiveHooks::filter_intersection_pair() -> ActiveHooks {
  ...
}

///|
#declaration_only
pub fn ActiveHooks::modify_solver_contacts() -> ActiveHooks {
  ...
}

///|
#declaration_only
pub fn ActiveHooks::contains(self : ActiveHooks, other : ActiveHooks) -> Bool {
  ...
}

///|
#declaration_only
pub fn ActiveHooks::insert(
  self : ActiveHooks,
  other : ActiveHooks,
) -> ActiveHooks {
  ...
}

///|
pub(all) enum ColliderEnabled {
  Enabled
  DisabledByParent
  Disabled
}

///|
#declaration_only
pub type Collider

///|
#declaration_only
pub type Shape

///|
#declaration_only
pub type SharedShape

///|
#declaration_only
pub fn SharedShape::new(shape : Shape) -> SharedShape {
  ...
}

///|
#declaration_only
pub fn SharedShape::as_shape(self : SharedShape) -> Shape {
  ...
}

///|
#declaration_only
pub fn SharedShape::ball(radius : @core.Real) -> SharedShape {
  ...
}

///|
#declaration_only
pub fn SharedShape::cuboid(
  half_width : @core.Real,
  half_height : @core.Real,
) -> SharedShape {
  ...
}

///|
#declaration_only
pub fn SharedShape::capsule_x(
  half_height : @core.Real,
  radius : @core.Real,
) -> SharedShape {
  ...
}

///|
#declaration_only
pub fn SharedShape::capsule_y(
  half_height : @core.Real,
  radius : @core.Real,
) -> SharedShape {
  ...
}

///|
#declaration_only
pub fn SharedShape::segment(a : @core.Vec2, b : @core.Vec2) -> SharedShape {
  ...
}

///|
#declaration_only
pub fn SharedShape::triangle(
  a : @core.Vec2,
  b : @core.Vec2,
  c : @core.Vec2,
) -> SharedShape {
  ...
}

///|
#declaration_only
pub fn SharedShape::polyline(
  vertices : Array[@core.Vec2],
  indices : Array[(Int, Int)]?,
) -> SharedShape {
  ...
}

///|
#declaration_only
pub fn SharedShape::trimesh(
  vertices : Array[@core.Vec2],
  indices : Array[(Int, Int, Int)],
) -> SharedShape? {
  ...
}

///|
#declaration_only
pub fn SharedShape::heightfield(
  heights : Array[@core.Real],
  scale : @core.Vec2,
) -> SharedShape {
  ...
}

///|
#declaration_only
pub type Triangle

///|
#declaration_only
pub fn Triangle::new(
  a : @core.Vec2,
  b : @core.Vec2,
  c : @core.Vec2,
) -> Triangle {
  ...
}

///|
#declaration_only
pub fn Triangle::as_shape(self : Triangle) -> Shape {
  ...
}

///|
#declaration_only
pub type HeightField

///|
#declaration_only
pub fn HeightField::new(
  heights : Array[@core.Real],
  scale : @core.Vec2,
) -> HeightField {
  ...
}

///|
#declaration_only
pub fn HeightField::as_shape(self : HeightField) -> Shape {
  ...
}

///|
#declaration_only
pub type MeshConverterError

///|
#declaration_only
pub fn MeshConverterError::to_string(self : MeshConverterError) -> String {
  ...
}

///|
#declaration_only
pub type MeshConverter

///|
#declaration_only
pub fn MeshConverter::trimesh() -> MeshConverter {
  ...
}

///|
#declaration_only
pub fn MeshConverter::aabb() -> MeshConverter {
  ...
}

///|
#declaration_only
pub fn MeshConverter::obb() -> MeshConverter {
  ...
}

///|
#declaration_only
pub fn MeshConverter::convex_hull() -> MeshConverter {
  ...
}

///|
#declaration_only
pub fn MeshConverter::apply(
  self : MeshConverter,
  vertices : Array[@core.Vec2],
  indices : Array[(Int, Int, Int)],
) -> Result[(SharedShape, @core.Isometry2), MeshConverterError] {
  ...
}

///|
#declaration_only
pub type ColliderPair

///|
#declaration_only
pub fn ColliderPair::new(
  a : ColliderHandle,
  b : ColliderHandle,
) -> ColliderPair {
  ...
}

///|
#declaration_only
pub fn ColliderPair::collider1(self : ColliderPair) -> ColliderHandle {
  ...
}

///|
#declaration_only
pub fn ColliderPair::collider2(self : ColliderPair) -> ColliderHandle {
  ...
}

///|
#declaration_only
pub type BroadPhasePairEvent

///|
#declaration_only
pub type BvhOptimizationStrategy

///|
#declaration_only
pub fn BvhOptimizationStrategy::none() -> BvhOptimizationStrategy {
  ...
}

///|
#declaration_only
pub fn BvhOptimizationStrategy::subtree_optimizer() -> BvhOptimizationStrategy {
  ...
}

///|
#declaration_only
pub type ColliderBuilder

///|
#declaration_only
pub fn ColliderBuilder::ball(radius : @core.Real) -> ColliderBuilder {
  ...
}

///|
#declaration_only
pub fn ColliderBuilder::cuboid(
  half_width : @core.Real,
  half_height : @core.Real,
) -> ColliderBuilder {
  ...
}

///|
#declaration_only
pub fn ColliderBuilder::capsule_x(
  half_height : @core.Real,
  radius : @core.Real,
) -> ColliderBuilder {
  ...
}

///|
#declaration_only
pub fn ColliderBuilder::capsule_y(
  half_height : @core.Real,
  radius : @core.Real,
) -> ColliderBuilder {
  ...
}

///|
#declaration_only
pub fn ColliderBuilder::capsule_from_endpoints(
  a : @core.Vec2,
  b : @core.Vec2,
  radius : @core.Real,
) -> ColliderBuilder {
  ...
}

///|
#declaration_only
pub fn ColliderBuilder::triangle(
  a : @core.Vec2,
  b : @core.Vec2,
  c : @core.Vec2,
) -> ColliderBuilder {
  ...
}

///|
#declaration_only
pub fn ColliderBuilder::round_triangle(
  a : @core.Vec2,
  b : @core.Vec2,
  c : @core.Vec2,
  border_radius : @core.Real,
) -> ColliderBuilder {
  ...
}

///|
#declaration_only
pub fn ColliderBuilder::halfspace(
  outward_normal : @core.Vec2,
) -> ColliderBuilder {
  ...
}

///|
#declaration_only
pub fn ColliderBuilder::compound(
  shapes : Array[(@core.Isometry2, Shape)],
) -> ColliderBuilder {
  ...
}

///|
#declaration_only
pub fn ColliderBuilder::round_convex_hull(
  points : Array[@core.Vec2],
  border_radius : @core.Real,
) -> ColliderBuilder? {
  ...
}

///|
#declaration_only
pub fn ColliderBuilder::segment(
  a : @core.Vec2,
  b : @core.Vec2,
) -> ColliderBuilder {
  ...
}

///|
#declaration_only
pub fn ColliderBuilder::polyline(
  vertices : Array[@core.Vec2],
  indices : Array[(Int, Int)]?,
) -> ColliderBuilder {
  ...
}

///|
#declaration_only
pub fn ColliderBuilder::heightfield(
  heights : Array[@core.Real],
  scale : @core.Vec2,
) -> ColliderBuilder {
  ...
}

///|
#declaration_only
pub fn ColliderBuilder::trimesh(
  vertices : Array[@core.Vec2],
  indices : Array[(Int, Int, Int)],
) -> ColliderBuilder? {
  ...
}

///|
#declaration_only
pub fn ColliderBuilder::convex_hull(
  points : Array[@core.Vec2],
) -> ColliderBuilder? {
  ...
}

///|
#declaration_only
pub fn ColliderBuilder::translation(
  self : ColliderBuilder,
  translation : @core.Vec2,
) -> ColliderBuilder {
  ...
}

///|
#declaration_only
pub fn ColliderBuilder::rotation(
  self : ColliderBuilder,
  rotation : @core.Real,
) -> ColliderBuilder {
  ...
}

///|
#declaration_only
pub fn ColliderBuilder::active_collision_types(
  self : ColliderBuilder,
  types : ActiveCollisionTypes,
) -> ColliderBuilder {
  ...
}

///|
#declaration_only
pub fn ColliderBuilder::sensor(
  self : ColliderBuilder,
  enabled : Bool,
) -> ColliderBuilder {
  ...
}

///|
#declaration_only
pub fn ColliderBuilder::active_events(
  self : ColliderBuilder,
  events : ActiveEvents,
) -> ColliderBuilder {
  ...
}

///|
#declaration_only
pub fn ColliderBuilder::active_hooks(
  self : ColliderBuilder,
  hooks : ActiveHooks,
) -> ColliderBuilder {
  ...
}

///|
#declaration_only
pub fn ColliderBuilder::collision_groups(
  self : ColliderBuilder,
  groups : @dynamics.InteractionGroups,
) -> ColliderBuilder {
  ...
}

///|
#declaration_only
pub fn ColliderBuilder::solver_groups(
  self : ColliderBuilder,
  groups : @dynamics.InteractionGroups,
) -> ColliderBuilder {
  ...
}

///|
#declaration_only
pub fn ColliderBuilder::density(
  self : ColliderBuilder,
  density : @core.Real,
) -> ColliderBuilder {
  ...
}

///|
#declaration_only
pub fn ColliderBuilder::friction(
  self : ColliderBuilder,
  friction : @core.Real,
) -> ColliderBuilder {
  ...
}

///|
#declaration_only
pub fn ColliderBuilder::friction_combine_rule(
  self : ColliderBuilder,
  rule : @dynamics.CoefficientCombineRule,
) -> ColliderBuilder {
  ...
}

///|
#declaration_only
pub fn ColliderBuilder::restitution(
  self : ColliderBuilder,
  value : @core.Real,
) -> ColliderBuilder {
  ...
}

///|
#declaration_only
pub fn ColliderBuilder::restitution_combine_rule(
  self : ColliderBuilder,
  rule : @dynamics.CoefficientCombineRule,
) -> ColliderBuilder {
  ...
}

///|
#declaration_only
pub fn ColliderBuilder::contact_skin(
  self : ColliderBuilder,
  skin : @core.Real,
) -> ColliderBuilder {
  ...
}

///|
#declaration_only
pub fn ColliderBuilder::contact_force_event_threshold(
  self : ColliderBuilder,
  threshold : @core.Real,
) -> ColliderBuilder {
  ...
}

///|
#declaration_only
pub fn ColliderBuilder::user_data(
  self : ColliderBuilder,
  data : Int,
) -> ColliderBuilder {
  ...
}

///|
#declaration_only
pub fn ColliderBuilder::user_data128(
  self : ColliderBuilder,
  data : @core.UserData128,
) -> ColliderBuilder {
  ...
}

///|
#declaration_only
pub fn ColliderBuilder::enabled(
  self : ColliderBuilder,
  enabled : Bool,
) -> ColliderBuilder {
  ...
}

///|
#declaration_only
pub fn ColliderBuilder::build(self : ColliderBuilder) -> Collider {
  ...
}

///|
#declaration_only
pub fn Collider::shape(self : Collider) -> Shape {
  ...
}

///|
#declaration_only
pub fn Collider::set_shape(self : Collider, shape : Shape) -> Collider {
  ...
}

///|
#declaration_only
pub fn Collider::translation(self : Collider) -> @core.Vec2 {
  ...
}

///|
#declaration_only
pub fn Collider::position(self : Collider) -> @core.Isometry2 {
  ...
}

///|
#declaration_only
pub fn Collider::rotation(self : Collider) -> @core.Real {
  ...
}

///|
#declaration_only
pub fn Collider::position_wrt_parent(self : Collider) -> @core.Isometry2? {
  ...
}

///|
#declaration_only
pub fn Collider::parent(self : Collider) -> @dynamics.RigidBodyHandle? {
  ...
}

///|
#declaration_only
pub fn Collider::is_sensor(self : Collider) -> Bool {
  ...
}

///|
#declaration_only
pub fn Collider::active_collision_types(
  self : Collider,
) -> ActiveCollisionTypes {
  ...
}

///|
#declaration_only
pub fn Collider::active_events(self : Collider) -> ActiveEvents {
  ...
}

///|
#declaration_only
pub fn Collider::active_hooks(self : Collider) -> ActiveHooks {
  ...
}

///|
#declaration_only
pub fn Collider::collision_groups(
  self : Collider,
) -> @dynamics.InteractionGroups {
  ...
}

///|
#declaration_only
pub fn Collider::solver_groups(self : Collider) -> @dynamics.InteractionGroups {
  ...
}

///|
#declaration_only
pub fn Collider::friction(self : Collider) -> @core.Real {
  ...
}

///|
#declaration_only
pub fn Collider::friction_combine_rule(
  self : Collider,
) -> @dynamics.CoefficientCombineRule {
  ...
}

///|
#declaration_only
pub fn Collider::restitution(self : Collider) -> @core.Real {
  ...
}

///|
#declaration_only
pub fn Collider::restitution_combine_rule(
  self : Collider,
) -> @dynamics.CoefficientCombineRule {
  ...
}

///|
#declaration_only
pub fn Collider::contact_skin(self : Collider) -> @core.Real {
  ...
}

///|
#declaration_only
pub fn Collider::contact_force_event_threshold(self : Collider) -> @core.Real {
  ...
}

///|
#declaration_only
pub fn Collider::user_data(self : Collider) -> Int {
  ...
}

///|
#declaration_only
pub fn Collider::user_data128(self : Collider) -> @core.UserData128 {
  ...
}

///|
#declaration_only
pub fn Collider::is_enabled(self : Collider) -> Bool {
  ...
}

///|
#declaration_only
pub fn Collider::set_sensor(self : Collider, enabled : Bool) -> Collider {
  ...
}

///|
#declaration_only
pub fn Collider::set_active_collision_types(
  self : Collider,
  types : ActiveCollisionTypes,
) -> Collider {
  ...
}

///|
#declaration_only
pub fn Collider::set_active_events(
  self : Collider,
  events : ActiveEvents,
) -> Collider {
  ...
}

///|
#declaration_only
pub fn Collider::set_active_hooks(
  self : Collider,
  hooks : ActiveHooks,
) -> Collider {
  ...
}

///|
#declaration_only
pub fn Collider::set_collision_groups(
  self : Collider,
  groups : @dynamics.InteractionGroups,
) -> Collider {
  ...
}

///|
#declaration_only
pub fn Collider::set_solver_groups(
  self : Collider,
  groups : @dynamics.InteractionGroups,
) -> Collider {
  ...
}

///|
#declaration_only
pub fn Collider::set_friction(
  self : Collider,
  friction : @core.Real,
) -> Collider {
  ...
}

///|
#declaration_only
pub fn Collider::set_friction_combine_rule(
  self : Collider,
  rule : @dynamics.CoefficientCombineRule,
) -> Collider {
  ...
}

///|
#declaration_only
pub fn Collider::set_restitution(
  self : Collider,
  restitution : @core.Real,
) -> Collider {
  ...
}

///|
#declaration_only
pub fn Collider::set_restitution_combine_rule(
  self : Collider,
  rule : @dynamics.CoefficientCombineRule,
) -> Collider {
  ...
}

///|
#declaration_only
pub fn Collider::set_contact_skin(
  self : Collider,
  skin : @core.Real,
) -> Collider {
  ...
}

///|
#declaration_only
pub fn Collider::set_contact_force_event_threshold(
  self : Collider,
  threshold : @core.Real,
) -> Collider {
  ...
}

///|
#declaration_only
pub fn Collider::set_user_data(self : Collider, data : Int) -> Collider {
  ...
}

///|
#declaration_only
pub fn Collider::set_user_data128(
  self : Collider,
  data : @core.UserData128,
) -> Collider {
  ...
}

///|
#declaration_only
pub fn Collider::set_translation(
  self : Collider,
  translation : @core.Vec2,
) -> Collider {
  ...
}

///|
#declaration_only
pub fn Collider::set_rotation(
  self : Collider,
  rotation : @core.Real,
) -> Collider {
  ...
}

///|
#declaration_only
pub fn Collider::set_position(
  self : Collider,
  position : @core.Isometry2,
) -> Collider {
  ...
}

///|
#declaration_only
pub fn Collider::set_translation_wrt_parent(
  self : Collider,
  translation : @core.Vec2,
) -> Collider {
  ...
}

///|
#declaration_only
pub fn Collider::set_rotation_wrt_parent(
  self : Collider,
  rotation : @core.Real,
) -> Collider {
  ...
}

///|
#declaration_only
pub fn Collider::set_enabled(self : Collider, enabled : Bool) -> Collider {
  ...
}

///|
#declaration_only
pub type ColliderSet

///|
#declaration_only
pub fn ColliderSet::new() -> ColliderSet {
  ...
}

///|
#declaration_only
pub fn ColliderSet::serialize(self : ColliderSet) -> String {
  ...
}

///|
#declaration_only
pub fn ColliderSet::deserialize(data : String) -> ColliderSet {
  ...
}

///|
#declaration_only
pub fn ColliderSet::len(self : ColliderSet) -> Int {
  ...
}

///|
#declaration_only
pub fn ColliderSet::is_empty(self : ColliderSet) -> Bool {
  ...
}

///|
#declaration_only
pub fn ColliderSet::insert(
  self : ColliderSet,
  collider : Collider,
) -> ColliderHandle {
  ...
}

///|
#declaration_only
pub fn ColliderSet::colliders_with_parent(
  self : ColliderSet,
  parent : @dynamics.RigidBodyHandle,
) -> Array[ColliderHandle] {
  ...
}

///|
#declaration_only
pub fn ColliderSet::insert_with_parent(
  self : ColliderSet,
  collider : Collider,
  parent : @dynamics.RigidBodyHandle,
  bodies : @dynamics.RigidBodySet,
) -> ColliderHandle {
  ...
}

///|
#declaration_only
pub fn ColliderSet::get(
  self : ColliderSet,
  handle : ColliderHandle,
) -> Collider? {
  ...
}

///|
#declaration_only
pub fn ColliderSet::get_mut(
  self : ColliderSet,
  handle : ColliderHandle,
) -> Collider? {
  ...
}

///|
#declaration_only
pub fn ColliderSet::remove(
  self : ColliderSet,
  handle : ColliderHandle,
  islands : @dynamics.IslandManager,
  bodies : @dynamics.RigidBodySet,
  wake_up : Bool,
) -> Unit {
  ...
}

///|
#declaration_only
pub fn ColliderSet::set_parent(
  self : ColliderSet,
  handle : ColliderHandle,
  parent : @dynamics.RigidBodyHandle?,
  bodies : @dynamics.RigidBodySet,
) -> Unit {
  ...
}

///|
#declaration_only
pub fn ColliderSet::set_parent_enabled(
  self : ColliderSet,
  parent : @dynamics.RigidBodyHandle,
  enabled : Bool,
) -> Unit {
  ...
}

///|
#declaration_only
pub fn ColliderSet::take_modified(self : ColliderSet) -> Array[ColliderHandle] {
  ...
}

///|
#declaration_only
pub fn ColliderSet::clear_changes_for(
  self : ColliderSet,
  handles : Array[ColliderHandle],
) -> Unit {
  ...
}

///|
#declaration_only
pub fn handle_user_changes_to_colliders(
  bodies : @dynamics.RigidBodySet,
  colliders : ColliderSet,
  modified_colliders : Array[ColliderHandle],
) -> Unit {
  ...
}

///|
pub struct IntersectionPair {
  collider1 : ColliderHandle
  collider2 : ColliderHandle
  intersecting : Bool
}

///|
#declaration_only
pub type ContactPair

///|
#declaration_only
pub fn ContactPair::manifold_count(self : ContactPair) -> Int {
  ...
}

///|
#declaration_only
pub fn ContactPair::manifolds(self : ContactPair) -> Array[ContactManifold] {
  ...
}

///|
#declaration_only
pub type ContactManifold

///|
#declaration_only
pub fn ContactManifold::normal(self : ContactManifold) -> @core.Vec2 {
  ...
}

///|
#declaration_only
pub fn ContactManifold::points(
  self : ContactManifold,
) -> Array[ContactManifoldPoint] {
  ...
}

///|
#declaration_only
pub type SolverFlags

///|
#declaration_only
pub fn SolverFlags::empty() -> SolverFlags {
  ...
}

///|
#declaration_only
pub fn SolverFlags::compute_impulses() -> SolverFlags {
  ...
}

///|
#declaration_only
pub fn SolverFlags::default() -> SolverFlags {
  ...
}

///|
#declaration_only
pub fn SolverFlags::contains(self : SolverFlags, other : SolverFlags) -> Bool {
  ...
}

///|
#declaration_only
pub fn SolverFlags::insert(
  self : SolverFlags,
  other : SolverFlags,
) -> SolverFlags {
  ...
}

///|
#declaration_only
pub fn SolverFlags::remove(
  self : SolverFlags,
  other : SolverFlags,
) -> SolverFlags {
  ...
}

///|
#declaration_only
pub type SolverContact

///|
#declaration_only
pub fn SolverContact::new(
  point : @core.Vec2,
  dist : @core.Real,
) -> SolverContact {
  ...
}

///|
#declaration_only
pub fn SolverContact::point(self : SolverContact) -> @core.Vec2 {
  ...
}

///|
#declaration_only
pub fn SolverContact::set_point(
  self : SolverContact,
  point : @core.Vec2,
) -> Unit {
  ...
}

///|
#declaration_only
pub fn SolverContact::dist(self : SolverContact) -> @core.Real {
  ...
}

///|
#declaration_only
pub fn SolverContact::set_dist(self : SolverContact, dist : @core.Real) -> Unit {
  ...
}

///|
#declaration_only
pub fn SolverContact::friction(self : SolverContact) -> @core.Real {
  ...
}

///|
#declaration_only
pub fn SolverContact::set_friction(
  self : SolverContact,
  friction : @core.Real,
) -> Unit {
  ...
}

///|
#declaration_only
pub fn SolverContact::restitution(self : SolverContact) -> @core.Real {
  ...
}

///|
#declaration_only
pub fn SolverContact::set_restitution(
  self : SolverContact,
  restitution : @core.Real,
) -> Unit {
  ...
}

///|
#declaration_only
pub fn SolverContact::tangent_velocity(self : SolverContact) -> @core.Vec2 {
  ...
}

///|
#declaration_only
pub fn SolverContact::set_tangent_velocity(
  self : SolverContact,
  tangent_velocity : @core.Vec2,
) -> Unit {
  ...
}

///|
#declaration_only
pub type ContactManifoldPoint

///|
#declaration_only
pub fn ContactManifoldPoint::local_p1(
  self : ContactManifoldPoint,
) -> @core.Vec2 {
  ...
}

///|
#declaration_only
pub fn ContactManifoldPoint::local_p2(
  self : ContactManifoldPoint,
) -> @core.Vec2 {
  ...
}

///|
#declaration_only
pub fn ContactManifoldPoint::dist(self : ContactManifoldPoint) -> @core.Real {
  ...
}

///|
#declaration_only
pub fn ContactManifoldPoint::fid1(self : ContactManifoldPoint) -> FeatureId {
  ...
}

///|
#declaration_only
pub fn ContactManifoldPoint::fid2(self : ContactManifoldPoint) -> FeatureId {
  ...
}

///|
#declaration_only
pub type NarrowPhase

///|
#declaration_only
pub fn NarrowPhase::new() -> NarrowPhase {
  ...
}

///|
#declaration_only
pub fn NarrowPhase::intersection_pairs(
  self : NarrowPhase,
) -> Array[IntersectionPair] {
  ...
}

///|
#declaration_only
pub fn NarrowPhase::contact_pairs(
  self : NarrowPhase,
) -> Array[(ColliderHandle, ColliderHandle, ContactPair)] {
  ...
}

///|
#declaration_only
pub fn NarrowPhase::intersection_pairs_with(
  self : NarrowPhase,
  handle : ColliderHandle,
) -> Array[IntersectionPair] {
  ...
}

///|
#declaration_only
pub fn NarrowPhase::contact_pairs_with(
  self : NarrowPhase,
  handle : ColliderHandle,
) -> Array[(ColliderHandle, ColliderHandle, ContactPair)] {
  ...
}

///|
#declaration_only
pub fn NarrowPhase::contact_pair(
  self : NarrowPhase,
  first : ColliderHandle,
  second : ColliderHandle,
) -> ContactPair? {
  ...
}

///|
#declaration_only
pub fn NarrowPhase::intersection_pair(
  self : NarrowPhase,
  first : ColliderHandle,
  second : ColliderHandle,
) -> IntersectionPair? {
  ...
}

///|
#declaration_only
pub fn NarrowPhase::update(
  self : NarrowPhase,
  bodies : @dynamics.RigidBodySet,
  colliders : ColliderSet,
  impulse_joints : @dynamics.ImpulseJointSet,
  multibody_joints : @dynamics.MultibodyJointSet,
) -> Unit {
  ...
}

///|
#declaration_only
pub fn NarrowPhase::update_with_pairs(
  self : NarrowPhase,
  bodies : @dynamics.RigidBodySet,
  colliders : ColliderSet,
  prediction_distance : @core.Real,
  pairs : Array[(ColliderHandle, ColliderHandle)],
  impulse_joints : @dynamics.ImpulseJointSet,
  multibody_joints : @dynamics.MultibodyJointSet,
) -> Unit {
  ...
}

///|
#declaration_only
pub type BroadPhaseBvh

///|
#declaration_only
pub fn BroadPhaseBvh::new() -> BroadPhaseBvh {
  ...
}

///|
#declaration_only
pub fn BroadPhaseBvh::with_optimization_strategy(
  optimization_strategy : BvhOptimizationStrategy,
) -> BroadPhaseBvh {
  ...
}

///|
#declaration_only
pub fn BroadPhaseBvh::as_query_pipeline(
  self : BroadPhaseBvh,
  bodies : @dynamics.RigidBodySet,
  colliders : ColliderSet,
  filter : QueryFilter,
) -> QueryPipeline {
  ...
}

///|
#declaration_only
pub fn BroadPhaseBvh::as_query_pipeline_mut(
  self : BroadPhaseBvh,
  bodies : @dynamics.RigidBodySet,
  colliders : ColliderSet,
  filter : QueryFilter,
) -> QueryPipelineMut {
  ...
}

///|
#declaration_only
pub fn BroadPhaseBvh::update(
  self : BroadPhaseBvh,
  prediction_distance : @core.Real,
  bodies : @dynamics.RigidBodySet,
  colliders : ColliderSet,
) -> Unit {
  ...
}

///|
#declaration_only
pub fn BroadPhaseBvh::pairs(
  self : BroadPhaseBvh,
) -> Array[(ColliderHandle, ColliderHandle)] {
  ...
}

///|
#declaration_only
pub fn BroadPhaseBvh::take_events(
  self : BroadPhaseBvh,
) -> Array[BroadPhasePairEvent] {
  ...
}

///|
#declaration_only
pub type QueryPipeline

///|
#declaration_only
pub type RayIntersection

///|
#declaration_only
pub fn RayIntersection::toi(self : RayIntersection) -> @core.Real {
  ...
}

///|
#declaration_only
pub fn RayIntersection::normal(self : RayIntersection) -> @core.Vec2 {
  ...
}

///|
#declaration_only
pub type PointProjection

///|
#declaration_only
pub fn PointProjection::point(self : PointProjection) -> @core.Vec2 {
  ...
}

///|
#declaration_only
pub fn PointProjection::is_inside(self : PointProjection) -> Bool {
  ...
}

///|
// NOTE: defined in collision/query_pipeline.mbt.
#declaration_only
pub type FeatureId

///|
#declaration_only
pub type ShapeCastOptions

///|
#declaration_only
pub fn ShapeCastOptions::new(
  max_toi : @core.Real,
  stop_at_penetration : Bool,
) -> ShapeCastOptions {
  ...
}

///|
#declaration_only
pub fn ShapeCastOptions::max_toi(self : ShapeCastOptions) -> @core.Real {
  ...
}

///|
#declaration_only
pub fn ShapeCastOptions::target_distance(self : ShapeCastOptions) -> @core.Real {
  ...
}

///|
#declaration_only
pub fn ShapeCastOptions::stop_at_penetration(self : ShapeCastOptions) -> Bool {
  ...
}

///|
#declaration_only
pub fn ShapeCastOptions::with_target_distance(
  self : ShapeCastOptions,
  target_distance : @core.Real,
) -> ShapeCastOptions {
  ...
}

///|
#declaration_only
pub type NonlinearRigidMotion

///|
#declaration_only
pub fn NonlinearRigidMotion::identity() -> NonlinearRigidMotion {
  ...
}

///|
#declaration_only
pub fn NonlinearRigidMotion::new(
  start_pos : @core.Isometry2,
  linvel : @core.Vec2,
  angvel : @core.Real,
) -> NonlinearRigidMotion {
  ...
}

///|
#declaration_only
pub type ShapeCastHit

///|
#declaration_only
pub fn ShapeCastHit::toi(self : ShapeCastHit) -> @core.Real {
  ...
}

///|
#declaration_only
pub fn ShapeCastHit::point(self : ShapeCastHit) -> @core.Vec2 {
  ...
}

///|
#declaration_only
pub fn ShapeCastHit::normal(self : ShapeCastHit) -> @core.Vec2 {
  ...
}

///|
#declaration_only
pub type QueryFilter

///|
#declaration_only
pub type QueryFilterFlags

///|
#declaration_only
pub fn QueryFilterFlags::new(bits : Int) -> QueryFilterFlags {
  ...
}

///|
#declaration_only
pub fn QueryFilterFlags::empty() -> QueryFilterFlags {
  ...
}

///|
#declaration_only
pub fn QueryFilterFlags::contains(self : QueryFilterFlags, bits : Int) -> Bool {
  ...
}

///|
#declaration_only
pub fn QueryFilterFlags::insert(
  self : QueryFilterFlags,
  bits : Int,
) -> QueryFilterFlags {
  ...
}

///|
pub struct Ray {
  origin : @core.Vec2
  dir : @core.Vec2
}

///|
#declaration_only
pub type Ray3

///|
#declaration_only
pub fn Ray::new(origin : @core.Vec2, dir : @core.Vec2) -> Ray {
  ...
}

///|
#declaration_only
pub fn Ray3::new(origin : @core.Vec3, dir : @core.Vec3) -> Ray3 {
  ...
}

///|
#declaration_only
pub fn QueryFilter::new() -> QueryFilter {
  ...
}

///|
#declaration_only
pub type QueryFilter3DReal

///|
#declaration_only
pub fn QueryFilter3DReal::new() -> QueryFilter3DReal {
  ...
}

///|
#declaration_only
pub fn QueryFilter3DReal::exclude_rigid_body(
  self : QueryFilter3DReal,
  handle : @dynamics.RigidBodyHandle,
) -> QueryFilter3DReal {
  ...
}

///|
#declaration_only
pub fn QueryFilter3DReal::exclude_collider(
  self : QueryFilter3DReal,
  handle : ColliderHandle3D,
) -> QueryFilter3DReal {
  ...
}

///|
#declaration_only
pub fn QueryFilter3DReal::groups(
  self : QueryFilter3DReal,
  groups : @dynamics.InteractionGroups,
) -> QueryFilter3DReal {
  ...
}

///|
#declaration_only
pub type RayIntersection3

///|
#declaration_only
pub fn RayIntersection3::toi(self : RayIntersection3) -> @core.Real {
  ...
}

///|
#declaration_only
pub fn RayIntersection3::normal(self : RayIntersection3) -> @core.Vec3 {
  ...
}

///|
#declaration_only
pub fn QueryFilter::exclude_rigid_body(
  self : QueryFilter,
  handle : @dynamics.RigidBodyHandle,
) -> QueryFilter {
  ...
}

///|
#declaration_only
pub fn QueryFilter::exclude_collider(
  self : QueryFilter,
  handle : ColliderHandle,
) -> QueryFilter {
  ...
}

///|
#declaration_only
pub fn QueryFilter::groups(
  self : QueryFilter,
  groups : @dynamics.InteractionGroups,
) -> QueryFilter {
  ...
}

///|
#declaration_only
pub fn QueryFilter::predicate(
  self : QueryFilter,
  predicate : (ColliderHandle, Collider) -> Bool,
) -> QueryFilter {
  ...
}

///|
#declaration_only
pub fn QueryFilter::exclude_fixed(self : QueryFilter) -> QueryFilter {
  ...
}

///|
#declaration_only
pub fn QueryFilter::exclude_kinematic(self : QueryFilter) -> QueryFilter {
  ...
}

///|
#declaration_only
pub fn QueryFilter::exclude_dynamic(self : QueryFilter) -> QueryFilter {
  ...
}

///|
#declaration_only
pub fn QueryFilter::only_dynamic() -> QueryFilter {
  ...
}

///|
#declaration_only
pub fn QueryFilter::only_kinematic() -> QueryFilter {
  ...
}

///|
#declaration_only
pub fn QueryFilter::only_fixed() -> QueryFilter {
  ...
}

///|
#declaration_only
pub fn QueryFilter::exclude_sensors(self : QueryFilter) -> QueryFilter {
  ...
}

///|
#declaration_only
pub fn QueryFilter::exclude_solids(self : QueryFilter) -> QueryFilter {
  ...
}

///|
#declaration_only
pub fn QueryFilter::groups_filter(
  self : QueryFilter,
) -> @dynamics.InteractionGroups? {
  ...
}

///|
#declaration_only
pub fn QueryPipeline::new(
  filter : QueryFilter,
  bodies : @dynamics.RigidBodySet,
  colliders : ColliderSet,
) -> QueryPipeline {
  ...
}

///|
/// Convenience constructor for dim3 wrapper sets (`RigidBodySet3`/`ColliderSet3`).
#declaration_only
pub fn QueryPipeline::new3(
  filter : QueryFilter,
  bodies : @dynamics.RigidBodySet3,
  colliders : ColliderSet3,
) -> QueryPipeline {
  ...
}

///|
#declaration_only
pub fn QueryPipeline::update(self : QueryPipeline) -> Unit {
  ...
}

///|
#declaration_only
pub fn QueryPipeline::with_filter(
  self : QueryPipeline,
  filter : QueryFilter,
) -> QueryPipeline {
  ...
}

///|
#declaration_only
pub type QueryPipelineMut

///|
#declaration_only
pub fn QueryPipelineMut::new(
  filter : QueryFilter,
  bodies : @dynamics.RigidBodySet,
  colliders : ColliderSet,
) -> QueryPipelineMut {
  ...
}

///|
/// Convenience constructor for dim3 wrapper sets (`RigidBodySet3`/`ColliderSet3`).
#declaration_only
pub fn QueryPipelineMut::new3(
  filter : QueryFilter,
  bodies : @dynamics.RigidBodySet3,
  colliders : ColliderSet3,
) -> QueryPipelineMut {
  ...
}

///|
#declaration_only
pub fn QueryPipelineMut::as_ref(self : QueryPipelineMut) -> QueryPipeline {
  ...
}

///|
#declaration_only
pub fn QueryPipelineMut::update(self : QueryPipelineMut) -> Unit {
  ...
}

///|
#declaration_only
pub fn QueryPipeline::cast_ray(
  self : QueryPipeline,
  ray : Ray,
  max_toi : @core.Real,
  solid : Bool,
) -> (ColliderHandle, @core.Real)? {
  ...
}

///|
#declaration_only
pub fn QueryPipeline::cast_ray_and_get_normal(
  self : QueryPipeline,
  ray : Ray,
  max_toi : @core.Real,
  solid : Bool,
) -> (ColliderHandle, RayIntersection)? {
  ...
}

///|
#declaration_only
pub type QueryPipeline3DReal

///|
#declaration_only
pub type QueryPipelineMut3DReal

///|
#declaration_only
pub fn QueryPipeline3DReal::new(
  filter : QueryFilter3DReal,
  bodies : @dynamics.RigidBodySet3D,
  colliders : ColliderSet3D,
) -> QueryPipeline3DReal {
  ...
}

///|
#declaration_only
pub fn QueryPipeline3DReal::update(self : QueryPipeline3DReal) -> Unit {
  ...
}

///|
#declaration_only
pub fn QueryPipeline3DReal::with_filter(
  self : QueryPipeline3DReal,
  filter : QueryFilter3DReal,
) -> QueryPipeline3DReal {
  ...
}

///|
#declaration_only
pub fn QueryPipeline3DReal::cast_ray_and_get_normal(
  self : QueryPipeline3DReal,
  ray : Ray3,
  max_toi : @core.Real,
  solid : Bool,
) -> (ColliderHandle3D, RayIntersection3)? {
  ...
}

///|
#declaration_only
pub fn QueryPipeline3DReal::rigid_body(
  self : QueryPipeline3DReal,
  handle : @dynamics.RigidBodyHandle,
) -> @dynamics.RigidBody3D? {
  ...
}

///|
#declaration_only
pub fn QueryPipeline3DReal::rigid_body_mut(
  self : QueryPipeline3DReal,
  handle : @dynamics.RigidBodyHandle,
) -> @dynamics.RigidBody3D? {
  ...
}

///|
#declaration_only
pub fn QueryPipeline3DReal::collider(
  self : QueryPipeline3DReal,
  handle : ColliderHandle3D,
) -> Collider3D? {
  ...
}

///|
#declaration_only
pub fn QueryPipelineMut3DReal::new(
  filter : QueryFilter3DReal,
  bodies : @dynamics.RigidBodySet3D,
  colliders : ColliderSet3D,
) -> QueryPipelineMut3DReal {
  ...
}

///|
#declaration_only
pub fn QueryPipelineMut3DReal::as_ref(
  self : QueryPipelineMut3DReal,
) -> QueryPipeline3DReal {
  ...
}

///|
#declaration_only
pub fn QueryPipelineMut3DReal::update(self : QueryPipelineMut3DReal) -> Unit {
  ...
}

///|
#declaration_only
pub fn QueryPipeline::intersect_ray(
  self : QueryPipeline,
  ray : Ray,
  max_toi : @core.Real,
  solid : Bool,
) -> Array[(ColliderHandle, RayIntersection)] {
  ...
}

///|
#declaration_only
pub fn QueryPipeline::cast_shape(
  self : QueryPipeline,
  shape_pos : @core.Isometry2,
  shape_vel : @core.Vec2,
  shape : Shape,
  options : ShapeCastOptions,
) -> (ColliderHandle, ShapeCastHit)? {
  ...
}

///|
#declaration_only
pub fn QueryPipeline::cast_shape_nonlinear(
  self : QueryPipeline,
  shape_motion : NonlinearRigidMotion,
  shape : Shape,
  start_time : @core.Real,
  end_time : @core.Real,
  stop_at_penetration : Bool,
) -> (ColliderHandle, ShapeCastHit)? {
  ...
}

///|
#declaration_only
pub fn QueryPipeline::collider(
  self : QueryPipeline,
  handle : ColliderHandle,
) -> Collider? {
  ...
}

///|
#declaration_only
pub fn QueryPipeline::rigid_body(
  self : QueryPipeline,
  handle : @dynamics.RigidBodyHandle,
) -> @dynamics.RigidBody? {
  ...
}

///|
#declaration_only
pub fn QueryPipeline::intersect_shape(
  self : QueryPipeline,
  shape_pos : @core.Isometry2,
  shape : Shape,
) -> Array[ColliderHandle] {
  ...
}

///|
#declaration_only
pub fn QueryPipeline::project_point(
  self : QueryPipeline,
  point : @core.Vec2,
  max_dist : @core.Real,
  solid : Bool,
) -> (ColliderHandle, PointProjection)? {
  ...
}

///|
#declaration_only
pub fn QueryPipeline::project_point_and_get_feature(
  self : QueryPipeline,
  point : @core.Vec2,
  max_dist : @core.Real,
  solid : Bool,
) -> (ColliderHandle, PointProjection, FeatureId)? {
  ...
}

///|
#declaration_only
pub fn QueryPipeline::intersect_aabb_conservative(
  self : QueryPipeline,
  aabb : @core.Aabb,
) -> Array[ColliderHandle] {
  ...
}

///|
#declaration_only
pub fn QueryPipeline::intersect_point(
  self : QueryPipeline,
  point : @core.Vec2,
) -> Array[ColliderHandle] {
  ...
}
