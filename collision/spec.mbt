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
pub(all) enum ActiveEvents {
  CollisionEvents
  ContactForceEvents
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
pub fn Collider::active_events(self : Collider) -> ActiveEvents? {
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
  events : ActiveEvents?,
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
pub type NarrowPhase

///|
#declaration_only
pub fn NarrowPhase::new() -> NarrowPhase {
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
) -> Unit {
  ...
}

///|
#declaration_only
pub fn NarrowPhase::update_with_pairs(
  self : NarrowPhase,
  bodies : @dynamics.RigidBodySet,
  colliders : ColliderSet,
  pairs : Array[(ColliderHandle, ColliderHandle)],
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
pub fn ShapeCastOptions::stop_at_penetration(self : ShapeCastOptions) -> Bool {
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
pub fn Ray::new(origin : @core.Vec2, dir : @core.Vec2) -> Ray {
  ...
}

///|
#declaration_only
pub fn QueryFilter::new() -> QueryFilter {
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
