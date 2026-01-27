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
pub struct ColliderHandle {
  id : Int
  generation : Int
}

///|
fn ColliderHandle::new(id : Int, generation : Int) -> ColliderHandle {
  ColliderHandle::{ id, generation }
}

///|
pub fn ColliderHandle::equals(
  self : ColliderHandle,
  other : ColliderHandle,
) -> Bool {
  self.id == other.id && self.generation == other.generation
}

///|
const COLLIDER_CHANGES_IN_MODIFIED_SET : Int = 1 << 0

///|
const COLLIDER_CHANGES_POSITION : Int = 1 << 1

///|
const COLLIDER_CHANGES_SHAPE : Int = 1 << 2

///|
const COLLIDER_CHANGES_LOCAL_MASS_PROPERTIES : Int = 1 << 3

///|
const COLLIDER_CHANGES_PARENT : Int = 1 << 4

///|
const COLLIDER_CHANGES_ENABLED_OR_DISABLED : Int = 1 << 5

///|
const COLLIDER_CHANGES_PARENT_EFFECTIVE_DOMINANCE : Int = 1 << 6

///|
const COLLIDER_CHANGES_ALL : Int = COLLIDER_CHANGES_IN_MODIFIED_SET |
  COLLIDER_CHANGES_POSITION |
  COLLIDER_CHANGES_SHAPE |
  COLLIDER_CHANGES_LOCAL_MASS_PROPERTIES |
  COLLIDER_CHANGES_PARENT |
  COLLIDER_CHANGES_ENABLED_OR_DISABLED |
  COLLIDER_CHANGES_PARENT_EFFECTIVE_DOMINANCE

///|
struct ColliderChanges {
  mut bits : Int
}

///|
fn ColliderChanges::all() -> ColliderChanges {
  { bits: COLLIDER_CHANGES_ALL }
}

///|
fn ColliderChanges::contains(self : ColliderChanges, flag : Int) -> Bool {
  (self.bits & flag) != 0
}

///|
fn ColliderChanges::insert(
  self : ColliderChanges,
  flag : Int,
) -> ColliderChanges {
  self.bits = self.bits | flag
  self
}

///|
fn ColliderChanges::remove(
  self : ColliderChanges,
  flag : Int,
) -> ColliderChanges {
  let clear_bits = self.bits & flag
  self.bits = self.bits - clear_bits
  self
}

///|
pub struct ActiveCollisionTypes {
  value : Int
}

///|
const ACTIVE_COLLISION_DYNAMIC_DYNAMIC : Int = 0x0001

///|
const ACTIVE_COLLISION_DYNAMIC_KINEMATIC : Int = 0x000C

///|
const ACTIVE_COLLISION_DYNAMIC_FIXED : Int = 0x0002

///|
const ACTIVE_COLLISION_KINEMATIC_KINEMATIC : Int = 0xCC00

///|
const ACTIVE_COLLISION_KINEMATIC_FIXED : Int = 0x2200

///|
const ACTIVE_COLLISION_FIXED_FIXED : Int = 0x0020

///|
const ACTIVE_COLLISION_DEFAULT : Int = ACTIVE_COLLISION_DYNAMIC_DYNAMIC |
  ACTIVE_COLLISION_DYNAMIC_KINEMATIC |
  ACTIVE_COLLISION_DYNAMIC_FIXED

///|
const ACTIVE_COLLISION_ALL : Int = ACTIVE_COLLISION_DEFAULT |
  ACTIVE_COLLISION_KINEMATIC_KINEMATIC |
  ACTIVE_COLLISION_KINEMATIC_FIXED |
  ACTIVE_COLLISION_FIXED_FIXED

///|
pub fn ActiveCollisionTypes::all() -> ActiveCollisionTypes {
  { value: ACTIVE_COLLISION_ALL }
}

///|
pub fn ActiveCollisionTypes::default() -> ActiveCollisionTypes {
  { value: ACTIVE_COLLISION_DEFAULT }
}

///|
pub fn ActiveCollisionTypes::equals(
  self : ActiveCollisionTypes,
  other : ActiveCollisionTypes,
) -> Bool {
  self.value == other.value
}

///|
fn rigid_body_type_index(body_type : @dynamics.RigidBodyType) -> Int {
  match body_type {
    @dynamics.RigidBodyType::Dynamic => 0
    @dynamics.RigidBodyType::Fixed => 1
    @dynamics.RigidBodyType::KinematicPositionBased => 2
    @dynamics.RigidBodyType::KinematicVelocityBased => 3
  }
}

///|
pub fn ActiveCollisionTypes::test_body_types(
  self : ActiveCollisionTypes,
  body_type1 : @dynamics.RigidBodyType,
  body_type2 : @dynamics.RigidBodyType,
) -> Bool {
  let index1 = rigid_body_type_index(body_type1)
  let index2 = rigid_body_type_index(body_type2)
  let mask1 = (self.value >> (index1 * 4)) & 0xF
  let mask2 = (self.value >> (index2 * 4)) & 0xF
  (mask1 & (1 << index2)) != 0 || (mask2 & (1 << index1)) != 0
}

///|
const ACTIVE_HOOKS_FILTER_CONTACT_PAIRS : Int = 1 << 0

///|
const ACTIVE_HOOKS_FILTER_INTERSECTION_PAIR : Int = 1 << 1

///|
const ACTIVE_HOOKS_MODIFY_SOLVER_CONTACTS : Int = 1 << 2

///|
pub struct ActiveHooks {
  mut value : Int
}

///|
pub fn ActiveHooks::empty() -> ActiveHooks {
  { value: 0 }
}

///|
pub fn ActiveHooks::all() -> ActiveHooks {
  {
    value: ACTIVE_HOOKS_FILTER_CONTACT_PAIRS |
    ACTIVE_HOOKS_FILTER_INTERSECTION_PAIR |
    ACTIVE_HOOKS_MODIFY_SOLVER_CONTACTS,
  }
}

///|
pub fn ActiveHooks::filter_contact_pairs() -> ActiveHooks {
  { value: ACTIVE_HOOKS_FILTER_CONTACT_PAIRS }
}

///|
pub fn ActiveHooks::filter_intersection_pair() -> ActiveHooks {
  { value: ACTIVE_HOOKS_FILTER_INTERSECTION_PAIR }
}

///|
pub fn ActiveHooks::modify_solver_contacts() -> ActiveHooks {
  { value: ACTIVE_HOOKS_MODIFY_SOLVER_CONTACTS }
}

///|
pub fn ActiveHooks::contains(self : ActiveHooks, other : ActiveHooks) -> Bool {
  (self.value & other.value) == other.value
}

///|
pub fn ActiveHooks::insert(
  self : ActiveHooks,
  other : ActiveHooks,
) -> ActiveHooks {
  self.value = self.value | other.value
  self
}

///|
pub(all) enum Shape {
  Ball(@core.Real)
  Cuboid(@core.Real, @core.Real)
  CapsuleX(@core.Real, @core.Real)
  CapsuleY(@core.Real, @core.Real)
  Segment(@core.Vec2, @core.Vec2)
}

///|
pub struct Collider {
  mut shape : Shape
  mut local_translation : @core.Vec2
  mut local_rotation : @core.Real
  mut world_translation : @core.Vec2
  mut world_rotation : @core.Real
  mut active_collision_types : ActiveCollisionTypes
  mut sensor : Bool
  mut active_events : ActiveEvents
  mut active_hooks : ActiveHooks
  mut collision_groups : @dynamics.InteractionGroups
  mut solver_groups : @dynamics.InteractionGroups
  mut density : @core.Real
  mut friction : @core.Real
  mut friction_combine_rule : @dynamics.CoefficientCombineRule
  mut restitution : @core.Real
  mut restitution_combine_rule : @dynamics.CoefficientCombineRule
  mut contact_skin : @core.Real
  mut contact_force_event_threshold : @core.Real
  mut user_data : Int
  mut parent : @dynamics.RigidBodyHandle?
  mut enabled : ColliderEnabled
  mut changes : ColliderChanges
}

///|
pub struct ColliderBuilder {
  shape : Shape
  mut translation : @core.Vec2
  mut rotation : @core.Real
  mut active_collision_types : ActiveCollisionTypes
  mut sensor : Bool
  mut active_events : ActiveEvents
  mut active_hooks : ActiveHooks
  mut collision_groups : @dynamics.InteractionGroups
  mut solver_groups : @dynamics.InteractionGroups
  mut density : @core.Real
  mut friction : @core.Real
  mut friction_combine_rule : @dynamics.CoefficientCombineRule
  mut restitution : @core.Real
  mut restitution_combine_rule : @dynamics.CoefficientCombineRule
  mut contact_skin : @core.Real
  mut contact_force_event_threshold : @core.Real
  mut user_data : Int
  mut enabled : Bool
}

///|
pub fn ColliderBuilder::ball(radius : @core.Real) -> ColliderBuilder {
  {
    shape: Shape::Ball(radius),
    translation: @core.Vec2::zero(),
    rotation: 0.0F,
    active_collision_types: ActiveCollisionTypes::default(),
    sensor: false,
    active_events: ActiveEvents::empty(),
    active_hooks: ActiveHooks::empty(),
    collision_groups: @dynamics.InteractionGroups::all(),
    solver_groups: @dynamics.InteractionGroups::all(),
    density: 1.0F,
    friction: 0.5F,
    friction_combine_rule: @dynamics.CoefficientCombineRule::Average,
    restitution: 0.0F,
    restitution_combine_rule: @dynamics.CoefficientCombineRule::Average,
    contact_skin: 0.0F,
    contact_force_event_threshold: 0.0F,
    user_data: 0,
    enabled: true,
  }
}

///|
pub fn ColliderBuilder::cuboid(
  half_width : @core.Real,
  half_height : @core.Real,
) -> ColliderBuilder {
  {
    shape: Shape::Cuboid(half_width, half_height),
    translation: @core.Vec2::zero(),
    rotation: 0.0F,
    active_collision_types: ActiveCollisionTypes::default(),
    sensor: false,
    active_events: ActiveEvents::empty(),
    active_hooks: ActiveHooks::empty(),
    collision_groups: @dynamics.InteractionGroups::all(),
    solver_groups: @dynamics.InteractionGroups::all(),
    density: 1.0F,
    friction: 0.5F,
    friction_combine_rule: @dynamics.CoefficientCombineRule::Average,
    restitution: 0.0F,
    restitution_combine_rule: @dynamics.CoefficientCombineRule::Average,
    contact_skin: 0.0F,
    contact_force_event_threshold: 0.0F,
    user_data: 0,
    enabled: true,
  }
}

///|
pub fn ColliderBuilder::capsule_x(
  half_height : @core.Real,
  radius : @core.Real,
) -> ColliderBuilder {
  {
    shape: Shape::CapsuleX(half_height, radius),
    translation: @core.Vec2::zero(),
    rotation: 0.0F,
    active_collision_types: ActiveCollisionTypes::default(),
    sensor: false,
    active_events: ActiveEvents::empty(),
    active_hooks: ActiveHooks::empty(),
    collision_groups: @dynamics.InteractionGroups::all(),
    solver_groups: @dynamics.InteractionGroups::all(),
    density: 1.0F,
    friction: 0.5F,
    friction_combine_rule: @dynamics.CoefficientCombineRule::Average,
    restitution: 0.0F,
    restitution_combine_rule: @dynamics.CoefficientCombineRule::Average,
    contact_skin: 0.0F,
    contact_force_event_threshold: 0.0F,
    user_data: 0,
    enabled: true,
  }
}

///|
pub fn ColliderBuilder::capsule_y(
  half_height : @core.Real,
  radius : @core.Real,
) -> ColliderBuilder {
  {
    shape: Shape::CapsuleY(half_height, radius),
    translation: @core.Vec2::zero(),
    rotation: 0.0F,
    active_collision_types: ActiveCollisionTypes::default(),
    sensor: false,
    active_events: ActiveEvents::empty(),
    active_hooks: ActiveHooks::empty(),
    collision_groups: @dynamics.InteractionGroups::all(),
    solver_groups: @dynamics.InteractionGroups::all(),
    density: 1.0F,
    friction: 0.5F,
    friction_combine_rule: @dynamics.CoefficientCombineRule::Average,
    restitution: 0.0F,
    restitution_combine_rule: @dynamics.CoefficientCombineRule::Average,
    contact_skin: 0.0F,
    contact_force_event_threshold: 0.0F,
    user_data: 0,
    enabled: true,
  }
}

///|
pub fn ColliderBuilder::segment(
  a : @core.Vec2,
  b : @core.Vec2,
) -> ColliderBuilder {
  {
    shape: Shape::Segment(a, b),
    translation: @core.Vec2::zero(),
    rotation: 0.0F,
    active_collision_types: ActiveCollisionTypes::default(),
    sensor: false,
    active_events: ActiveEvents::empty(),
    active_hooks: ActiveHooks::empty(),
    collision_groups: @dynamics.InteractionGroups::all(),
    solver_groups: @dynamics.InteractionGroups::all(),
    density: 1.0F,
    friction: 0.5F,
    friction_combine_rule: @dynamics.CoefficientCombineRule::Average,
    restitution: 0.0F,
    restitution_combine_rule: @dynamics.CoefficientCombineRule::Average,
    contact_skin: 0.0F,
    contact_force_event_threshold: 0.0F,
    user_data: 0,
    enabled: true,
  }
}

///|
pub fn ColliderBuilder::translation(
  self : ColliderBuilder,
  translation : @core.Vec2,
) -> ColliderBuilder {
  self.translation = translation
  self
}

///|
pub fn ColliderBuilder::rotation(
  self : ColliderBuilder,
  rotation : @core.Real,
) -> ColliderBuilder {
  self.rotation = rotation
  self
}

///|
pub fn ColliderBuilder::active_collision_types(
  self : ColliderBuilder,
  types : ActiveCollisionTypes,
) -> ColliderBuilder {
  self.active_collision_types = types
  self
}

///|
pub fn ColliderBuilder::sensor(
  self : ColliderBuilder,
  enabled : Bool,
) -> ColliderBuilder {
  self.sensor = enabled
  self
}

///|
pub fn ColliderBuilder::active_events(
  self : ColliderBuilder,
  events : ActiveEvents,
) -> ColliderBuilder {
  self.active_events = events
  self
}

///|
pub fn ColliderBuilder::active_hooks(
  self : ColliderBuilder,
  hooks : ActiveHooks,
) -> ColliderBuilder {
  self.active_hooks = hooks
  self
}

///|
pub fn ColliderBuilder::collision_groups(
  self : ColliderBuilder,
  groups : @dynamics.InteractionGroups,
) -> ColliderBuilder {
  self.collision_groups = groups
  self
}

///|
pub fn ColliderBuilder::solver_groups(
  self : ColliderBuilder,
  groups : @dynamics.InteractionGroups,
) -> ColliderBuilder {
  self.solver_groups = groups
  self
}

///|
pub fn ColliderBuilder::density(
  self : ColliderBuilder,
  density : @core.Real,
) -> ColliderBuilder {
  self.density = density
  self
}

///|
pub fn ColliderBuilder::friction(
  self : ColliderBuilder,
  friction : @core.Real,
) -> ColliderBuilder {
  self.friction = friction
  self
}

///|
pub fn ColliderBuilder::friction_combine_rule(
  self : ColliderBuilder,
  rule : @dynamics.CoefficientCombineRule,
) -> ColliderBuilder {
  self.friction_combine_rule = rule
  self
}

///|
pub fn ColliderBuilder::restitution(
  self : ColliderBuilder,
  value : @core.Real,
) -> ColliderBuilder {
  self.restitution = value
  self
}

///|
pub fn ColliderBuilder::restitution_combine_rule(
  self : ColliderBuilder,
  rule : @dynamics.CoefficientCombineRule,
) -> ColliderBuilder {
  self.restitution_combine_rule = rule
  self
}

///|
pub fn ColliderBuilder::contact_skin(
  self : ColliderBuilder,
  skin : @core.Real,
) -> ColliderBuilder {
  self.contact_skin = skin
  self
}

///|
pub fn ColliderBuilder::contact_force_event_threshold(
  self : ColliderBuilder,
  threshold : @core.Real,
) -> ColliderBuilder {
  self.contact_force_event_threshold = threshold
  self
}

///|
pub fn ColliderBuilder::user_data(
  self : ColliderBuilder,
  data : Int,
) -> ColliderBuilder {
  self.user_data = data
  self
}

///|
pub fn ColliderBuilder::enabled(
  self : ColliderBuilder,
  enabled : Bool,
) -> ColliderBuilder {
  self.enabled = enabled
  self
}

///|
pub fn ColliderBuilder::build(self : ColliderBuilder) -> Collider {
  {
    shape: self.shape,
    local_translation: self.translation,
    local_rotation: self.rotation,
    world_translation: self.translation,
    world_rotation: self.rotation,
    active_collision_types: self.active_collision_types,
    sensor: self.sensor,
    active_events: self.active_events,
    active_hooks: self.active_hooks,
    collision_groups: self.collision_groups,
    solver_groups: self.solver_groups,
    density: self.density,
    friction: self.friction,
    friction_combine_rule: self.friction_combine_rule,
    restitution: self.restitution,
    restitution_combine_rule: self.restitution_combine_rule,
    contact_skin: self.contact_skin,
    contact_force_event_threshold: self.contact_force_event_threshold,
    user_data: self.user_data,
    parent: None,
    enabled: if self.enabled {
      ColliderEnabled::Enabled
    } else {
      ColliderEnabled::Disabled
    },
    changes: ColliderChanges::all(),
  }
}

///|
pub fn Collider::shape(self : Collider) -> Shape {
  self.shape
}

///|
pub fn Collider::set_shape(self : Collider, shape : Shape) -> Collider {
  self.shape = shape
  self.changes = self.changes.insert(COLLIDER_CHANGES_SHAPE)
  self
}

///|
pub fn Collider::translation(self : Collider) -> @core.Vec2 {
  self.world_translation
}

///|
pub fn Collider::position(self : Collider) -> @core.Isometry2 {
  @core.Isometry2::new(
    self.world_translation,
    @core.Rot2::from_angle(self.world_rotation),
  )
}

///|
pub fn Collider::rotation(self : Collider) -> @core.Real {
  self.world_rotation
}

///|
pub fn Collider::position_wrt_parent(self : Collider) -> @core.Isometry2? {
  if self.parent is Some(_) {
    Some(
      @core.Isometry2::new(
        self.local_translation,
        @core.Rot2::from_angle(self.local_rotation),
      ),
    )
  } else {
    None
  }
}

///|
pub fn Collider::parent(self : Collider) -> @dynamics.RigidBodyHandle? {
  self.parent
}

///|
pub fn Collider::is_sensor(self : Collider) -> Bool {
  self.sensor
}

///|
pub fn Collider::active_collision_types(
  self : Collider,
) -> ActiveCollisionTypes {
  self.active_collision_types
}

///|
pub fn Collider::active_events(self : Collider) -> ActiveEvents {
  self.active_events
}

///|
pub fn Collider::active_hooks(self : Collider) -> ActiveHooks {
  self.active_hooks
}

///|
pub fn Collider::collision_groups(
  self : Collider,
) -> @dynamics.InteractionGroups {
  self.collision_groups
}

///|
pub fn Collider::solver_groups(self : Collider) -> @dynamics.InteractionGroups {
  self.solver_groups
}

///|
pub fn Collider::density(self : Collider) -> @core.Real {
  self.density
}

///|
pub fn Collider::mass_properties(self : Collider) -> @core.MassProperties {
  shape_mass_properties(self.shape, self.density)
}

///|
pub fn Collider::friction(self : Collider) -> @core.Real {
  self.friction
}

///|
pub fn Collider::friction_combine_rule(
  self : Collider,
) -> @dynamics.CoefficientCombineRule {
  self.friction_combine_rule
}

///|
pub fn Collider::restitution(self : Collider) -> @core.Real {
  self.restitution
}

///|
pub fn Collider::restitution_combine_rule(
  self : Collider,
) -> @dynamics.CoefficientCombineRule {
  self.restitution_combine_rule
}

///|
pub fn Collider::contact_skin(self : Collider) -> @core.Real {
  self.contact_skin
}

///|
pub fn Collider::contact_force_event_threshold(self : Collider) -> @core.Real {
  self.contact_force_event_threshold
}

///|
pub fn Collider::user_data(self : Collider) -> Int {
  self.user_data
}

///|
pub fn Collider::is_enabled(self : Collider) -> Bool {
  self.enabled is ColliderEnabled::Enabled
}

///|
fn shape_mass_properties(
  shape : Shape,
  density : @core.Real,
) -> @core.MassProperties {
  let pi = Float::from_double(@math.PI)
  match shape {
    Shape::Ball(radius) => {
      let area = pi * radius * radius
      let mass = density * area
      let inertia = 0.5F * mass * radius * radius
      @core.MassProperties::new(mass, inertia, @core.Vec2::zero())
    }
    Shape::Cuboid(hw, hh) => {
      let width = 2.0F * hw
      let height = 2.0F * hh
      let area = width * height
      let mass = density * area
      let inertia = mass * (width * width + height * height) / 12.0F
      @core.MassProperties::new(mass, inertia, @core.Vec2::zero())
    }
    Shape::CapsuleX(half_height, radius) => {
      let area_rect = 4.0F * radius * half_height
      let area_semi = 0.5F * pi * radius * radius
      let mass_rect = density * area_rect
      let mass_semi = density * area_semi
      let inertia_rect = if mass_rect <= 1.0e-12F {
        0.0F
      } else {
        mass_rect / 3.0F * (half_height * half_height + radius * radius)
      }
      let a = if radius <= 1.0e-12F {
        0.0F
      } else {
        4.0F * radius / (3.0F * pi)
      }
      let inertia_semi_center = 0.5F * mass_semi * radius * radius
      let inertia_semi_centroid = inertia_semi_center - mass_semi * a * a
      let d = half_height + a
      let inertia_semi_origin = inertia_semi_centroid + mass_semi * d * d
      let mass = mass_rect + 2.0F * mass_semi
      let inertia = inertia_rect + 2.0F * inertia_semi_origin
      @core.MassProperties::new(mass, inertia, @core.Vec2::zero())
    }
    Shape::CapsuleY(half_height, radius) => {
      // Same as CapsuleX: inertia is a scalar about the z-axis.
      let area_rect = 4.0F * radius * half_height
      let area_semi = 0.5F * pi * radius * radius
      let mass_rect = density * area_rect
      let mass_semi = density * area_semi
      let inertia_rect = if mass_rect <= 1.0e-12F {
        0.0F
      } else {
        mass_rect / 3.0F * (half_height * half_height + radius * radius)
      }
      let a = if radius <= 1.0e-12F {
        0.0F
      } else {
        4.0F * radius / (3.0F * pi)
      }
      let inertia_semi_center = 0.5F * mass_semi * radius * radius
      let inertia_semi_centroid = inertia_semi_center - mass_semi * a * a
      let d = half_height + a
      let inertia_semi_origin = inertia_semi_centroid + mass_semi * d * d
      let mass = mass_rect + 2.0F * mass_semi
      let inertia = inertia_rect + 2.0F * inertia_semi_origin
      @core.MassProperties::new(mass, inertia, @core.Vec2::zero())
    }
    Shape::Segment(a, b) => {
      // Like Rapier/Parry: segments have no area, so they don't contribute to mass properties.
      let com = @core.Vec2::new((a.x + b.x) * 0.5F, (a.y + b.y) * 0.5F)
      @core.MassProperties::new(0.0F, 0.0F, com)
    }
  }
}

///|
pub fn Collider::set_density(self : Collider, density : @core.Real) -> Collider {
  if self.density != density {
    self.density = density
    self.changes = self.changes.insert(COLLIDER_CHANGES_LOCAL_MASS_PROPERTIES)
  }
  self
}

///|
pub fn Collider::set_sensor(self : Collider, enabled : Bool) -> Collider {
  if self.sensor != enabled {
    self.sensor = enabled
    self.changes = self.changes.insert(COLLIDER_CHANGES_SHAPE)
  }
  self
}

///|
pub fn Collider::set_active_collision_types(
  self : Collider,
  types : ActiveCollisionTypes,
) -> Collider {
  self.active_collision_types = types
  self.changes = self.changes.insert(COLLIDER_CHANGES_SHAPE)
  self
}

///|
pub fn Collider::set_active_events(
  self : Collider,
  events : ActiveEvents,
) -> Collider {
  self.active_events = events
  self.changes = self.changes.insert(COLLIDER_CHANGES_SHAPE)
  self
}

///|
pub fn Collider::set_active_hooks(
  self : Collider,
  hooks : ActiveHooks,
) -> Collider {
  self.active_hooks = hooks
  self.changes = self.changes.insert(COLLIDER_CHANGES_SHAPE)
  self
}

///|
pub fn Collider::set_collision_groups(
  self : Collider,
  groups : @dynamics.InteractionGroups,
) -> Collider {
  self.collision_groups = groups
  self.changes = self.changes.insert(COLLIDER_CHANGES_SHAPE)
  self
}

///|
pub fn Collider::set_solver_groups(
  self : Collider,
  groups : @dynamics.InteractionGroups,
) -> Collider {
  self.solver_groups = groups
  self.changes = self.changes.insert(COLLIDER_CHANGES_SHAPE)
  self
}

///|
pub fn Collider::set_friction(
  self : Collider,
  friction : @core.Real,
) -> Collider {
  self.friction = friction
  self.changes = self.changes.insert(COLLIDER_CHANGES_SHAPE)
  self
}

///|
pub fn Collider::set_friction_combine_rule(
  self : Collider,
  rule : @dynamics.CoefficientCombineRule,
) -> Collider {
  self.friction_combine_rule = rule
  self.changes = self.changes.insert(COLLIDER_CHANGES_SHAPE)
  self
}

///|
pub fn Collider::set_restitution(
  self : Collider,
  restitution : @core.Real,
) -> Collider {
  self.restitution = restitution
  self.changes = self.changes.insert(COLLIDER_CHANGES_SHAPE)
  self
}

///|
pub fn Collider::set_restitution_combine_rule(
  self : Collider,
  rule : @dynamics.CoefficientCombineRule,
) -> Collider {
  self.restitution_combine_rule = rule
  self.changes = self.changes.insert(COLLIDER_CHANGES_SHAPE)
  self
}

///|
pub fn Collider::set_contact_skin(
  self : Collider,
  skin : @core.Real,
) -> Collider {
  self.contact_skin = skin
  self.changes = self.changes.insert(COLLIDER_CHANGES_SHAPE)
  self
}

///|
pub fn Collider::set_contact_force_event_threshold(
  self : Collider,
  threshold : @core.Real,
) -> Collider {
  self.contact_force_event_threshold = threshold
  self.changes = self.changes.insert(COLLIDER_CHANGES_SHAPE)
  self
}

///|
pub fn Collider::set_user_data(self : Collider, data : Int) -> Collider {
  self.user_data = data
  self
}

///|
pub fn Collider::set_translation(
  self : Collider,
  translation : @core.Vec2,
) -> Collider {
  if self.parent is None {
    self.local_translation = translation
    self.world_translation = translation
    self.changes = self.changes.insert(COLLIDER_CHANGES_POSITION)
  }
  self
}

///|
pub fn Collider::set_rotation(
  self : Collider,
  rotation : @core.Real,
) -> Collider {
  if self.parent is None {
    self.local_rotation = rotation
    self.world_rotation = rotation
    self.changes = self.changes.insert(COLLIDER_CHANGES_POSITION)
  }
  self
}

///|
pub fn Collider::set_position(
  self : Collider,
  position : @core.Isometry2,
) -> Collider {
  if self.parent is None {
    self.local_translation = position.translation
    self.local_rotation = position.rotation.angle()
    self.world_translation = position.translation
    self.world_rotation = position.rotation.angle()
    self.changes = self.changes.insert(COLLIDER_CHANGES_POSITION)
  }
  self
}

///|
pub fn Collider::set_translation_wrt_parent(
  self : Collider,
  translation : @core.Vec2,
) -> Collider {
  if self.parent is Some(_) {
    self.local_translation = translation
    self.changes = self.changes.insert(COLLIDER_CHANGES_PARENT)
  }
  self
}

///|
pub fn Collider::set_rotation_wrt_parent(
  self : Collider,
  rotation : @core.Real,
) -> Collider {
  if self.parent is Some(_) {
    self.local_rotation = rotation
    self.changes = self.changes.insert(COLLIDER_CHANGES_PARENT)
  }
  self
}

///|
pub fn Collider::set_enabled(self : Collider, enabled : Bool) -> Collider {
  match self.enabled {
    ColliderEnabled::Enabled | ColliderEnabled::DisabledByParent =>
      if !enabled {
        self.changes = self.changes.insert(COLLIDER_CHANGES_ENABLED_OR_DISABLED)
        self.enabled = ColliderEnabled::Disabled
      }
    ColliderEnabled::Disabled =>
      if enabled {
        self.changes = self.changes.insert(COLLIDER_CHANGES_ENABLED_OR_DISABLED)
        self.enabled = ColliderEnabled::Enabled
      }
  }
  self
}

///|
fn Collider::clear_changes(self : Collider) -> Collider {
  self.changes = self.changes.remove(COLLIDER_CHANGES_ALL)
  self
}

///|
pub struct ColliderSet {
  colliders : Array[Collider?]
  generations : Array[Int]
  free_list : Array[Int]
  modified_colliders : Array[ColliderHandle]
}

///|
pub fn ColliderSet::new() -> ColliderSet {
  { colliders: [], generations: [], free_list: [], modified_colliders: [] }
}

///|
fn ColliderSet::allocate_handle(self : ColliderSet) -> ColliderHandle {
  if self.free_list.pop() is Some(index) {
    let generation = self.generations[index]
    ColliderHandle::new(index, generation)
  } else {
    let index = self.colliders.length()
    self.colliders.push(None)
    self.generations.push(0)
    ColliderHandle::new(index, 0)
  }
}

///|
fn collider_handle_equals(
  left : ColliderHandle,
  right : ColliderHandle,
) -> Bool {
  left.id == right.id && left.generation == right.generation
}

///|
fn is_same_parent(left : Collider, right : Collider) -> Bool {
  if left.parent is Some(parent1) && right.parent is Some(parent2) {
    @dynamics.RigidBodyHandle::equals(parent1, parent2)
  } else {
    false
  }
}

///|
fn update_collider_world_pose(
  collider : Collider,
  bodies : @dynamics.RigidBodySet,
) -> Collider {
  if collider.parent is Some(parent) {
    if bodies.get(parent) is Some(body) {
      // Rapier semantics: world_position = body_position * local_position_wrt_parent.
      let body_pos = body.position()
      let local_pos = @core.Isometry2::new(
        collider.local_translation,
        @core.Rot2::from_angle(collider.local_rotation),
      )
      let world_pos = body_pos.mul(local_pos)
      collider.world_translation = world_pos.translation
      collider.world_rotation = world_pos.rotation.angle()
      return collider
    }
  }
  collider.world_translation = collider.local_translation
  collider.world_rotation = collider.local_rotation
  collider
}

///|
fn ColliderSet::push_modified_unchecked(
  self : ColliderSet,
  handle : ColliderHandle,
  collider : Collider,
) -> Unit {
  collider.changes = collider.changes.insert(COLLIDER_CHANGES_IN_MODIFIED_SET)
  self.modified_colliders.push(handle)
}

///|
fn ColliderSet::push_modified(
  self : ColliderSet,
  handle : ColliderHandle,
  collider : Collider,
) -> Unit {
  if !collider.changes.contains(COLLIDER_CHANGES_IN_MODIFIED_SET) {
    self.push_modified_unchecked(handle, collider)
  }
}

///|
fn ColliderSet::sync_with_bodies(
  self : ColliderSet,
  bodies : @dynamics.RigidBodySet,
) -> Unit {
  for i in 0..<self.colliders.length() {
    if self.colliders[i] is Some(collider) {
      self.colliders[i] = Some(update_collider_world_pose(collider, bodies))
    }
  }
}

///|
pub fn ColliderSet::insert(
  self : ColliderSet,
  collider : Collider,
) -> ColliderHandle {
  let handle = self.allocate_handle()
  let updated = collider
  updated.changes = ColliderChanges::all()
  self.colliders[handle.id] = Some(updated)
  if self.colliders[handle.id] is Some(collider_ref) {
    self.push_modified_unchecked(handle, collider_ref)
  }
  handle
}

///|
pub fn ColliderSet::colliders_with_parent(
  self : ColliderSet,
  parent : @dynamics.RigidBodyHandle,
) -> Array[ColliderHandle] {
  let result : Array[ColliderHandle] = []
  for i in 0..<self.colliders.length() {
    if self.colliders[i] is Some(collider) {
      if collider.parent is Some(current_parent) &&
        @dynamics.RigidBodyHandle::equals(current_parent, parent) {
        result.push(ColliderHandle::new(i, self.generations[i]))
      }
    }
  }
  result
}

///|
pub fn ColliderSet::insert_with_parent(
  self : ColliderSet,
  collider : Collider,
  parent : @dynamics.RigidBodyHandle,
  bodies : @dynamics.RigidBodySet,
) -> ColliderHandle {
  let handle = self.allocate_handle()
  let updated = collider
  updated.changes = ColliderChanges::all()
  updated.parent = Some(parent)
  let updated_world = update_collider_world_pose(updated, bodies)
  self.colliders[handle.id] = Some(updated_world)
  if self.colliders[handle.id] is Some(collider_ref) {
    self.push_modified_unchecked(handle, collider_ref)
  }
  handle
}

///|
pub fn ColliderSet::get(
  self : ColliderSet,
  handle : ColliderHandle,
) -> Collider? {
  if handle.id < 0 || handle.id >= self.colliders.length() {
    return None
  }
  if self.generations[handle.id] != handle.generation {
    return None
  }
  self.colliders[handle.id]
}

///|
pub fn ColliderSet::get_mut(
  self : ColliderSet,
  handle : ColliderHandle,
) -> Collider? {
  self.get_mut_internal_with_modification_tracking(handle)
}

///|
pub fn ColliderSet::get_mut_internal(
  self : ColliderSet,
  handle : ColliderHandle,
) -> Collider? {
  if handle.id < 0 || handle.id >= self.colliders.length() {
    return None
  }
  if self.generations[handle.id] != handle.generation {
    return None
  }
  self.colliders[handle.id]
}

///|
pub fn ColliderSet::get_mut_internal_with_modification_tracking(
  self : ColliderSet,
  handle : ColliderHandle,
) -> Collider? {
  if self.get_mut_internal(handle) is Some(collider) {
    self.push_modified(handle, collider)
    Some(collider)
  } else {
    None
  }
}

///|
pub fn ColliderSet::remove(
  self : ColliderSet,
  handle : ColliderHandle,
  islands : @dynamics.IslandManager,
  bodies : @dynamics.RigidBodySet,
  wake_up : Bool,
) -> Unit {
  if handle.id < 0 || handle.id >= self.colliders.length() {
    islands |> ignore
    bodies |> ignore
    wake_up |> ignore
    return
  }
  if self.generations[handle.id] != handle.generation {
    islands |> ignore
    bodies |> ignore
    wake_up |> ignore
    return
  }
  if self.colliders[handle.id] is Some(collider) {
    // Removing a collider must invalidate the parent rigid-body mass-properties like Rapier.
    if collider.parent is Some(parent) {
      if bodies.get_mut_internal_with_modification_tracking(parent)
        is Some(body) {
        body.mark_local_mass_properties_changed() |> ignore
        if wake_up && body.is_dynamic_or_kinematic() {
          body.wake_up(true) |> ignore
        }
      }
    }
    self.colliders[handle.id] = None
    self.generations[handle.id] = self.generations[handle.id] + 1
    self.free_list.push(handle.id)
  }
  islands |> ignore
}

///|
pub fn ColliderSet::take_modified(self : ColliderSet) -> Array[ColliderHandle] {
  let result : Array[ColliderHandle] = []
  for i in 0..<self.modified_colliders.length() {
    let handle = self.modified_colliders[i]
    result.push(handle)
    if handle.id >= 0 && handle.id < self.colliders.length() {
      if self.generations[handle.id] == handle.generation {
        if self.colliders[handle.id] is Some(collider) {
          collider.changes = collider.changes.remove(
            COLLIDER_CHANGES_IN_MODIFIED_SET,
          )
        }
      }
    }
  }
  self.modified_colliders.clear()
  result
}

///|
pub fn ColliderSet::clear_changes_for(
  self : ColliderSet,
  handles : Array[ColliderHandle],
) -> Unit {
  for i in 0..<handles.length() {
    let handle = handles[i]
    if self.get_mut_internal(handle) is Some(collider) {
      collider.clear_changes() |> ignore
    }
  }
}

///|
pub fn ColliderSet::set_parent(
  self : ColliderSet,
  handle : ColliderHandle,
  parent : @dynamics.RigidBodyHandle?,
  bodies : @dynamics.RigidBodySet,
) -> Unit {
  if handle.id < 0 || handle.id >= self.colliders.length() {
    return
  }
  if self.generations[handle.id] != handle.generation {
    return
  }
  if self.colliders[handle.id] is Some(collider) {
    collider.parent = parent
    collider.changes = collider.changes.insert(COLLIDER_CHANGES_PARENT)
    self.colliders[handle.id] = Some(
      update_collider_world_pose(collider, bodies),
    )
    if self.colliders[handle.id] is Some(updated) {
      self.push_modified(handle, updated)
    }
  }
}

///|
pub fn ColliderSet::set_parent_enabled(
  self : ColliderSet,
  parent : @dynamics.RigidBodyHandle,
  enabled : Bool,
) -> Unit {
  for i in 0..<self.colliders.length() {
    if self.colliders[i] is Some(collider) {
      if collider.parent is Some(current_parent) &&
        @dynamics.RigidBodyHandle::equals(current_parent, parent) {
        if enabled {
          if collider.enabled is ColliderEnabled::DisabledByParent {
            collider.enabled = ColliderEnabled::Enabled
            collider.changes = collider.changes.insert(
              COLLIDER_CHANGES_ENABLED_OR_DISABLED,
            )
          }
        } else if collider.enabled is ColliderEnabled::Enabled {
          collider.enabled = ColliderEnabled::DisabledByParent
          collider.changes = collider.changes.insert(
            COLLIDER_CHANGES_ENABLED_OR_DISABLED,
          )
        }
        let handle = ColliderHandle::new(i, self.generations[i])
        self.push_modified(handle, collider)
      }
    }
  }
}

///|
pub fn handle_user_changes_to_colliders(
  bodies : @dynamics.RigidBodySet,
  colliders : ColliderSet,
  modified_colliders : Array[ColliderHandle],
) -> Unit {
  for i in 0..<modified_colliders.length() {
    let handle = modified_colliders[i]
    if colliders.get_mut_internal(handle) is Some(collider) {
      let changes = collider.changes
      let parent = collider.parent
      if changes.contains(COLLIDER_CHANGES_PARENT) {
        let updated = update_collider_world_pose(collider, bodies)
        updated.changes = updated.changes.insert(COLLIDER_CHANGES_POSITION)
        if handle.id >= 0 && handle.id < colliders.colliders.length() {
          colliders.colliders[handle.id] = Some(updated)
        }
      }
      if changes.contains(COLLIDER_CHANGES_PARENT) ||
        changes.contains(COLLIDER_CHANGES_SHAPE) ||
        changes.contains(COLLIDER_CHANGES_LOCAL_MASS_PROPERTIES) ||
        changes.contains(COLLIDER_CHANGES_ENABLED_OR_DISABLED) {
        if parent is Some(rb_handle) {
          if bodies.get_mut_internal_with_modification_tracking(rb_handle)
            is Some(body) {
            body.mark_local_mass_properties_changed() |> ignore
          }
        }
      }
    }
  }
}

///|
pub struct ContactPair {
  manifolds : Array[ContactManifold]
}

///|
pub fn ContactPair::manifold_count(self : ContactPair) -> Int {
  self.manifolds.length()
}

///|
pub fn ContactPair::manifolds(self : ContactPair) -> Array[ContactManifold] {
  let result : Array[ContactManifold] = []
  for i in 0..<self.manifolds.length() {
    result.push(self.manifolds[i])
  }
  result
}

///|
pub struct ContactManifoldPoint {
  local_p1 : @core.Vec2
  local_p2 : @core.Vec2
  dist : @core.Real
  fid1 : FeatureId
  fid2 : FeatureId
}

///|
pub fn ContactManifoldPoint::local_p1(
  self : ContactManifoldPoint,
) -> @core.Vec2 {
  self.local_p1
}

///|
pub fn ContactManifoldPoint::local_p2(
  self : ContactManifoldPoint,
) -> @core.Vec2 {
  self.local_p2
}

///|
pub fn ContactManifoldPoint::dist(self : ContactManifoldPoint) -> @core.Real {
  self.dist
}

///|
pub fn ContactManifoldPoint::fid1(self : ContactManifoldPoint) -> FeatureId {
  self.fid1
}

///|
pub fn ContactManifoldPoint::fid2(self : ContactManifoldPoint) -> FeatureId {
  self.fid2
}

///|
pub struct ContactManifold {
  normal : @core.Vec2
  points : Array[ContactManifoldPoint]
}

///|
pub fn ContactManifold::normal(self : ContactManifold) -> @core.Vec2 {
  self.normal
}

///|
pub fn ContactManifold::points(
  self : ContactManifold,
) -> Array[ContactManifoldPoint] {
  let result : Array[ContactManifoldPoint] = []
  for i in 0..<self.points.length() {
    result.push(self.points[i])
  }
  result
}

///|
pub struct NarrowPhase {
  mut intersections : Array[IntersectionPair]
  mut contacts : Array[(ColliderHandle, ColliderHandle, ContactPair)]
}

///|
pub fn NarrowPhase::new() -> NarrowPhase {
  { intersections: [], contacts: [] }
}

///|
pub fn NarrowPhase::intersection_pairs(
  self : NarrowPhase,
) -> Array[IntersectionPair] {
  let result : Array[IntersectionPair] = []
  for i in 0..<self.intersections.length() {
    result.push(self.intersections[i])
  }
  result
}

///|
pub fn NarrowPhase::contact_pairs(
  self : NarrowPhase,
) -> Array[(ColliderHandle, ColliderHandle, ContactPair)] {
  let result : Array[(ColliderHandle, ColliderHandle, ContactPair)] = []
  for i in 0..<self.contacts.length() {
    result.push(self.contacts[i])
  }
  result
}

///|
fn find_intersection_pair(
  pairs : Array[IntersectionPair],
  first : ColliderHandle,
  second : ColliderHandle,
) -> IntersectionPair? {
  for i in 0..<pairs.length() {
    let pair = pairs[i]
    let matches = collider_handle_equals(pair.collider1, first) &&
      collider_handle_equals(pair.collider2, second)
    let matches_swapped = collider_handle_equals(pair.collider1, second) &&
      collider_handle_equals(pair.collider2, first)
    if matches || matches_swapped {
      return Some(pair)
    }
  }
  None
}

///|
fn find_contact_pair(
  pairs : Array[(ColliderHandle, ColliderHandle, ContactPair)],
  first : ColliderHandle,
  second : ColliderHandle,
) -> ContactPair? {
  for i in 0..<pairs.length() {
    let pair = pairs[i]
    let matches = collider_handle_equals(pair.0, first) &&
      collider_handle_equals(pair.1, second)
    let matches_swapped = collider_handle_equals(pair.0, second) &&
      collider_handle_equals(pair.1, first)
    if matches || matches_swapped {
      return Some(pair.2)
    }
  }
  None
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
fn cuboid_axes(rotation : @core.Real) -> (@core.Vec2, @core.Vec2) {
  let rot = @core.Rot2::from_angle(rotation)
  // rot.rotate_vec2((1, 0)) and rot.rotate_vec2((0, 1)) without allocating.
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
fn cuboid_cuboid_intersect(
  pos1 : @core.Vec2,
  rot1 : @core.Real,
  hw1 : @core.Real,
  hh1 : @core.Real,
  pos2 : @core.Vec2,
  rot2 : @core.Real,
  hw2 : @core.Real,
  hh2 : @core.Real,
) -> Bool {
  let (ax1, ay1) = cuboid_axes(rot1)
  let (ax2, ay2) = cuboid_axes(rot2)
  let delta = pos1.sub(pos2)
  let axes = [ax1, ay1, ax2, ay2]
  for axis in axes {
    let dist = @core.abs(delta.dot(axis))
    let r1 = projection_radius(axis, ax1, ay1, hw1, hh1)
    let r2 = projection_radius(axis, ax2, ay2, hw2, hh2)
    if dist > r1 + r2 {
      return false
    }
  }
  true
}

///|
fn shapes_intersect(
  shape1 : Shape,
  pos1 : @core.Vec2,
  rot1 : @core.Real,
  shape2 : Shape,
  pos2 : @core.Vec2,
  rot2 : @core.Real,
) -> Bool {
  fn cross2(a : @core.Vec2, b : @core.Vec2) -> @core.Real {
    a.x * b.y - a.y * b.x
  }

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
      a.add(@core.Vec2::new(ab.x * t, ab.y * t))
    }
  }

  fn segment_ball_intersect(
    seg_center : @core.Vec2,
    seg_rot : @core.Real,
    a : @core.Vec2,
    b : @core.Vec2,
    ball_center : @core.Vec2,
    radius : @core.Real,
  ) -> Bool {
    let (wa, wb) = segment_world_endpoints(seg_center, seg_rot, a, b)
    let q = closest_point_on_segment(wa, wb, ball_center)
    ball_center.sub(q).length_squared() <= radius * radius
  }

  fn segment_aabb_intersect(
    a : @core.Vec2,
    b : @core.Vec2,
    hw : @core.Real,
    hh : @core.Real,
  ) -> Bool {
    // Liang–Barsky clipping against axis-aligned AABB centered at origin.
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

  fn segment_cuboid_intersect(
    seg_center : @core.Vec2,
    seg_rot : @core.Real,
    a : @core.Vec2,
    b : @core.Vec2,
    cuboid_center : @core.Vec2,
    cuboid_rot : @core.Real,
    hw : @core.Real,
    hh : @core.Real,
  ) -> Bool {
    let (wa, wb) = segment_world_endpoints(seg_center, seg_rot, a, b)
    let rot = @core.Rot2::from_angle(cuboid_rot)
    let inv = rot.inverse()
    let la = inv.rotate_vec2(wa.sub(cuboid_center))
    let lb = inv.rotate_vec2(wb.sub(cuboid_center))
    segment_aabb_intersect(la, lb, hw, hh)
  }

  fn on_segment(a : @core.Vec2, b : @core.Vec2, p : @core.Vec2) -> Bool {
    min_value(a.x, b.x) - 1.0e-6F <= p.x &&
    p.x <= max_value(a.x, b.x) + 1.0e-6F &&
    min_value(a.y, b.y) - 1.0e-6F <= p.y &&
    p.y <= max_value(a.y, b.y) + 1.0e-6F
  }

  fn segment_segment_intersect_world(
    a1 : @core.Vec2,
    b1 : @core.Vec2,
    a2 : @core.Vec2,
    b2 : @core.Vec2,
  ) -> Bool {
    let r = b1.sub(a1)
    let s = b2.sub(a2)
    let denom = cross2(r, s)
    let qp = a2.sub(a1)
    let num_t = cross2(qp, s)
    let num_u = cross2(qp, r)
    if @core.abs(denom) <= 1.0e-12F {
      // Parallel: either disjoint or collinear overlap.
      if @core.abs(cross2(qp, r)) > 1.0e-6F {
        return false
      }
      // Collinear: check overlap of endpoints.
      on_segment(a1, b1, a2) ||
      on_segment(a1, b1, b2) ||
      on_segment(a2, b2, a1) ||
      on_segment(a2, b2, b1)
    } else {
      let t = num_t / denom
      let u = num_u / denom
      t >= 0.0F && t <= 1.0F && u >= 0.0F && u <= 1.0F
    }
  }

  fn segment_segment_intersect(
    center1 : @core.Vec2,
    rot1 : @core.Real,
    a1 : @core.Vec2,
    b1 : @core.Vec2,
    center2 : @core.Vec2,
    rot2 : @core.Real,
    a2 : @core.Vec2,
    b2 : @core.Vec2,
  ) -> Bool {
    let (wa1, wb1) = segment_world_endpoints(center1, rot1, a1, b1)
    let (wa2, wb2) = segment_world_endpoints(center2, rot2, a2, b2)
    segment_segment_intersect_world(wa1, wb1, wa2, wb2)
  }

  fn shapes_intersect_basic(
    s1 : Shape,
    p1 : @core.Vec2,
    r1 : @core.Real,
    s2 : Shape,
    p2 : @core.Vec2,
    r2 : @core.Real,
  ) -> Bool {
    match (s1, s2) {
      (Shape::Ball(br1), Shape::Ball(br2)) => {
        let delta = p2.sub(p1)
        let dist2 = delta.length_squared()
        let radius = br1 + br2
        dist2 <= radius * radius
      }
      (Shape::Cuboid(hw1, hh1), Shape::Cuboid(hw2, hh2)) =>
        cuboid_cuboid_intersect(p1, r1, hw1, hh1, p2, r2, hw2, hh2)
      (Shape::Ball(r), Shape::Cuboid(hw, hh)) =>
        ball_cuboid_intersect(p1, r, p2, r2, hw, hh)
      (Shape::Cuboid(hw, hh), Shape::Ball(r)) =>
        ball_cuboid_intersect(p2, r, p1, r1, hw, hh)
      (Shape::Segment(a, b), Shape::Ball(r)) =>
        segment_ball_intersect(p1, r1, a, b, p2, r)
      (Shape::Ball(r), Shape::Segment(a, b)) =>
        segment_ball_intersect(p2, r2, a, b, p1, r)
      (Shape::Segment(a, b), Shape::Cuboid(hw, hh)) =>
        segment_cuboid_intersect(p1, r1, a, b, p2, r2, hw, hh)
      (Shape::Cuboid(hw, hh), Shape::Segment(a, b)) =>
        segment_cuboid_intersect(p2, r2, a, b, p1, r1, hw, hh)
      (Shape::Segment(a1, b1), Shape::Segment(a2, b2)) =>
        segment_segment_intersect(p1, r1, a1, b1, p2, r2, a2, b2)
      _ => false
    }
  }

  fn capsule_components(
    center : @core.Vec2,
    rotation : @core.Real,
    axis : @core.Vec2,
    half_height : @core.Real,
    radius : @core.Real,
    core_hw : @core.Real,
    core_hh : @core.Real,
  ) -> Array[(Shape, @core.Vec2, @core.Real)] {
    let rot = @core.Rot2::from_angle(rotation)
    let offset = rot.rotate_vec2(
      @core.Vec2::new(axis.x * half_height, axis.y * half_height),
    )
    let a = center.add(offset)
    let b = center.sub(offset)
    [
      (Shape::Ball(radius), a, 0.0F),
      (Shape::Ball(radius), b, 0.0F),
      (Shape::Cuboid(core_hw, core_hh), center, rotation),
    ]
  }

  match (shape1, shape2) {
    (Shape::Ball(_), Shape::Ball(_))
    | (Shape::Ball(_), Shape::Cuboid(_, _))
    | (Shape::Cuboid(_, _), Shape::Ball(_))
    | (Shape::Cuboid(_, _), Shape::Cuboid(_, _)) =>
      shapes_intersect_basic(shape1, pos1, rot1, shape2, pos2, rot2)
    (Shape::Ball(_), Shape::Segment(_, _))
    | (Shape::Segment(_, _), Shape::Ball(_))
    | (Shape::Cuboid(_, _), Shape::Segment(_, _))
    | (Shape::Segment(_, _), Shape::Cuboid(_, _))
    | (Shape::Segment(_, _), Shape::Segment(_, _)) =>
      shapes_intersect_basic(shape1, pos1, rot1, shape2, pos2, rot2)
    (Shape::CapsuleX(h, r), other) => {
      let comps = capsule_components(
        pos1,
        rot1,
        @core.Vec2::new(1.0F, 0.0F),
        h,
        r,
        h,
        r,
      )
      for i in 0..<comps.length() {
        let c = comps[i]
        if shapes_intersect_basic(c.0, c.1, c.2, other, pos2, rot2) {
          return true
        }
      }
      false
    }
    (Shape::CapsuleY(h, r), other) => {
      let comps = capsule_components(
        pos1,
        rot1,
        @core.Vec2::new(0.0F, 1.0F),
        h,
        r,
        r,
        h,
      )
      for i in 0..<comps.length() {
        let c = comps[i]
        if shapes_intersect_basic(c.0, c.1, c.2, other, pos2, rot2) {
          return true
        }
      }
      false
    }
    (other, Shape::CapsuleX(h, r)) => {
      let comps = capsule_components(
        pos2,
        rot2,
        @core.Vec2::new(1.0F, 0.0F),
        h,
        r,
        h,
        r,
      )
      for i in 0..<comps.length() {
        let c = comps[i]
        if shapes_intersect_basic(other, pos1, rot1, c.0, c.1, c.2) {
          return true
        }
      }
      false
    }
    (other, Shape::CapsuleY(h, r)) => {
      let comps = capsule_components(
        pos2,
        rot2,
        @core.Vec2::new(0.0F, 1.0F),
        h,
        r,
        r,
        h,
      )
      for i in 0..<comps.length() {
        let c = comps[i]
        if shapes_intersect_basic(other, pos1, rot1, c.0, c.1, c.2) {
          return true
        }
      }
      false
    }
  }
}

///|
fn ball_cuboid_intersect(
  ball_pos : @core.Vec2,
  radius : @core.Real,
  cuboid_pos : @core.Vec2,
  cuboid_rot : @core.Real,
  half_width : @core.Real,
  half_height : @core.Real,
) -> Bool {
  let rot = @core.Rot2::from_angle(cuboid_rot)
  let inv = rot.inverse()
  let local_ball = inv.rotate_vec2(ball_pos.sub(cuboid_pos))
  let clamped = @core.Vec2::new(
    clamp_value(local_ball.x, -half_width, half_width),
    clamp_value(local_ball.y, -half_height, half_height),
  )
  local_ball.sub(clamped).length_squared() <= radius * radius
}

///|
pub fn NarrowPhase::update(
  self : NarrowPhase,
  bodies : @dynamics.RigidBodySet,
  colliders : ColliderSet,
  impulse_joints : @dynamics.ImpulseJointSet,
  multibody_joints : @dynamics.MultibodyJointSet,
) -> Unit {
  colliders.sync_with_bodies(bodies)
  let pairs : Array[(ColliderHandle, ColliderHandle)] = []
  for i in 0..<colliders.colliders.length() {
    if colliders.colliders[i] is Some(collider1) {
      if !collider1.is_enabled() {
        continue
      }
      let handle1 = ColliderHandle::new(i, colliders.generations[i])
      for j in (i + 1)..<colliders.colliders.length() {
        if colliders.colliders[j] is Some(collider2) {
          if !collider2.is_enabled() {
            continue
          }
          let handle2 = ColliderHandle::new(j, colliders.generations[j])
          pairs.push((handle1, handle2))
        }
      }
    }
  }
  self.update_with_pairs(
    bodies, colliders, 0.0F, pairs, impulse_joints, multibody_joints,
  )
}

///|
fn contact_manifold_ball_ball(
  collider1 : Collider,
  radius1 : @core.Real,
  collider2 : Collider,
  radius2 : @core.Real,
) -> ContactManifold {
  let p1 = collider1.world_translation
  let p2 = collider2.world_translation
  let d = p2.sub(p1)
  let dist = d.length()
  let normal = normalize_or_fallback(d, @core.Vec2::new(1.0F, 0.0F))
  let signed_dist = dist - (radius1 + radius2)
  let world_p1 = p1.add(vec2_scale(normal, radius1))
  let world_p2 = p2.sub(vec2_scale(normal, radius2))
  let local_p1 = collider1.position().inverse().transform_point(world_p1)
  let local_p2 = collider2.position().inverse().transform_point(world_p2)
  ContactManifold::{
    normal,
    points: [
      ContactManifoldPoint::{
        local_p1,
        local_p2,
        dist: signed_dist,
        fid1: FeatureId::Face(0),
        fid2: FeatureId::Face(0),
      },
    ],
  }
}

///|
fn contact_manifold_ball_cuboid(
  ball : Collider,
  radius : @core.Real,
  cuboid : Collider,
  half_width : @core.Real,
  half_height : @core.Real,
) -> ContactManifold {
  let ball_center = ball.world_translation
  let cuboid_center = cuboid.world_translation
  let (projection, feature2) = project_point_on_cuboid_oriented_and_get_feature(
    cuboid_center,
    cuboid.world_rotation,
    half_width,
    half_height,
    ball_center,
    false,
  )
  let closest = projection.point()
  let d = closest.sub(ball_center)
  let dist = d.length()
  let normal = normalize_or_fallback(d, @core.Vec2::new(1.0F, 0.0F))
  let signed_dist = dist - radius
  let world_p1 = ball_center.add(vec2_scale(normal, radius))
  let world_p2 = closest
  let local_p1 = ball.position().inverse().transform_point(world_p1)
  let local_p2 = cuboid.position().inverse().transform_point(world_p2)
  ContactManifold::{
    normal,
    points: [
      ContactManifoldPoint::{
        local_p1,
        local_p2,
        dist: signed_dist,
        fid1: FeatureId::Face(0),
        fid2: feature2,
      },
    ],
  }
}

///|
fn contact_manifold_cuboid_ball(
  cuboid : Collider,
  half_width : @core.Real,
  half_height : @core.Real,
  ball : Collider,
  radius : @core.Real,
) -> ContactManifold {
  let ball_center = ball.world_translation
  let cuboid_center = cuboid.world_translation
  let (projection, feature1) = project_point_on_cuboid_oriented_and_get_feature(
    cuboid_center,
    cuboid.world_rotation,
    half_width,
    half_height,
    ball_center,
    false,
  )
  let closest = projection.point()
  let d = ball_center.sub(closest)
  let dist = d.length()
  let normal = normalize_or_fallback(d, @core.Vec2::new(1.0F, 0.0F))
  let signed_dist = dist - radius
  let world_p1 = closest
  let world_p2 = ball_center.sub(vec2_scale(normal, radius))
  let local_p1 = cuboid.position().inverse().transform_point(world_p1)
  let local_p2 = ball.position().inverse().transform_point(world_p2)
  ContactManifold::{
    normal,
    points: [
      ContactManifoldPoint::{
        local_p1,
        local_p2,
        dist: signed_dist,
        fid1: feature1,
        fid2: FeatureId::Face(0),
      },
    ],
  }
}

///|
priv struct ClipVertex {
  v : @core.Vec2
  id : Int
}

///|
fn clip_segment_to_line(
  input : Array[ClipVertex],
  normal : @core.Vec2,
  offset : @core.Real,
) -> Array[ClipVertex] {
  let output : Array[ClipVertex] = []
  if input.length() != 2 {
    return output
  }
  let v1 = input[0]
  let v2 = input[1]
  let d1 = normal.dot(v1.v) - offset
  let d2 = normal.dot(v2.v) - offset
  if d1 <= 0.0F {
    output.push(v1)
  }
  if d2 <= 0.0F {
    output.push(v2)
  }
  if d1 * d2 < 0.0F {
    let t = d1 / (d1 - d2)
    let dv = v2.v.sub(v1.v)
    let v = v1.v.add(@core.Vec2::new(dv.x * t, dv.y * t))
    let id = if d1 > 0.0F { v1.id } else { v2.id }
    output.push(ClipVertex::{ v, id })
  }
  output
}

///|
fn cuboid_world_vertices(
  center : @core.Vec2,
  rotation : @core.Real,
  half_width : @core.Real,
  half_height : @core.Real,
) -> Array[@core.Vec2] {
  let rot = @core.Rot2::from_angle(rotation)
  // Match `FeatureId::Vertex` convention used by query_pipeline:
  // 0 = (+,+), 1 = (-,+), 2 = (-,-), 3 = (+,-)
  let v0 = center.add(rot.rotate_vec2(@core.Vec2::new(half_width, half_height)))
  let v1 = center.add(
    rot.rotate_vec2(@core.Vec2::new(-half_width, half_height)),
  )
  let v2 = center.add(
    rot.rotate_vec2(@core.Vec2::new(-half_width, -half_height)),
  )
  let v3 = center.add(
    rot.rotate_vec2(@core.Vec2::new(half_width, -half_height)),
  )
  [v0, v1, v2, v3]
}

///|
fn cuboid_edge_endpoints_ccw(edge : Int) -> (Int, Int) {
  // Edge indices follow the convention used by query_pipeline:
  // 0 = x=+hw, 1 = x=-hw, 2 = y=+hh, 3 = y=-hh
  if edge == 0 {
    // v3 -> v0
    (3, 0)
  } else if edge == 1 {
    // v1 -> v2
    (1, 2)
  } else if edge == 2 {
    // v0 -> v1
    (0, 1)
  } else {
    // v2 -> v3
    (2, 3)
  }
}

///|
fn cuboid_feature_from_local_point(
  local_point : @core.Vec2,
  half_width : @core.Real,
  half_height : @core.Real,
) -> FeatureId {
  let on_x = @core.abs(@core.abs(local_point.x) - half_width) <= 1.0e-6F
  let on_y = @core.abs(@core.abs(local_point.y) - half_height) <= 1.0e-6F
  if on_x && on_y {
    let vx = if local_point.x >= 0.0F { 1 } else { 0 }
    let vy = if local_point.y >= 0.0F { 1 } else { 0 }
    let vid = match (vx, vy) {
      (1, 1) => 0
      (0, 1) => 1
      (0, 0) => 2
      (1, 0) => 3
      _ => 0
    }
    FeatureId::Vertex(vid)
  } else if on_x {
    FeatureId::Edge(if local_point.x >= 0.0F { 0 } else { 1 })
  } else {
    FeatureId::Edge(if local_point.y >= 0.0F { 2 } else { 3 })
  }
}

///|
fn capsule_world_segment(
  center : @core.Vec2,
  rotation : @core.Real,
  half_height : @core.Real,
  axis : @core.Vec2,
) -> (@core.Vec2, @core.Vec2) {
  let rot = @core.Rot2::from_angle(rotation)
  let world_axis = rot.rotate_vec2(axis)
  (
    center.sub(vec2_scale(world_axis, half_height)),
    center.add(vec2_scale(world_axis, half_height)),
  )
}

///|
fn closest_point_on_segment_with_param(
  a : @core.Vec2,
  b : @core.Vec2,
  p : @core.Vec2,
) -> (@core.Vec2, @core.Real) {
  let ab = b.sub(a)
  let denom = ab.dot(ab)
  if denom <= 1.0e-12F {
    (a, 0.0F)
  } else {
    let t = clamp_value(p.sub(a).dot(ab) / denom, 0.0F, 1.0F)
    (a.add(@core.Vec2::new(ab.x * t, ab.y * t)), t)
  }
}

///|
fn closest_points_segment_segment(
  p1 : @core.Vec2,
  q1 : @core.Vec2,
  p2 : @core.Vec2,
  q2 : @core.Vec2,
) -> (@core.Vec2, @core.Vec2, @core.Real, @core.Real) {
  // Algorithm adapted from "Real-Time Collision Detection" (Christer Ericson).
  let d1 = q1.sub(p1)
  let d2 = q2.sub(p2)
  let r = p1.sub(p2)
  let a = d1.dot(d1)
  let e = d2.dot(d2)
  let f = d2.dot(r)
  let mut s = 0.0F
  let mut t = 0.0F
  if a <= 1.0e-12F && e <= 1.0e-12F {
    (p1, p2, 0.0F, 0.0F)
  } else if a <= 1.0e-12F {
    s = 0.0F
    t = clamp_value(f / e, 0.0F, 1.0F)
    (p1, p2.add(@core.Vec2::new(d2.x * t, d2.y * t)), s, t)
  } else {
    let c = d1.dot(r)
    if e <= 1.0e-12F {
      t = 0.0F
      s = clamp_value(-c / a, 0.0F, 1.0F)
    } else {
      let b = d1.dot(d2)
      let denom = a * e - b * b
      if denom != 0.0F {
        s = clamp_value((b * f - c * e) / denom, 0.0F, 1.0F)
      } else {
        s = 0.0F
      }
      let tnom = b * s + f
      if tnom < 0.0F {
        t = 0.0F
        s = clamp_value(-c / a, 0.0F, 1.0F)
      } else if tnom > e {
        t = 1.0F
        s = clamp_value((b - c) / a, 0.0F, 1.0F)
      } else {
        t = tnom / e
      }
    }
    (
      p1.add(@core.Vec2::new(d1.x * s, d1.y * s)),
      p2.add(@core.Vec2::new(d2.x * t, d2.y * t)),
      s,
      t,
    )
  }
}

///|
fn segment_aabb_intersection_params(
  a : @core.Vec2,
  b : @core.Vec2,
  half_width : @core.Real,
  half_height : @core.Real,
) -> (@core.Real, @core.Real)? {
  // Liang–Barsky clipping against axis-aligned AABB centered at origin.
  let d = b.sub(a)
  let mut t0 = 0.0F
  let mut t1 = 1.0F
  let p = [-d.x, d.x, -d.y, d.y]
  let q = [
    a.x + half_width,
    half_width - a.x,
    a.y + half_height,
    half_height - a.y,
  ]
  for i in 0..<4 {
    let pi = p[i]
    let qi = q[i]
    if @core.abs(pi) <= 1.0e-12F {
      if qi < 0.0F {
        return None
      }
    } else {
      let r = qi / pi
      if pi < 0.0F {
        if r > t1 {
          return None
        }
        if r > t0 {
          t0 = r
        }
      } else {
        if r < t0 {
          return None
        }
        if r < t1 {
          t1 = r
        }
      }
    }
  }
  Some((t0, t1))
}

///|
fn closest_points_segment_aabb_local(
  a : @core.Vec2,
  b : @core.Vec2,
  half_width : @core.Real,
  half_height : @core.Real,
) -> (@core.Vec2, @core.Vec2, @core.Real, FeatureId, FeatureId) {
  if segment_aabb_intersection_params(a, b, half_width, half_height)
    is Some((t0, t1)) {
    let d = b.sub(a)
    // Pick a deterministic intersection point on the boundary (entry point).
    let seg_p = a.add(@core.Vec2::new(d.x * t0, d.y * t0))
    let box_p = seg_p
    let seg_feature = if t0 <= 1.0e-6F {
      FeatureId::Vertex(0)
    } else if t0 >= 1.0F - 1.0e-6F {
      FeatureId::Vertex(1)
    } else {
      FeatureId::Edge(0)
    }
    let box_feature = cuboid_feature_from_local_point(
      box_p, half_width, half_height,
    )
    t1 |> ignore
    return (seg_p, box_p, 0.0F, seg_feature, box_feature)
  }

  // Compute the minimum distance between the segment and each AABB edge segment.
  let edges = [
    // x=+hw
    (
      @core.Vec2::new(half_width, -half_height),
      @core.Vec2::new(half_width, half_height),
      FeatureId::Edge(0),
    ),
    // x=-hw
    (
      @core.Vec2::new(-half_width, half_height),
      @core.Vec2::new(-half_width, -half_height),
      FeatureId::Edge(1),
    ),
    // y=+hh
    (
      @core.Vec2::new(half_width, half_height),
      @core.Vec2::new(-half_width, half_height),
      FeatureId::Edge(2),
    ),
    // y=-hh
    (
      @core.Vec2::new(-half_width, -half_height),
      @core.Vec2::new(half_width, -half_height),
      FeatureId::Edge(3),
    ),
  ]
  let mut best_dist2 = 1.0e30F
  let mut best_seg = a
  let mut best_box = @core.Vec2::new(half_width, half_height)
  let mut best_s = 0.0F
  for i in 0..<edges.length() {
    let e = edges[i]
    let (ps, pb, s, _) = closest_points_segment_segment(a, b, e.0, e.1)
    let delta = pb.sub(ps)
    let dist2 = delta.dot(delta)
    if dist2 < best_dist2 {
      best_dist2 = dist2
      best_seg = ps
      best_box = pb
      best_s = s
    }
  }
  let seg_feature = if best_s <= 1.0e-6F {
    FeatureId::Vertex(0)
  } else if best_s >= 1.0F - 1.0e-6F {
    FeatureId::Vertex(1)
  } else {
    FeatureId::Edge(0)
  }
  let box_feature = cuboid_feature_from_local_point(
    best_box, half_width, half_height,
  )
  (best_seg, best_box, Float::sqrt(best_dist2), seg_feature, box_feature)
}

///|
fn contact_manifold_cuboid_cuboid(
  cuboid1 : Collider,
  half_width1 : @core.Real,
  half_height1 : @core.Real,
  cuboid2 : Collider,
  half_width2 : @core.Real,
  half_height2 : @core.Real,
  prediction_distance : @core.Real,
) -> ContactManifold {
  let (ax1, ay1) = cuboid_axes(cuboid1.world_rotation)
  let (ax2, ay2) = cuboid_axes(cuboid2.world_rotation)
  let delta = cuboid2.world_translation.sub(cuboid1.world_translation)
  let axes = [ax1, ay1, ax2, ay2]
  let mut best_overlap = 1.0e30F
  let mut best_axis = ax1
  let mut best_axis_owner = 0 // 0 = cuboid1, 1 = cuboid2
  let mut best_axis_id = 0 // 0 = x, 1 = y
  for i in 0..<axes.length() {
    let axis = axes[i]
    let proj = @core.abs(delta.dot(axis))
    let r1 = projection_radius(axis, ax1, ay1, half_width1, half_height1)
    let r2 = projection_radius(axis, ax2, ay2, half_width2, half_height2)
    let overlap = r1 + r2 - proj
    if overlap < best_overlap {
      best_overlap = overlap
      best_axis = axis
      best_axis_owner = if i < 2 { 0 } else { 1 }
      best_axis_id = if i % 2 == 0 { 0 } else { 1 }
    }
  }
  let sign = if delta.dot(best_axis) < 0.0F { -1.0F } else { 1.0F }
  let normal12 = vec2_scale(best_axis, sign)
  let reference_is_1 = best_axis_owner == 0
  let ref_normal = if reference_is_1 {
    normal12
  } else {
    @core.Vec2::new(-normal12.x, -normal12.y)
  }
  let ref_edge_id = if best_axis_id == 0 {
    if ref_normal.dot(if reference_is_1 { ax1 } else { ax2 }) >= 0.0F {
      0
    } else {
      1
    }
  } else if ref_normal.dot(if reference_is_1 { ay1 } else { ay2 }) >= 0.0F {
    2
  } else {
    3
  }
  let ref_verts = if reference_is_1 {
    cuboid_world_vertices(
      cuboid1.world_translation,
      cuboid1.world_rotation,
      half_width1,
      half_height1,
    )
  } else {
    cuboid_world_vertices(
      cuboid2.world_translation,
      cuboid2.world_rotation,
      half_width2,
      half_height2,
    )
  }
  let (rv1i, rv2i) = cuboid_edge_endpoints_ccw(ref_edge_id)
  let rv1 = ref_verts[rv1i]
  let rv2 = ref_verts[rv2i]
  let ref_tangent = normalize_or_fallback(
    rv2.sub(rv1),
    @core.Vec2::new(1.0F, 0.0F),
  )
  let side_normal1 = @core.Vec2::new(-ref_tangent.x, -ref_tangent.y)
  let side_normal2 = ref_tangent
  let clip1 = clip_segment_to_line(
    {
      let (iax, iay) = if reference_is_1 { (ax2, ay2) } else { (ax1, ay1) }
      let mut best_dot = 1.0e30F
      let mut incident_edge_id = 0
      let normals = [
        iax,
        @core.Vec2::new(-iax.x, -iax.y),
        iay,
        @core.Vec2::new(-iay.x, -iay.y),
      ]
      for k in 0..<4 {
        let d = normals[k].dot(ref_normal)
        if d < best_dot {
          best_dot = d
          incident_edge_id = k
        }
      }
      let iverts = if reference_is_1 {
        cuboid_world_vertices(
          cuboid2.world_translation,
          cuboid2.world_rotation,
          half_width2,
          half_height2,
        )
      } else {
        cuboid_world_vertices(
          cuboid1.world_translation,
          cuboid1.world_rotation,
          half_width1,
          half_height1,
        )
      }
      let (iv1i, iv2i) = cuboid_edge_endpoints_ccw(incident_edge_id)
      [
        ClipVertex::{ v: iverts[iv1i], id: iv1i },
        ClipVertex::{ v: iverts[iv2i], id: iv2i },
      ]
    },
    side_normal1,
    side_normal1.dot(rv1),
  )
  let clip2 = clip_segment_to_line(clip1, side_normal2, side_normal2.dot(rv2))
  let front_offset = ref_normal.dot(rv1)
  let points : Array[ContactManifoldPoint] = []
  for i in 0..<clip2.length() {
    let cv = clip2[i]
    let separation = ref_normal.dot(cv.v) - front_offset
    if separation <= prediction_distance + 1.0e-6F {
      let cp_ref = cv.v.sub(vec2_scale(ref_normal, separation))
      let (world_p1, world_p2, fid1, fid2) = if reference_is_1 {
        (cp_ref, cv.v, FeatureId::Edge(ref_edge_id), FeatureId::Vertex(cv.id))
      } else {
        (cv.v, cp_ref, FeatureId::Vertex(cv.id), FeatureId::Edge(ref_edge_id))
      }
      points.push(ContactManifoldPoint::{
        local_p1: cuboid1.position().inverse().transform_point(world_p1),
        local_p2: cuboid2.position().inverse().transform_point(world_p2),
        dist: separation,
        fid1,
        fid2,
      })
    }
  }

  // Deterministic ordering along the reference tangent.
  if points.length() == 2 {
    let p0 = cuboid1.position().transform_point(points[0].local_p1)
    let p1 = cuboid1.position().transform_point(points[1].local_p1)
    if p0.dot(ref_tangent) > p1.dot(ref_tangent) {
      let tmp = points[0]
      points[0] = points[1]
      points[1] = tmp
    }
  }
  ContactManifold::{ normal: normal12, points }
}

///|
fn contact_manifold_ball_segment(
  ball : Collider,
  radius : @core.Real,
  segment : Collider,
  a : @core.Vec2,
  b : @core.Vec2,
) -> ContactManifold {
  let ball_center = ball.world_translation
  let (projection, seg_feature) = project_point_on_segment_and_get_feature(
    segment.world_translation,
    segment.world_rotation,
    a,
    b,
    ball_center,
  )
  let closest = projection.point()
  let d = closest.sub(ball_center)
  let dist = d.length()
  let normal = normalize_or_fallback(d, @core.Vec2::new(1.0F, 0.0F))
  let signed_dist = dist - radius
  let world_p1 = ball_center.add(vec2_scale(normal, radius))
  let world_p2 = closest
  ContactManifold::{
    normal,
    points: [
      ContactManifoldPoint::{
        local_p1: ball.position().inverse().transform_point(world_p1),
        local_p2: segment.position().inverse().transform_point(world_p2),
        dist: signed_dist,
        fid1: FeatureId::Face(0),
        fid2: seg_feature,
      },
    ],
  }
}

///|
fn contact_manifold_segment_ball(
  segment : Collider,
  a : @core.Vec2,
  b : @core.Vec2,
  ball : Collider,
  radius : @core.Real,
) -> ContactManifold {
  let ball_center = ball.world_translation
  let (projection, seg_feature) = project_point_on_segment_and_get_feature(
    segment.world_translation,
    segment.world_rotation,
    a,
    b,
    ball_center,
  )
  let closest = projection.point()
  let d = ball_center.sub(closest)
  let dist = d.length()
  let normal = normalize_or_fallback(d, @core.Vec2::new(1.0F, 0.0F))
  let signed_dist = dist - radius
  let world_p1 = closest
  let world_p2 = ball_center.sub(vec2_scale(normal, radius))
  ContactManifold::{
    normal,
    points: [
      ContactManifoldPoint::{
        local_p1: segment.position().inverse().transform_point(world_p1),
        local_p2: ball.position().inverse().transform_point(world_p2),
        dist: signed_dist,
        fid1: seg_feature,
        fid2: FeatureId::Face(0),
      },
    ],
  }
}

///|
fn capsule_segment_feature(t : @core.Real) -> FeatureId {
  if t <= 1.0e-6F {
    FeatureId::Vertex(0)
  } else if t >= 1.0F - 1.0e-6F {
    FeatureId::Vertex(1)
  } else {
    FeatureId::Edge(0)
  }
}

///|
fn contact_manifold_ball_capsule(
  ball : Collider,
  radius_ball : @core.Real,
  capsule : Collider,
  half_height : @core.Real,
  radius_capsule : @core.Real,
  axis : @core.Vec2,
) -> ContactManifold {
  let ball_center = ball.world_translation
  let (sa, sb) = capsule_world_segment(
    capsule.world_translation,
    capsule.world_rotation,
    half_height,
    axis,
  )
  let (q, t) = closest_point_on_segment_with_param(sa, sb, ball_center)
  let d = q.sub(ball_center)
  let dist = d.length()
  let normal = normalize_or_fallback(d, @core.Vec2::new(1.0F, 0.0F))
  let signed_dist = dist - (radius_ball + radius_capsule)
  let world_p1 = ball_center.add(vec2_scale(normal, radius_ball))
  let world_p2 = q.sub(vec2_scale(normal, radius_capsule))
  ContactManifold::{
    normal,
    points: [
      ContactManifoldPoint::{
        local_p1: ball.position().inverse().transform_point(world_p1),
        local_p2: capsule.position().inverse().transform_point(world_p2),
        dist: signed_dist,
        fid1: FeatureId::Face(0),
        fid2: capsule_segment_feature(t),
      },
    ],
  }
}

///|
fn contact_manifold_capsule_ball(
  capsule : Collider,
  half_height : @core.Real,
  radius_capsule : @core.Real,
  axis : @core.Vec2,
  ball : Collider,
  radius_ball : @core.Real,
) -> ContactManifold {
  let ball_center = ball.world_translation
  let (sa, sb) = capsule_world_segment(
    capsule.world_translation,
    capsule.world_rotation,
    half_height,
    axis,
  )
  let (q, t) = closest_point_on_segment_with_param(sa, sb, ball_center)
  let d = ball_center.sub(q)
  let dist = d.length()
  let normal = normalize_or_fallback(d, @core.Vec2::new(1.0F, 0.0F))
  let signed_dist = dist - (radius_ball + radius_capsule)
  let world_p1 = q.add(vec2_scale(normal, radius_capsule))
  let world_p2 = ball_center.sub(vec2_scale(normal, radius_ball))
  ContactManifold::{
    normal,
    points: [
      ContactManifoldPoint::{
        local_p1: capsule.position().inverse().transform_point(world_p1),
        local_p2: ball.position().inverse().transform_point(world_p2),
        dist: signed_dist,
        fid1: capsule_segment_feature(t),
        fid2: FeatureId::Face(0),
      },
    ],
  }
}

///|
fn contact_manifold_capsule_capsule(
  capsule1 : Collider,
  half_height1 : @core.Real,
  radius1 : @core.Real,
  axis1 : @core.Vec2,
  capsule2 : Collider,
  half_height2 : @core.Real,
  radius2 : @core.Real,
  axis2 : @core.Vec2,
) -> ContactManifold {
  let (a1, b1) = capsule_world_segment(
    capsule1.world_translation,
    capsule1.world_rotation,
    half_height1,
    axis1,
  )
  let (a2, b2) = capsule_world_segment(
    capsule2.world_translation,
    capsule2.world_rotation,
    half_height2,
    axis2,
  )
  let (p1, p2, s, t) = closest_points_segment_segment(a1, b1, a2, b2)
  let d = p2.sub(p1)
  let dist = d.length()
  let normal = normalize_or_fallback(d, @core.Vec2::new(1.0F, 0.0F))
  let signed_dist = dist - (radius1 + radius2)
  let world_p1 = p1.add(vec2_scale(normal, radius1))
  let world_p2 = p2.sub(vec2_scale(normal, radius2))
  ContactManifold::{
    normal,
    points: [
      ContactManifoldPoint::{
        local_p1: capsule1.position().inverse().transform_point(world_p1),
        local_p2: capsule2.position().inverse().transform_point(world_p2),
        dist: signed_dist,
        fid1: capsule_segment_feature(s),
        fid2: capsule_segment_feature(t),
      },
    ],
  }
}

///|
fn contact_manifold_segment_cuboid(
  segment : Collider,
  a : @core.Vec2,
  b : @core.Vec2,
  cuboid : Collider,
  half_width : @core.Real,
  half_height : @core.Real,
) -> ContactManifold {
  let (sa, sb) = segment_world_endpoints(
    segment.world_translation,
    segment.world_rotation,
    a,
    b,
  )
  let rot = @core.Rot2::from_angle(cuboid.world_rotation)
  let inv = rot.inverse()
  let la = inv.rotate_vec2(sa.sub(cuboid.world_translation))
  let lb = inv.rotate_vec2(sb.sub(cuboid.world_translation))
  let (seg_l, box_l, dist_center, seg_f, box_f) = closest_points_segment_aabb_local(
    la, lb, half_width, half_height,
  )
  let box_world = cuboid.world_translation.add(rot.rotate_vec2(box_l))
  let seg_world = cuboid.world_translation.add(rot.rotate_vec2(seg_l))
  let d = box_world.sub(seg_world)
  let normal = if dist_center <= 1.0e-12F {
    let n_local = match box_f {
      FeatureId::Edge(id) =>
        if id == 0 {
          @core.Vec2::new(1.0F, 0.0F)
        } else if id == 1 {
          @core.Vec2::new(-1.0F, 0.0F)
        } else if id == 2 {
          @core.Vec2::new(0.0F, 1.0F)
        } else {
          @core.Vec2::new(0.0F, -1.0F)
        }
      _ => @core.Vec2::new(1.0F, 0.0F)
    }
    let mut n = rot.rotate_vec2(n_local)
    if n.dot(cuboid.world_translation.sub(segment.world_translation)) < 0.0F {
      n = @core.Vec2::new(-n.x, -n.y)
    }
    n
  } else {
    normalize_or_fallback(d, @core.Vec2::new(1.0F, 0.0F))
  }
  ContactManifold::{
    normal,
    points: [
      ContactManifoldPoint::{
        local_p1: segment.position().inverse().transform_point(seg_world),
        local_p2: cuboid.position().inverse().transform_point(box_world),
        dist: dist_center,
        fid1: seg_f,
        fid2: box_f,
      },
    ],
  }
}

///|
fn contact_manifold_cuboid_segment(
  cuboid : Collider,
  half_width : @core.Real,
  half_height : @core.Real,
  segment : Collider,
  a : @core.Vec2,
  b : @core.Vec2,
) -> ContactManifold {
  let m = contact_manifold_segment_cuboid(
    segment, a, b, cuboid, half_width, half_height,
  )
  // Flip normal and swap feature ids/points into cuboid/segment order.
  let points : Array[ContactManifoldPoint] = []
  let ps = m.points()
  for i in 0..<ps.length() {
    let p = ps[i]
    points.push(ContactManifoldPoint::{
      local_p1: p.local_p2(),
      local_p2: p.local_p1(),
      dist: p.dist(),
      fid1: p.fid2(),
      fid2: p.fid1(),
    })
  }
  ContactManifold::{
    normal: @core.Vec2::new(-m.normal().x, -m.normal().y),
    points,
  }
}

///|
fn contact_manifold_segment_segment(
  seg1 : Collider,
  a1 : @core.Vec2,
  b1 : @core.Vec2,
  seg2 : Collider,
  a2 : @core.Vec2,
  b2 : @core.Vec2,
) -> ContactManifold {
  let (p1, q1) = segment_world_endpoints(
    seg1.world_translation,
    seg1.world_rotation,
    a1,
    b1,
  )
  let (p2, q2) = segment_world_endpoints(
    seg2.world_translation,
    seg2.world_rotation,
    a2,
    b2,
  )
  let (c1, c2, s, t) = closest_points_segment_segment(p1, q1, p2, q2)
  let d = c2.sub(c1)
  let dist = d.length()
  let normal = normalize_or_fallback(d, @core.Vec2::new(1.0F, 0.0F))
  let fid1 = if s <= 1.0e-6F {
    FeatureId::Vertex(0)
  } else if s >= 1.0F - 1.0e-6F {
    FeatureId::Vertex(1)
  } else {
    FeatureId::Edge(0)
  }
  let fid2 = if t <= 1.0e-6F {
    FeatureId::Vertex(0)
  } else if t >= 1.0F - 1.0e-6F {
    FeatureId::Vertex(1)
  } else {
    FeatureId::Edge(0)
  }
  ContactManifold::{
    normal,
    points: [
      ContactManifoldPoint::{
        local_p1: seg1.position().inverse().transform_point(c1),
        local_p2: seg2.position().inverse().transform_point(c2),
        dist,
        fid1,
        fid2,
      },
    ],
  }
}

///|
fn contact_manifold_segment_capsule(
  segment : Collider,
  a : @core.Vec2,
  b : @core.Vec2,
  capsule : Collider,
  half_height : @core.Real,
  radius : @core.Real,
  axis : @core.Vec2,
) -> ContactManifold {
  let (sa, sb) = segment_world_endpoints(
    segment.world_translation,
    segment.world_rotation,
    a,
    b,
  )
  let (ca, cb) = capsule_world_segment(
    capsule.world_translation,
    capsule.world_rotation,
    half_height,
    axis,
  )
  let (pseg, pcap, s, t) = closest_points_segment_segment(sa, sb, ca, cb)
  let d = pcap.sub(pseg)
  let dist = d.length()
  let normal = normalize_or_fallback(d, @core.Vec2::new(1.0F, 0.0F))
  let signed_dist = dist - radius
  let world_p1 = pseg
  let world_p2 = pcap.sub(vec2_scale(normal, radius))
  let fid1 = if s <= 1.0e-6F {
    FeatureId::Vertex(0)
  } else if s >= 1.0F - 1.0e-6F {
    FeatureId::Vertex(1)
  } else {
    FeatureId::Edge(0)
  }
  ContactManifold::{
    normal,
    points: [
      ContactManifoldPoint::{
        local_p1: segment.position().inverse().transform_point(world_p1),
        local_p2: capsule.position().inverse().transform_point(world_p2),
        dist: signed_dist,
        fid1,
        fid2: capsule_segment_feature(t),
      },
    ],
  }
}

///|
fn contact_manifold_capsule_segment(
  capsule : Collider,
  half_height : @core.Real,
  radius : @core.Real,
  axis : @core.Vec2,
  segment : Collider,
  a : @core.Vec2,
  b : @core.Vec2,
) -> ContactManifold {
  let m = contact_manifold_segment_capsule(
    segment, a, b, capsule, half_height, radius, axis,
  )
  let points : Array[ContactManifoldPoint] = []
  let ps = m.points()
  for i in 0..<ps.length() {
    let p = ps[i]
    points.push(ContactManifoldPoint::{
      local_p1: p.local_p2(),
      local_p2: p.local_p1(),
      dist: p.dist(),
      fid1: p.fid2(),
      fid2: p.fid1(),
    })
  }
  ContactManifold::{
    normal: @core.Vec2::new(-m.normal().x, -m.normal().y),
    points,
  }
}

///|
fn contact_manifold_capsule_cuboid(
  capsule : Collider,
  half_height : @core.Real,
  radius : @core.Real,
  axis : @core.Vec2,
  cuboid : Collider,
  half_width : @core.Real,
  half_height2 : @core.Real,
) -> ContactManifold {
  let (ca, cb) = capsule_world_segment(
    capsule.world_translation,
    capsule.world_rotation,
    half_height,
    axis,
  )
  let rot = @core.Rot2::from_angle(cuboid.world_rotation)
  let inv = rot.inverse()
  let la = inv.rotate_vec2(ca.sub(cuboid.world_translation))
  let lb = inv.rotate_vec2(cb.sub(cuboid.world_translation))
  let (seg_l, box_l, dist_center, _seg_f, box_f) = closest_points_segment_aabb_local(
    la, lb, half_width, half_height2,
  )
  let box_world = cuboid.world_translation.add(rot.rotate_vec2(box_l))
  let seg_world = cuboid.world_translation.add(rot.rotate_vec2(seg_l))
  let d = box_world.sub(seg_world)
  let mut normal = if dist_center <= 1.0e-12F {
    let n_local = match box_f {
      FeatureId::Edge(id) =>
        if id == 0 {
          @core.Vec2::new(1.0F, 0.0F)
        } else if id == 1 {
          @core.Vec2::new(-1.0F, 0.0F)
        } else if id == 2 {
          @core.Vec2::new(0.0F, 1.0F)
        } else {
          @core.Vec2::new(0.0F, -1.0F)
        }
      _ => @core.Vec2::new(1.0F, 0.0F)
    }
    rot.rotate_vec2(n_local)
  } else {
    normalize_or_fallback(d, @core.Vec2::new(1.0F, 0.0F))
  }
  if normal.dot(cuboid.world_translation.sub(capsule.world_translation)) < 0.0F {
    normal = @core.Vec2::new(-normal.x, -normal.y)
  }
  let signed_dist = dist_center - radius
  let world_p1 = seg_world.add(vec2_scale(normal, radius))
  let world_p2 = box_world

  // Compute param along capsule segment for a stable feature id.
  let (ta, tb) = capsule_world_segment(
    capsule.world_translation,
    capsule.world_rotation,
    half_height,
    axis,
  )
  let (_, t) = closest_point_on_segment_with_param(ta, tb, seg_world)
  ContactManifold::{
    normal,
    points: [
      ContactManifoldPoint::{
        local_p1: capsule.position().inverse().transform_point(world_p1),
        local_p2: cuboid.position().inverse().transform_point(world_p2),
        dist: signed_dist,
        fid1: capsule_segment_feature(t),
        fid2: box_f,
      },
    ],
  }
}

///|
fn contact_manifold_cuboid_capsule(
  cuboid : Collider,
  half_width : @core.Real,
  half_height : @core.Real,
  capsule : Collider,
  half_height2 : @core.Real,
  radius : @core.Real,
  axis : @core.Vec2,
) -> ContactManifold {
  let m = contact_manifold_capsule_cuboid(
    capsule, half_height2, radius, axis, cuboid, half_width, half_height,
  )
  let points : Array[ContactManifoldPoint] = []
  let ps = m.points()
  for i in 0..<ps.length() {
    let p = ps[i]
    points.push(ContactManifoldPoint::{
      local_p1: p.local_p2(),
      local_p2: p.local_p1(),
      dist: p.dist(),
      fid1: p.fid2(),
      fid2: p.fid1(),
    })
  }
  ContactManifold::{
    normal: @core.Vec2::new(-m.normal().x, -m.normal().y),
    points,
  }
}

///|
fn build_contact_pair(
  prediction_distance : @core.Real,
  collider1 : Collider,
  collider2 : Collider,
) -> ContactPair {
  let manifolds : Array[ContactManifold] = []
  match (collider1.shape, collider2.shape) {
    (Shape::Ball(r1), Shape::Ball(r2)) =>
      manifolds.push(contact_manifold_ball_ball(collider1, r1, collider2, r2))
    (Shape::Ball(r), Shape::Cuboid(hw, hh)) =>
      manifolds.push(
        contact_manifold_ball_cuboid(collider1, r, collider2, hw, hh),
      )
    (Shape::Cuboid(hw, hh), Shape::Ball(r)) =>
      manifolds.push(
        contact_manifold_cuboid_ball(collider1, hw, hh, collider2, r),
      )
    (Shape::Cuboid(hw1, hh1), Shape::Cuboid(hw2, hh2)) =>
      manifolds.push(
        contact_manifold_cuboid_cuboid(
          collider1, hw1, hh1, collider2, hw2, hh2, prediction_distance,
        ),
      )
    (Shape::Ball(r), Shape::Segment(a, b)) =>
      manifolds.push(
        contact_manifold_ball_segment(collider1, r, collider2, a, b),
      )
    (Shape::Segment(a, b), Shape::Ball(r)) =>
      manifolds.push(
        contact_manifold_segment_ball(collider1, a, b, collider2, r),
      )
    (Shape::Ball(r), Shape::CapsuleX(hh, cr)) =>
      manifolds.push(
        contact_manifold_ball_capsule(
          collider1,
          r,
          collider2,
          hh,
          cr,
          @core.Vec2::new(1.0F, 0.0F),
        ),
      )
    (Shape::Ball(r), Shape::CapsuleY(hh, cr)) =>
      manifolds.push(
        contact_manifold_ball_capsule(
          collider1,
          r,
          collider2,
          hh,
          cr,
          @core.Vec2::new(0.0F, 1.0F),
        ),
      )
    (Shape::CapsuleX(hh, cr), Shape::Ball(r)) =>
      manifolds.push(
        contact_manifold_capsule_ball(
          collider1,
          hh,
          cr,
          @core.Vec2::new(1.0F, 0.0F),
          collider2,
          r,
        ),
      )
    (Shape::CapsuleY(hh, cr), Shape::Ball(r)) =>
      manifolds.push(
        contact_manifold_capsule_ball(
          collider1,
          hh,
          cr,
          @core.Vec2::new(0.0F, 1.0F),
          collider2,
          r,
        ),
      )
    (Shape::CapsuleX(hh1, r1), Shape::CapsuleX(hh2, r2)) =>
      manifolds.push(
        contact_manifold_capsule_capsule(
          collider1,
          hh1,
          r1,
          @core.Vec2::new(1.0F, 0.0F),
          collider2,
          hh2,
          r2,
          @core.Vec2::new(1.0F, 0.0F),
        ),
      )
    (Shape::CapsuleX(hh1, r1), Shape::CapsuleY(hh2, r2)) =>
      manifolds.push(
        contact_manifold_capsule_capsule(
          collider1,
          hh1,
          r1,
          @core.Vec2::new(1.0F, 0.0F),
          collider2,
          hh2,
          r2,
          @core.Vec2::new(0.0F, 1.0F),
        ),
      )
    (Shape::CapsuleY(hh1, r1), Shape::CapsuleX(hh2, r2)) =>
      manifolds.push(
        contact_manifold_capsule_capsule(
          collider1,
          hh1,
          r1,
          @core.Vec2::new(0.0F, 1.0F),
          collider2,
          hh2,
          r2,
          @core.Vec2::new(1.0F, 0.0F),
        ),
      )
    (Shape::CapsuleY(hh1, r1), Shape::CapsuleY(hh2, r2)) =>
      manifolds.push(
        contact_manifold_capsule_capsule(
          collider1,
          hh1,
          r1,
          @core.Vec2::new(0.0F, 1.0F),
          collider2,
          hh2,
          r2,
          @core.Vec2::new(0.0F, 1.0F),
        ),
      )
    (Shape::Segment(a, b), Shape::Cuboid(hw, hh)) =>
      manifolds.push(
        contact_manifold_segment_cuboid(collider1, a, b, collider2, hw, hh),
      )
    (Shape::Cuboid(hw, hh), Shape::Segment(a, b)) =>
      manifolds.push(
        contact_manifold_cuboid_segment(collider1, hw, hh, collider2, a, b),
      )
    (Shape::Segment(a1, b1), Shape::Segment(a2, b2)) =>
      manifolds.push(
        contact_manifold_segment_segment(collider1, a1, b1, collider2, a2, b2),
      )
    (Shape::Segment(a, b), Shape::CapsuleX(hh, cr)) =>
      manifolds.push(
        contact_manifold_segment_capsule(
          collider1,
          a,
          b,
          collider2,
          hh,
          cr,
          @core.Vec2::new(1.0F, 0.0F),
        ),
      )
    (Shape::Segment(a, b), Shape::CapsuleY(hh, cr)) =>
      manifolds.push(
        contact_manifold_segment_capsule(
          collider1,
          a,
          b,
          collider2,
          hh,
          cr,
          @core.Vec2::new(0.0F, 1.0F),
        ),
      )
    (Shape::CapsuleX(hh, cr), Shape::Segment(a, b)) =>
      manifolds.push(
        contact_manifold_capsule_segment(
          collider1,
          hh,
          cr,
          @core.Vec2::new(1.0F, 0.0F),
          collider2,
          a,
          b,
        ),
      )
    (Shape::CapsuleY(hh, cr), Shape::Segment(a, b)) =>
      manifolds.push(
        contact_manifold_capsule_segment(
          collider1,
          hh,
          cr,
          @core.Vec2::new(0.0F, 1.0F),
          collider2,
          a,
          b,
        ),
      )
    (Shape::CapsuleX(hh, cr), Shape::Cuboid(hw, hh2)) =>
      manifolds.push(
        contact_manifold_capsule_cuboid(
          collider1,
          hh,
          cr,
          @core.Vec2::new(1.0F, 0.0F),
          collider2,
          hw,
          hh2,
        ),
      )
    (Shape::CapsuleY(hh, cr), Shape::Cuboid(hw, hh2)) =>
      manifolds.push(
        contact_manifold_capsule_cuboid(
          collider1,
          hh,
          cr,
          @core.Vec2::new(0.0F, 1.0F),
          collider2,
          hw,
          hh2,
        ),
      )
    (Shape::Cuboid(hw, hh2), Shape::CapsuleX(hh, cr)) =>
      manifolds.push(
        contact_manifold_cuboid_capsule(
          collider1,
          hw,
          hh2,
          collider2,
          hh,
          cr,
          @core.Vec2::new(1.0F, 0.0F),
        ),
      )
    (Shape::Cuboid(hw, hh2), Shape::CapsuleY(hh, cr)) =>
      manifolds.push(
        contact_manifold_cuboid_capsule(
          collider1,
          hw,
          hh2,
          collider2,
          hh,
          cr,
          @core.Vec2::new(0.0F, 1.0F),
        ),
      )
  }
  ContactPair::{ manifolds, }
}

///|
pub fn NarrowPhase::update_with_pairs(
  self : NarrowPhase,
  bodies : @dynamics.RigidBodySet,
  colliders : ColliderSet,
  prediction_distance : @core.Real,
  pairs : Array[(ColliderHandle, ColliderHandle)],
  impulse_joints : @dynamics.ImpulseJointSet,
  multibody_joints : @dynamics.MultibodyJointSet,
) -> Unit {
  multibody_joints |> ignore
  let pd = if prediction_distance < 0.0F { 0.0F } else { prediction_distance }
  let intersections : Array[IntersectionPair] = []
  let contacts : Array[(ColliderHandle, ColliderHandle, ContactPair)] = []
  for i in 0..<pairs.length() {
    let pair = pairs[i]
    let handle1 = pair.0
    let handle2 = pair.1
    if colliders.get(handle1) is Some(collider1) &&
      colliders.get(handle2) is Some(collider2) {
      if !collider1.is_enabled() || !collider2.is_enabled() {
        continue
      }
      if is_same_parent(collider1, collider2) {
        continue
      }
      // Deal with contacts disabled between bodies attached by impulse joints (Rapier parity).
      if collider1.parent is Some(parent1) && collider2.parent is Some(parent2) {
        if !@dynamics.RigidBodyHandle::equals(parent1, parent2) {
          let joints = impulse_joints.joints_between(parent1, parent2)
          let mut disabled = false
          for j in 0..<joints.length() {
            let joint = joints[j].1
            if !joint.data.contacts_enabled {
              disabled = true
              break
            }
          }
          if disabled {
            continue
          }
        }
      }
      let mut rb_type1 = @dynamics.RigidBodyType::Fixed
      let mut rb_type2 = @dynamics.RigidBodyType::Fixed
      if collider1.parent is Some(parent1) {
        if bodies.get(parent1) is Some(body1) {
          rb_type1 = body1.body_type()
        }
      }
      if collider2.parent is Some(parent2) {
        if bodies.get(parent2) is Some(body2) {
          rb_type2 = body2.body_type()
        }
      }
      if !collider1.active_collision_types.test_body_types(rb_type1, rb_type2) &&
        !collider2.active_collision_types.test_body_types(rb_type1, rb_type2) {
        continue
      }
      if !collider1.collision_groups.test_groups(collider2.collision_groups) {
        continue
      }
      let pos1 = collider1.world_translation
      let pos2 = collider2.world_translation
      let intersecting = shapes_intersect(
        collider1.shape,
        pos1,
        collider1.world_rotation,
        collider2.shape,
        pos2,
        collider2.world_rotation,
      )
      if intersecting {
        if collider1.sensor || collider2.sensor {
          intersections.push(IntersectionPair::{
            collider1: handle1,
            collider2: handle2,
            intersecting,
          })
        } else {
          let contact = build_contact_pair(pd, collider1, collider2)
          let mut keep = false
          for mi in 0..<contact.manifolds.length() {
            let manifold = contact.manifolds[mi]
            for pi in 0..<manifold.points.length() {
              if manifold.points[pi].dist <= pd {
                keep = true
                break
              }
            }
            if keep {
              break
            }
          }
          if keep {
            contacts.push((handle1, handle2, contact))
          }
        }
      }
    }
  }
  self.intersections = intersections
  self.contacts = contacts
}

///|
pub fn NarrowPhase::intersection_pairs_with(
  self : NarrowPhase,
  handle : ColliderHandle,
) -> Array[IntersectionPair] {
  let result : Array[IntersectionPair] = []
  for i in 0..<self.intersections.length() {
    let pair = self.intersections[i]
    let matches = collider_handle_equals(pair.collider1, handle) ||
      collider_handle_equals(pair.collider2, handle)
    if matches {
      result.push(pair)
    }
  }
  result
}

///|
pub fn NarrowPhase::contact_pairs_with(
  self : NarrowPhase,
  handle : ColliderHandle,
) -> Array[(ColliderHandle, ColliderHandle, ContactPair)] {
  let result : Array[(ColliderHandle, ColliderHandle, ContactPair)] = []
  for i in 0..<self.contacts.length() {
    let pair = self.contacts[i]
    if ColliderHandle::equals(pair.0, handle) ||
      ColliderHandle::equals(pair.1, handle) {
      result.push(pair)
    }
  }
  result
}

///|
pub fn NarrowPhase::contact_pair(
  self : NarrowPhase,
  first : ColliderHandle,
  second : ColliderHandle,
) -> ContactPair? {
  find_contact_pair(self.contacts, first, second)
}

///|
pub fn NarrowPhase::intersection_pair(
  self : NarrowPhase,
  first : ColliderHandle,
  second : ColliderHandle,
) -> IntersectionPair? {
  find_intersection_pair(self.intersections, first, second)
}

///|
priv struct Aabb {
  min : @core.Vec2
  max : @core.Vec2
}

///|
fn aabb_intersects(left : Aabb, right : Aabb) -> Bool {
  !(left.max.x < right.min.x ||
  left.min.x > right.max.x ||
  left.max.y < right.min.y ||
  left.min.y > right.max.y)
}

///|
fn aabb_union(left : Aabb, right : Aabb) -> Aabb {
  Aabb::{
    min: @core.Vec2::new(
      min_value(left.min.x, right.min.x),
      min_value(left.min.y, right.min.y),
    ),
    max: @core.Vec2::new(
      max_value(left.max.x, right.max.x),
      max_value(left.max.y, right.max.y),
    ),
  }
}

///|
fn aabb_center(aabb : Aabb) -> @core.Vec2 {
  @core.Vec2::new(
    0.5F * (aabb.min.x + aabb.max.x),
    0.5F * (aabb.min.y + aabb.max.y),
  )
}

///|
priv struct BvhLeaf {
  handle : ColliderHandle
  aabb : Aabb
  center : @core.Vec2
}

///|
priv struct BvhNode {
  aabb : Aabb
  left : Int?
  right : Int?
  leaf : ColliderHandle?
}

///|
fn swap_leaves(leaves : Array[BvhLeaf], i : Int, j : Int) -> Unit {
  if i == j {
    return
  }
  let tmp = leaves[i]
  leaves[i] = leaves[j]
  leaves[j] = tmp
}

///|
fn leaf_key(leaf : BvhLeaf, axis_x : Bool) -> @core.Real {
  if axis_x {
    leaf.center.x
  } else {
    leaf.center.y
  }
}

///|
fn quicksort_leaves(
  leaves : Array[BvhLeaf],
  start : Int,
  end : Int,
  axis_x : Bool,
) -> Unit {
  if end - start <= 1 {
    return
  }
  let pivot = leaf_key(leaves[(start + end) / 2], axis_x)
  let mut i = start
  let mut j = end - 1
  while i <= j {
    while leaf_key(leaves[i], axis_x) < pivot {
      i = i + 1
    }
    while leaf_key(leaves[j], axis_x) > pivot {
      j = j - 1
    }
    if i <= j {
      swap_leaves(leaves, i, j)
      i = i + 1
      j = j - 1
    }
  }
  if start < j + 1 {
    quicksort_leaves(leaves, start, j + 1, axis_x)
  }
  if i < end {
    quicksort_leaves(leaves, i, end, axis_x)
  }
}

///|
fn build_bvh(
  nodes : Array[BvhNode],
  leaves : Array[BvhLeaf],
  start : Int,
  end : Int,
) -> Int {
  let count = end - start
  if count == 1 {
    let leaf = leaves[start]
    let index = nodes.length()
    nodes.push(BvhNode::{
      aabb: leaf.aabb,
      left: None,
      right: None,
      leaf: Some(leaf.handle),
    })
    return index
  }

  // Choose split axis based on the spread of centers.
  let mut min_x = leaves[start].center.x
  let mut max_x = min_x
  let mut min_y = leaves[start].center.y
  let mut max_y = min_y
  for i in (start + 1)..<end {
    let c = leaves[i].center
    min_x = min_value(min_x, c.x)
    max_x = max_value(max_x, c.x)
    min_y = min_value(min_y, c.y)
    max_y = max_value(max_y, c.y)
  }
  let axis_x = max_x - min_x >= max_y - min_y
  quicksort_leaves(leaves, start, end, axis_x)
  let mid = start + count / 2
  let left = build_bvh(nodes, leaves, start, mid)
  let right = build_bvh(nodes, leaves, mid, end)
  let node_aabb = aabb_union(nodes[left].aabb, nodes[right].aabb)
  let index = nodes.length()
  nodes.push(BvhNode::{
    aabb: node_aabb,
    left: Some(left),
    right: Some(right),
    leaf: None,
  })
  index
}

///|
fn handle_less(left : ColliderHandle, right : ColliderHandle) -> Bool {
  if left.id < right.id {
    true
  } else if left.id > right.id {
    false
  } else {
    left.generation < right.generation
  }
}

///|
fn sorted_pair(
  a : ColliderHandle,
  b : ColliderHandle,
) -> (ColliderHandle, ColliderHandle) {
  if handle_less(a, b) {
    (a, b)
  } else {
    (b, a)
  }
}

///|
fn pair_less(
  left : (ColliderHandle, ColliderHandle),
  right : (ColliderHandle, ColliderHandle),
) -> Bool {
  if handle_less(left.0, right.0) {
    return true
  }
  if handle_less(right.0, left.0) {
    return false
  }
  handle_less(left.1, right.1)
}

///|
fn sort_pairs(pairs : Array[(ColliderHandle, ColliderHandle)]) -> Unit {
  for i in 1..<pairs.length() {
    let key = pairs[i]
    let mut j = i
    while j > 0 && pair_less(key, pairs[j - 1]) {
      pairs[j] = pairs[j - 1]
      j = j - 1
    }
    pairs[j] = key
  }
}

///|
fn collect_bvh_pairs(
  nodes : Array[BvhNode],
  a : Int,
  b : Int,
  out : Array[(ColliderHandle, ColliderHandle)],
) -> Unit {
  let node_a = nodes[a]
  let node_b = nodes[b]
  if !aabb_intersects(node_a.aabb, node_b.aabb) {
    return
  }
  if node_a.leaf is Some(handle_a) && node_b.leaf is Some(handle_b) {
    if ColliderHandle::equals(handle_a, handle_b) {
      return
    }
    out.push(sorted_pair(handle_a, handle_b))
    return
  }
  if a == b {
    if node_a.left is Some(left) && node_a.right is Some(right) {
      collect_bvh_pairs(nodes, left, left, out)
      collect_bvh_pairs(nodes, left, right, out)
      collect_bvh_pairs(nodes, right, right, out)
    }
    return
  }
  if node_a.leaf is Some(_) {
    if node_b.left is Some(left) && node_b.right is Some(right) {
      collect_bvh_pairs(nodes, a, left, out)
      collect_bvh_pairs(nodes, a, right, out)
    }
    return
  }
  if node_b.leaf is Some(_) {
    if node_a.left is Some(left) && node_a.right is Some(right) {
      collect_bvh_pairs(nodes, left, b, out)
      collect_bvh_pairs(nodes, right, b, out)
    }
    return
  }
  if node_a.left is Some(al) &&
    node_a.right is Some(ar) &&
    node_b.left is Some(bl) &&
    node_b.right is Some(br) {
    collect_bvh_pairs(nodes, al, bl, out)
    collect_bvh_pairs(nodes, al, br, out)
    collect_bvh_pairs(nodes, ar, bl, out)
    collect_bvh_pairs(nodes, ar, br, out)
  }
}

///|
fn compute_shape_aabb(
  shape : Shape,
  center : @core.Vec2,
  rotation : @core.Real,
  prediction_distance : @core.Real,
) -> Aabb {
  let pd = if prediction_distance < 0.0F { 0.0F } else { prediction_distance }
  match shape {
    Shape::Ball(radius) => {
      let r = radius + pd
      Aabb::{
        min: @core.Vec2::new(center.x - r, center.y - r),
        max: @core.Vec2::new(center.x + r, center.y + r),
      }
    }
    Shape::Cuboid(half_width, half_height) => {
      // Conservative AABB for an oriented rectangle.
      let rot = @core.Rot2::from_angle(rotation)
      let abs_cos = @core.abs(rot.cos)
      let abs_sin = @core.abs(rot.sin)
      let hw = abs_cos * half_width + abs_sin * half_height + pd
      let hh = abs_sin * half_width + abs_cos * half_height + pd
      Aabb::{
        min: @core.Vec2::new(center.x - hw, center.y - hh),
        max: @core.Vec2::new(center.x + hw, center.y + hh),
      }
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
      Aabb::{
        min: @core.Vec2::new(min_x - r, min_y - r),
        max: @core.Vec2::new(max_x + r, max_y + r),
      }
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
      Aabb::{
        min: @core.Vec2::new(min_x - r, min_y - r),
        max: @core.Vec2::new(max_x + r, max_y + r),
      }
    }
    Shape::Segment(a, b) => {
      let rot = @core.Rot2::from_angle(rotation)
      let wa = center.add(rot.rotate_vec2(a))
      let wb = center.add(rot.rotate_vec2(b))
      let min_x = if wa.x < wb.x { wa.x } else { wb.x }
      let max_x = if wa.x > wb.x { wa.x } else { wb.x }
      let min_y = if wa.y < wb.y { wa.y } else { wb.y }
      let max_y = if wa.y > wb.y { wa.y } else { wb.y }
      Aabb::{
        min: @core.Vec2::new(min_x - pd, min_y - pd),
        max: @core.Vec2::new(max_x + pd, max_y + pd),
      }
    }
  }
}

///|
pub struct BroadPhaseBvh {
  pairs : Array[(ColliderHandle, ColliderHandle)]
}

///|
pub fn BroadPhaseBvh::new() -> BroadPhaseBvh {
  { pairs: [] }
}

///|
pub fn BroadPhaseBvh::as_query_pipeline(
  self : BroadPhaseBvh,
  bodies : @dynamics.RigidBodySet,
  colliders : ColliderSet,
  filter : QueryFilter,
) -> QueryPipeline {
  self |> ignore
  QueryPipeline::new(filter, bodies, colliders)
}

///|
pub fn BroadPhaseBvh::update(
  self : BroadPhaseBvh,
  prediction_distance : @core.Real,
  bodies : @dynamics.RigidBodySet,
  colliders : ColliderSet,
) -> Unit {
  colliders.sync_with_bodies(bodies)
  self.pairs.clear()
  let leaves : Array[BvhLeaf] = []
  for i in 0..<colliders.colliders.length() {
    if colliders.colliders[i] is Some(collider) && collider.is_enabled() {
      let handle = ColliderHandle::new(i, colliders.generations[i])
      let aabb = compute_shape_aabb(
        collider.shape,
        collider.world_translation,
        collider.world_rotation,
        prediction_distance,
      )
      leaves.push(BvhLeaf::{ handle, aabb, center: aabb_center(aabb) })
    }
  }
  if leaves.length() <= 1 {
    return
  }
  let nodes : Array[BvhNode] = []
  let root = build_bvh(nodes, leaves, 0, leaves.length())
  collect_bvh_pairs(nodes, root, root, self.pairs)
  sort_pairs(self.pairs)
}

///|
pub fn BroadPhaseBvh::pairs(
  self : BroadPhaseBvh,
) -> Array[(ColliderHandle, ColliderHandle)] {
  self.pairs
}
