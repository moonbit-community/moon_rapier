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
pub(all) enum CharacterLength {
  Relative(@core.Real)
  Absolute(@core.Real)
}

///|
#declaration_only
pub fn CharacterLength::relative(value : @core.Real) -> CharacterLength {
  ...
}

///|
#declaration_only
pub fn CharacterLength::absolute(value : @core.Real) -> CharacterLength {
  ...
}

///|
#declaration_only
pub fn CharacterLength::map_absolute(
  self : CharacterLength,
  f : (@core.Real) -> @core.Real,
) -> CharacterLength {
  ...
}

///|
#declaration_only
pub fn CharacterLength::map_relative(
  self : CharacterLength,
  f : (@core.Real) -> @core.Real,
) -> CharacterLength {
  ...
}

///|
#declaration_only
pub type CharacterCollision

///|
#declaration_only
pub fn CharacterCollision::handle(
  self : CharacterCollision,
) -> @collision.ColliderHandle {
  ...
}

///|
#declaration_only
pub fn CharacterCollision::hit(
  self : CharacterCollision,
) -> @collision.ShapeCastHit {
  ...
}

///|
#declaration_only
pub fn CharacterCollision::translation_applied(
  self : CharacterCollision,
) -> @core.Vec2 {
  ...
}

///|
#declaration_only
pub fn CharacterCollision::translation_remaining(
  self : CharacterCollision,
) -> @core.Vec2 {
  ...
}

///|
#declaration_only
pub fn CharacterCollision::character_pos(
  self : CharacterCollision,
) -> @core.Isometry2 {
  ...
}

///|
#declaration_only
pub type EffectiveMovement

///|
#declaration_only
pub fn EffectiveMovement::translation(self : EffectiveMovement) -> @core.Vec2 {
  ...
}

///|
#declaration_only
pub fn EffectiveMovement::is_grounded(self : EffectiveMovement) -> Bool {
  ...
}

///|
#declaration_only
pub fn EffectiveMovement::is_sliding_down_slope(
  self : EffectiveMovement,
) -> Bool {
  ...
}

///|
#declaration_only
pub type CharacterAutostep

///|
#declaration_only
pub fn CharacterAutostep::new(
  max_height : CharacterLength,
  min_width : CharacterLength,
) -> CharacterAutostep {
  ...
}

///|
#declaration_only
pub fn CharacterAutostep::default() -> CharacterAutostep {
  ...
}

///|
#declaration_only
pub fn CharacterAutostep::max_height(
  self : CharacterAutostep,
) -> CharacterLength {
  ...
}

///|
#declaration_only
pub fn CharacterAutostep::min_width(
  self : CharacterAutostep,
) -> CharacterLength {
  ...
}

///|
#declaration_only
pub fn CharacterAutostep::include_dynamic_bodies(
  self : CharacterAutostep,
) -> Bool {
  ...
}

///|
#declaration_only
pub fn CharacterAutostep::with_include_dynamic_bodies(
  self : CharacterAutostep,
  include_dynamic_bodies : Bool,
) -> CharacterAutostep {
  ...
}

///|
#declaration_only
pub type KinematicCharacterController

///|
#declaration_only
pub fn KinematicCharacterController::new() -> KinematicCharacterController {
  ...
}

///|
#declaration_only
pub fn KinematicCharacterController::default() -> KinematicCharacterController {
  ...
}

///|
#declaration_only
pub fn KinematicCharacterController::with_up(
  self : KinematicCharacterController,
  up : @core.Vec2,
) -> KinematicCharacterController {
  ...
}

///|
#declaration_only
pub fn KinematicCharacterController::with_offset(
  self : KinematicCharacterController,
  offset : CharacterLength,
) -> KinematicCharacterController {
  ...
}

///|
#declaration_only
pub fn KinematicCharacterController::with_slide(
  self : KinematicCharacterController,
  slide : Bool,
) -> KinematicCharacterController {
  ...
}

///|
#declaration_only
pub fn KinematicCharacterController::with_max_slope_climb_angle(
  self : KinematicCharacterController,
  angle : @core.Real,
) -> KinematicCharacterController {
  ...
}

///|
#declaration_only
pub fn KinematicCharacterController::with_min_slope_slide_angle(
  self : KinematicCharacterController,
  angle : @core.Real,
) -> KinematicCharacterController {
  ...
}

///|
#declaration_only
pub fn KinematicCharacterController::with_snap_to_ground(
  self : KinematicCharacterController,
  snap : CharacterLength?,
) -> KinematicCharacterController {
  ...
}

///|
#declaration_only
pub fn KinematicCharacterController::with_autostep(
  self : KinematicCharacterController,
  autostep : CharacterAutostep?,
) -> KinematicCharacterController {
  ...
}

///|
#declaration_only
pub fn KinematicCharacterController::with_normal_nudge_factor(
  self : KinematicCharacterController,
  normal_nudge_factor : @core.Real,
) -> KinematicCharacterController {
  ...
}

///|
#declaration_only
pub fn KinematicCharacterController::move_shape(
  self : KinematicCharacterController,
  dt : @core.Real,
  query : @collision.QueryPipeline,
  bodies : @dynamics.RigidBodySet,
  colliders : @collision.ColliderSet,
  shape : @collision.Shape,
  position : @core.Isometry2,
  translation : @core.Vec2,
  handle_collision : (CharacterCollision) -> Unit,
) -> EffectiveMovement {
  ...
}

///|
#declaration_only
pub fn KinematicCharacterController::solve_character_collision_impulses(
  self : KinematicCharacterController,
  dt : @core.Real,
  query : @collision.QueryPipeline,
  bodies : @dynamics.RigidBodySet,
  colliders : @collision.ColliderSet,
  character_shape : @collision.Shape,
  character_mass : @core.Real,
  collisions : Array[CharacterCollision],
) -> Unit {
  ...
}

///|
#declaration_only
pub type PdErrors

///|
#declaration_only
pub fn PdErrors::from_vels(vels : @dynamics.RigidBodyVelocity) -> PdErrors {
  ...
}

///|
#declaration_only
pub type PdController

///|
#declaration_only
pub fn PdController::default() -> PdController {
  ...
}

///|
#declaration_only
pub fn PdController::new(
  kp : @core.Real,
  kd : @core.Real,
  axes : @dynamics.AxesMask,
) -> PdController {
  ...
}

///|
#declaration_only
pub fn PdController::linear_rigid_body_correction(
  self : PdController,
  rb : @dynamics.RigidBody,
  target_pos : @core.Vec2,
  target_linvel : @core.Vec2,
) -> @core.Vec2 {
  ...
}

///|
#declaration_only
pub fn PdController::angular_rigid_body_correction(
  self : PdController,
  rb : @dynamics.RigidBody,
  target_rot : @core.Rot2,
  target_angvel : @core.Real,
) -> @core.Real {
  ...
}

///|
#declaration_only
pub fn PdController::rigid_body_correction(
  self : PdController,
  rb : @dynamics.RigidBody,
  target_pose : @core.Isometry2,
  target_vels : @dynamics.RigidBodyVelocity,
) -> @dynamics.RigidBodyVelocity {
  ...
}

///|
#declaration_only
pub fn PdController::correction(
  self : PdController,
  pose_errors : PdErrors,
  vel_errors : PdErrors,
) -> @dynamics.RigidBodyVelocity {
  ...
}

///|
#declaration_only
pub type PidController

///|
#declaration_only
pub fn PidController::default() -> PidController {
  ...
}

///|
#declaration_only
pub fn PidController::new(
  kp : @core.Real,
  ki : @core.Real,
  kd : @core.Real,
  axes : @dynamics.AxesMask,
) -> PidController {
  ...
}

///|
#declaration_only
pub fn PidController::set_axes(
  self : PidController,
  axes : @dynamics.AxesMask,
) -> PidController {
  ...
}

///|
#declaration_only
pub fn PidController::axes(self : PidController) -> @dynamics.AxesMask {
  ...
}

///|
#declaration_only
pub fn PidController::reset_integrals(self : PidController) -> PidController {
  ...
}

///|
#declaration_only
pub fn PidController::linear_rigid_body_correction(
  self : PidController,
  dt : @core.Real,
  rb : @dynamics.RigidBody,
  target_pos : @core.Vec2,
  target_linvel : @core.Vec2,
) -> @core.Vec2 {
  ...
}

///|
#declaration_only
pub fn PidController::angular_rigid_body_correction(
  self : PidController,
  dt : @core.Real,
  rb : @dynamics.RigidBody,
  target_rot : @core.Rot2,
  target_angvel : @core.Real,
) -> @core.Real {
  ...
}

///|
#declaration_only
pub fn PidController::rigid_body_correction(
  self : PidController,
  dt : @core.Real,
  rb : @dynamics.RigidBody,
  target_pose : @core.Isometry2,
  target_vels : @dynamics.RigidBodyVelocity,
) -> @dynamics.RigidBodyVelocity {
  ...
}

///|
#declaration_only
pub fn PidController::correction(
  self : PidController,
  dt : @core.Real,
  pose_errors : PdErrors,
  vel_errors : PdErrors,
) -> @dynamics.RigidBodyVelocity {
  ...
}

///|
#declaration_only
pub type WheelTuning

///|
#declaration_only
pub fn WheelTuning::default() -> WheelTuning {
  ...
}

///|
#declaration_only
pub fn WheelTuning::with_suspension_stiffness(
  self : WheelTuning,
  suspension_stiffness : @core.Real,
) -> WheelTuning {
  ...
}

///|
#declaration_only
pub fn WheelTuning::with_suspension_compression(
  self : WheelTuning,
  suspension_compression : @core.Real,
) -> WheelTuning {
  ...
}

///|
#declaration_only
pub fn WheelTuning::with_suspension_damping(
  self : WheelTuning,
  suspension_damping : @core.Real,
) -> WheelTuning {
  ...
}

///|
#declaration_only
pub fn WheelTuning::with_max_suspension_travel(
  self : WheelTuning,
  max_suspension_travel : @core.Real,
) -> WheelTuning {
  ...
}

///|
#declaration_only
pub fn WheelTuning::with_side_friction_stiffness(
  self : WheelTuning,
  side_friction_stiffness : @core.Real,
) -> WheelTuning {
  ...
}

///|
#declaration_only
pub fn WheelTuning::with_friction_slip(
  self : WheelTuning,
  friction_slip : @core.Real,
) -> WheelTuning {
  ...
}

///|
#declaration_only
pub fn WheelTuning::with_max_suspension_force(
  self : WheelTuning,
  max_suspension_force : @core.Real,
) -> WheelTuning {
  ...
}

///|
#declaration_only
pub type RayCastInfo

///|
#declaration_only
pub fn RayCastInfo::default() -> RayCastInfo {
  ...
}

///|
#declaration_only
pub type Wheel

///|
#declaration_only
pub fn Wheel::raycast_info(self : Wheel) -> RayCastInfo {
  ...
}

///|
#declaration_only
pub fn Wheel::center(self : Wheel) -> @core.Vec2 {
  ...
}

///|
#declaration_only
pub fn Wheel::suspension(self : Wheel) -> @core.Vec2 {
  ...
}

///|
#declaration_only
pub type DynamicRayCastVehicleController

///|
#declaration_only
pub fn DynamicRayCastVehicleController::new(
  chassis : @dynamics.RigidBodyHandle,
) -> DynamicRayCastVehicleController {
  ...
}

///|
#declaration_only
pub fn DynamicRayCastVehicleController::add_wheel(
  self : DynamicRayCastVehicleController,
  chassis_connection_cs : @core.Vec2,
  direction_cs : @core.Vec2,
  suspension_rest_length : @core.Real,
  radius : @core.Real,
  tuning : WheelTuning,
) -> Unit {
  ...
}

///|
#declaration_only
pub fn DynamicRayCastVehicleController::set_engine_force(
  self : DynamicRayCastVehicleController,
  wheel_id : Int,
  force : @core.Real,
) -> Unit {
  ...
}

///|
#declaration_only
pub fn DynamicRayCastVehicleController::set_brake(
  self : DynamicRayCastVehicleController,
  wheel_id : Int,
  brake : @core.Real,
) -> Unit {
  ...
}

///|
#declaration_only
pub fn DynamicRayCastVehicleController::update_vehicle(
  self : DynamicRayCastVehicleController,
  dt : @core.Real,
  queries : @collision.QueryPipelineMut,
  bodies : @dynamics.RigidBodySet,
  colliders : @collision.ColliderSet,
) -> Unit {
  ...
}

///|
/// Dim3-flavored raycast vehicle controller.
#declaration_only
pub type RayCastInfo3

///|
#declaration_only
pub fn RayCastInfo3::default() -> RayCastInfo3 {
  ...
}

///|
#declaration_only
pub type Wheel3

///|
#declaration_only
pub fn Wheel3::raycast_info(self : Wheel3) -> RayCastInfo3 {
  ...
}

///|
#declaration_only
pub fn Wheel3::center(self : Wheel3) -> @core.Vec3 {
  ...
}

///|
#declaration_only
pub fn Wheel3::suspension(self : Wheel3) -> @core.Vec3 {
  ...
}

///|
#declaration_only
pub fn Wheel3::axle(self : Wheel3) -> @core.Vec3 {
  ...
}

///|
#declaration_only
pub type DynamicRayCastVehicleController3

///|
#declaration_only
pub fn DynamicRayCastVehicleController3::new(
  chassis : @dynamics.RigidBodyHandle,
) -> DynamicRayCastVehicleController3 {
  ...
}

///|
#declaration_only
pub fn DynamicRayCastVehicleController3::add_wheel(
  self : DynamicRayCastVehicleController3,
  chassis_connection_cs : @core.Vec3,
  direction_cs : @core.Vec3,
  axle_cs : @core.Vec3,
  suspension_rest_length : @core.Real,
  radius : @core.Real,
  tuning : WheelTuning,
) -> Int {
  ...
}

///|
#declaration_only
pub fn DynamicRayCastVehicleController3::set_engine_force(
  self : DynamicRayCastVehicleController3,
  wheel_id : Int,
  force : @core.Real,
) -> Unit {
  ...
}

///|
#declaration_only
pub fn DynamicRayCastVehicleController3::set_brake(
  self : DynamicRayCastVehicleController3,
  wheel_id : Int,
  brake : @core.Real,
) -> Unit {
  ...
}

///|
#declaration_only
pub fn DynamicRayCastVehicleController3::set_steering_value(
  self : DynamicRayCastVehicleController3,
  wheel_id : Int,
  steering : @core.Real,
) -> Unit {
  ...
}

///|
#declaration_only
pub fn DynamicRayCastVehicleController3::wheels(
  self : DynamicRayCastVehicleController3,
) -> Array[Wheel3] {
  ...
}

///|
#declaration_only
pub fn DynamicRayCastVehicleController3::update_vehicle(
  self : DynamicRayCastVehicleController3,
  dt : @core.Real,
  queries : @collision.QueryPipelineMut,
  bodies : @dynamics.RigidBodySet,
  colliders : @collision.ColliderSet,
) -> Unit {
  ...
}

///|
#declaration_only
pub type Wheel3DReal

///|
#declaration_only
pub type DynamicRayCastVehicleController3DReal

///|
#declaration_only
pub fn DynamicRayCastVehicleController3DReal::new(
  chassis : @dynamics.RigidBodyHandle,
) -> DynamicRayCastVehicleController3DReal {
  ...
}

///|
#declaration_only
pub fn DynamicRayCastVehicleController3DReal::add_wheel(
  self : DynamicRayCastVehicleController3DReal,
  connection_point_cs : @core.Vec3,
  direction_cs : @core.Vec3,
  axle_cs : @core.Vec3,
  suspension_rest_length : @core.Real,
  radius : @core.Real,
  tuning : WheelTuning,
) -> Int {
  ...
}

///|
#declaration_only
pub fn DynamicRayCastVehicleController3DReal::set_engine_force(
  self : DynamicRayCastVehicleController3DReal,
  wheel : Int,
  value : @core.Real,
) -> Unit {
  ...
}

///|
#declaration_only
pub fn DynamicRayCastVehicleController3DReal::set_brake(
  self : DynamicRayCastVehicleController3DReal,
  wheel : Int,
  value : @core.Real,
) -> Unit {
  ...
}

///|
#declaration_only
pub fn DynamicRayCastVehicleController3DReal::set_steering_value(
  self : DynamicRayCastVehicleController3DReal,
  wheel : Int,
  value : @core.Real,
) -> Unit {
  ...
}

///|
#declaration_only
pub fn DynamicRayCastVehicleController3DReal::wheels(
  self : DynamicRayCastVehicleController3DReal,
) -> Array[Wheel3DReal] {
  ...
}

///|
#declaration_only
pub fn DynamicRayCastVehicleController3DReal::update_vehicle(
  self : DynamicRayCastVehicleController3DReal,
  dt : @core.Real,
  queries : @collision.QueryPipelineMut3DReal,
  bodies : @dynamics.RigidBodySet3D,
  colliders : @collision.ColliderSet3D,
) -> Unit {
  ...
}

///|
#declaration_only
pub type EffectiveMovement3DReal

///|
#declaration_only
pub fn EffectiveMovement3DReal::translation(
  self : EffectiveMovement3DReal,
) -> @core.Vec3 {
  ...
}

///|
#declaration_only
pub fn EffectiveMovement3DReal::is_grounded(
  self : EffectiveMovement3DReal,
) -> Bool {
  ...
}

///|
#declaration_only
pub fn EffectiveMovement3DReal::is_sliding_down_slope(
  self : EffectiveMovement3DReal,
) -> Bool {
  ...
}

///|
#declaration_only
pub type CharacterCollision3DReal

///|
#declaration_only
pub fn CharacterCollision3DReal::handle(
  self : CharacterCollision3DReal,
) -> @collision.ColliderHandle3D {
  ...
}

///|
#declaration_only
pub fn CharacterCollision3DReal::hit(
  self : CharacterCollision3DReal,
) -> @collision.ShapeCastHit3 {
  ...
}

///|
#declaration_only
pub fn CharacterCollision3DReal::translation_applied(
  self : CharacterCollision3DReal,
) -> @core.Vec3 {
  ...
}

///|
#declaration_only
pub fn CharacterCollision3DReal::translation_remaining(
  self : CharacterCollision3DReal,
) -> @core.Vec3 {
  ...
}

///|
#declaration_only
pub fn CharacterCollision3DReal::character_pos(
  self : CharacterCollision3DReal,
) -> @core.Isometry3 {
  ...
}

///|
#declaration_only
pub type KinematicCharacterController3DReal

///|
#declaration_only
pub fn KinematicCharacterController3DReal::new() -> KinematicCharacterController3DReal {
  ...
}

///|
#declaration_only
pub fn KinematicCharacterController3DReal::default() -> KinematicCharacterController3DReal {
  ...
}

///|
#declaration_only
pub fn KinematicCharacterController3DReal::with_up(
  self : KinematicCharacterController3DReal,
  up : @core.Vec3,
) -> KinematicCharacterController3DReal {
  ...
}

///|
#declaration_only
pub fn KinematicCharacterController3DReal::with_offset(
  self : KinematicCharacterController3DReal,
  offset : CharacterLength,
) -> KinematicCharacterController3DReal {
  ...
}

///|
#declaration_only
pub fn KinematicCharacterController3DReal::with_slide(
  self : KinematicCharacterController3DReal,
  slide : Bool,
) -> KinematicCharacterController3DReal {
  ...
}

///|
#declaration_only
pub fn KinematicCharacterController3DReal::with_autostep(
  self : KinematicCharacterController3DReal,
  autostep : CharacterAutostep?,
) -> KinematicCharacterController3DReal {
  ...
}

///|
#declaration_only
pub fn KinematicCharacterController3DReal::with_max_slope_climb_angle(
  self : KinematicCharacterController3DReal,
  angle : @core.Real,
) -> KinematicCharacterController3DReal {
  ...
}

///|
#declaration_only
pub fn KinematicCharacterController3DReal::with_min_slope_slide_angle(
  self : KinematicCharacterController3DReal,
  angle : @core.Real,
) -> KinematicCharacterController3DReal {
  ...
}

///|
#declaration_only
pub fn KinematicCharacterController3DReal::with_snap_to_ground(
  self : KinematicCharacterController3DReal,
  snap : CharacterLength?,
) -> KinematicCharacterController3DReal {
  ...
}

///|
#declaration_only
pub fn KinematicCharacterController3DReal::with_normal_nudge_factor(
  self : KinematicCharacterController3DReal,
  nudge : @core.Real,
) -> KinematicCharacterController3DReal {
  ...
}

///|
#declaration_only
pub fn KinematicCharacterController3DReal::move_shape(
  self : KinematicCharacterController3DReal,
  dt : @core.Real,
  query : @collision.QueryPipeline3DReal,
  bodies : @dynamics.RigidBodySet3D,
  colliders : @collision.ColliderSet3D,
  shape : @collision.Shape3D,
  position : @core.Isometry3,
  translation : @core.Vec3,
  handle_collision : (CharacterCollision3DReal) -> Unit,
) -> EffectiveMovement3DReal {
  ...
}

///|
#declaration_only
pub fn KinematicCharacterController3DReal::solve_character_collision_impulses(
  self : KinematicCharacterController3DReal,
  dt : @core.Real,
  query : @collision.QueryPipeline3DReal,
  bodies : @dynamics.RigidBodySet3D,
  colliders : @collision.ColliderSet3D,
  character_shape : @collision.Shape3D,
  character_mass : @core.Real,
  collisions : Array[CharacterCollision3DReal],
) -> Unit {
  ...
}
