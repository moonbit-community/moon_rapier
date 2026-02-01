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
/// Minimal real 3D kinematic character controller.
///
/// This is a pragmatic controller intended to migrate the 3D character controller example to the
/// 3DReal backend before full 3D shape casting support exists.
pub struct EffectiveMovement3DReal {
  translation : @core.Vec3
  grounded : Bool
  is_sliding_down_slope : Bool
}

///|
pub fn EffectiveMovement3DReal::translation(
  self : EffectiveMovement3DReal,
) -> @core.Vec3 {
  self.translation
}

///|
pub fn EffectiveMovement3DReal::is_grounded(
  self : EffectiveMovement3DReal,
) -> Bool {
  self.grounded
}

///|
pub fn EffectiveMovement3DReal::is_sliding_down_slope(
  self : EffectiveMovement3DReal,
) -> Bool {
  self.is_sliding_down_slope
}

///|
pub struct KinematicCharacterController3DReal {
  up : @core.Vec3
  // Desired distance between the character origin and the ground along `up`.
  offset : @core.Real
  // Angles measured from the up axis.
  max_slope_climb_angle : @core.Real
  min_slope_slide_angle : @core.Real
  slide : Bool
}

///|
pub fn KinematicCharacterController3DReal::new() -> KinematicCharacterController3DReal {
  {
    up: @core.Vec3::new(0.0F, 1.0F, 0.0F),
    offset: 0.01F,
    slide: true,
    max_slope_climb_angle: 0.785398163F,
    min_slope_slide_angle: 0.785398163F,
  }
}

///|
pub fn KinematicCharacterController3DReal::default() -> KinematicCharacterController3DReal {
  KinematicCharacterController3DReal::new()
}

///|
pub fn KinematicCharacterController3DReal::with_offset(
  self : KinematicCharacterController3DReal,
  offset : @core.Real,
) -> KinematicCharacterController3DReal {
  {
    up: self.up,
    offset,
    slide: self.slide,
    max_slope_climb_angle: self.max_slope_climb_angle,
    min_slope_slide_angle: self.min_slope_slide_angle,
  }
}

///|
pub fn KinematicCharacterController3DReal::with_max_slope_climb_angle(
  self : KinematicCharacterController3DReal,
  angle : @core.Real,
) -> KinematicCharacterController3DReal {
  {
    up: self.up,
    offset: self.offset,
    slide: self.slide,
    max_slope_climb_angle: angle,
    min_slope_slide_angle: self.min_slope_slide_angle,
  }
}

///|
pub fn KinematicCharacterController3DReal::with_min_slope_slide_angle(
  self : KinematicCharacterController3DReal,
  angle : @core.Real,
) -> KinematicCharacterController3DReal {
  {
    up: self.up,
    offset: self.offset,
    slide: self.slide,
    max_slope_climb_angle: self.max_slope_climb_angle,
    min_slope_slide_angle: angle,
  }
}

///|
fn cc3d_real_normalized_or_default(v : @core.Vec3) -> @core.Vec3 {
  let n2 = v.length_squared()
  if n2 <= 1.0e-12F {
    @core.Vec3::new(0.0F, 1.0F, 0.0F)
  } else {
    v.scale(1.0F / Float::sqrt(n2))
  }
}

///|
fn clamp01(x : @core.Real) -> @core.Real {
  if x < 0.0F {
    0.0F
  } else if x > 1.0F {
    1.0F
  } else {
    x
  }
}

///|
pub fn KinematicCharacterController3DReal::move_character(
  self : KinematicCharacterController3DReal,
  bodies : @dynamics.RigidBodySet3D,
  colliders : @collision.ColliderSet3D,
  queries : @collision.QueryPipelineMut3DReal,
  character : @dynamics.RigidBodyHandle,
  desired_translation : @core.Vec3,
) -> EffectiveMovement3DReal {
  colliders.sync_with_bodies(bodies)
  queries.update()
  let qp = queries
    .as_ref()
    .with_filter(
      @collision.QueryFilter3DReal::new().exclude_rigid_body(character),
    )
  let up = cc3d_real_normalized_or_default(self.up)
  let down = up.scale(-1.0F)
  if bodies.get_mut(character) is Some(rb) {
    let start = rb.translation()
    let target_xz = start.add(
      @core.Vec3::new(desired_translation.x, 0.0F, desired_translation.z),
    )
    let probe_origin = target_xz.add(up.scale(2.0F))
    let ray = @collision.Ray3::new(probe_origin, down)
    let max_toi = 10.0F
    let mut grounded = false
    let mut sliding = false
    let mut out_pos = target_xz
    if qp.cast_ray_and_get_normal(ray, max_toi, true) is Some((_h, hit)) {
      let n = cc3d_real_normalized_or_default(hit.normal())
      let cos_angle = clamp01(n.dot(up))
      let angle = Float::from_double(@math.acos(cos_angle.to_double()))
      let ground_y = probe_origin.add(down.scale(hit.toi())).dot(up)
      let base_y = ground_y + self.offset

      // If the surface is too steep to climb, cancel horizontal motion.
      if angle > self.max_slope_climb_angle {
        out_pos = start
        sliding = self.slide && angle > self.min_slope_slide_angle
      } else {
        grounded = true
        // Project the target position onto the ground along `up`.
        out_pos = @core.Vec3::new(out_pos.x, 0.0F, out_pos.z).add(
          up.scale(base_y),
        )
      }
    }
    rb.set_next_kinematic_translation(out_pos)
    EffectiveMovement3DReal::{
      translation: out_pos.sub(start),
      grounded,
      is_sliding_down_slope: sliding,
    }
  } else {
    EffectiveMovement3DReal::{
      translation: @core.Vec3::zero(),
      grounded: false,
      is_sliding_down_slope: false,
    }
  }
}
