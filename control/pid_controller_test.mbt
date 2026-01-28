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
test "pd_controller masks axes" {
  let controller = PdController::new(
    1.0F,
    1.0F,
    @dynamics.AxesMask::lin_x().insert(@dynamics.AxesMask::ang_z()),
  )
  let pose_errors = PdErrors::{
    linear: @core.Vec2::new(1.0F, 2.0F),
    angular: 3.0F,
  }
  let vel_errors = PdErrors::{
    linear: @core.Vec2::new(4.0F, 5.0F),
    angular: 6.0F,
  }
  let correction = controller.correction(pose_errors, vel_errors)
  let linvel = correction.linvel()
  inspect(linvel.x == 5.0F, content="true")
  inspect(linvel.y == 0.0F, content="true")
  inspect(correction.angvel() == 9.0F, content="true")
}

///|
test "pid_controller integrates errors" {
  let controller = PidController::new(
    0.0F,
    1.0F,
    0.0F,
    @dynamics.AxesMask::all(),
  )
  let pose_errors = PdErrors::{
    linear: @core.Vec2::new(1.0F, 0.0F),
    angular: 2.0F,
  }
  let vel_errors = PdErrors::{ linear: @core.Vec2::zero(), angular: 0.0F }
  let correction = controller.correction(0.5F, pose_errors, vel_errors)
  let linvel = correction.linvel()
  inspect(linvel.x == 0.5F, content="true")
  inspect(linvel.y == 0.0F, content="true")
  inspect(correction.angvel() == 1.0F, content="true")
}

///|
test "pd_controller rigid_body_correction zero at target" {
  let rb = @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(1.0F, 2.0F))
    .rotation(@core.Rot2::from_angle(0.3F))
    .linvel(@core.Vec2::new(0.4F, -0.2F))
    .angvel(0.5F)
    .build()
  let controller = PdController::new(10.0F, 1.0F, @dynamics.AxesMask::all())
  let corr = controller.rigid_body_correction(rb, rb.position(), rb.vels())
  let linvel = corr.linvel()
  inspect(linvel.x == 0.0F, content="true")
  inspect(linvel.y == 0.0F, content="true")
  inspect(corr.angvel() == 0.0F, content="true")
}
