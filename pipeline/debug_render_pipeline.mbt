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
pub struct DebugColor {
  h : @core.Real
  s : @core.Real
  l : @core.Real
  a : @core.Real
}

///|
pub fn DebugColor::new(
  h : @core.Real,
  s : @core.Real,
  l : @core.Real,
  a : @core.Real,
) -> DebugColor {
  { h, s, l, a }
}

///|
pub struct DebugRenderStyle {
  subdivisions : Int
  border_subdivisions : Int
  collider_dynamic_color : DebugColor
  collider_fixed_color : DebugColor
  collider_kinematic_color : DebugColor
  collider_parentless_color : DebugColor
  impulse_joint_anchor_color : DebugColor
  impulse_joint_separation_color : DebugColor
  multibody_joint_anchor_color : DebugColor
  multibody_joint_separation_color : DebugColor
  sleep_color_multiplier : DebugColor
  disabled_color_multiplier : DebugColor
  rigid_body_axes_length : @core.Real
  contact_depth_color : DebugColor
  contact_normal_color : DebugColor
  contact_normal_length : @core.Real
  collider_aabb_color : DebugColor
}

///|
pub fn DebugRenderStyle::default() -> DebugRenderStyle {
  {
    subdivisions: 20,
    border_subdivisions: 5,
    collider_dynamic_color: DebugColor::new(340.0F, 1.0F, 0.3F, 1.0F),
    collider_kinematic_color: DebugColor::new(20.0F, 1.0F, 0.3F, 1.0F),
    collider_fixed_color: DebugColor::new(30.0F, 1.0F, 0.4F, 1.0F),
    collider_parentless_color: DebugColor::new(30.0F, 1.0F, 0.4F, 1.0F),
    impulse_joint_anchor_color: DebugColor::new(240.0F, 0.5F, 0.4F, 1.0F),
    impulse_joint_separation_color: DebugColor::new(0.0F, 0.5F, 0.4F, 1.0F),
    multibody_joint_anchor_color: DebugColor::new(300.0F, 1.0F, 0.4F, 1.0F),
    multibody_joint_separation_color: DebugColor::new(0.0F, 1.0F, 0.4F, 1.0F),
    sleep_color_multiplier: DebugColor::new(1.0F, 1.0F, 0.2F, 1.0F),
    disabled_color_multiplier: DebugColor::new(0.0F, 0.0F, 1.0F, 1.0F),
    rigid_body_axes_length: 0.5F,
    contact_depth_color: DebugColor::new(120.0F, 1.0F, 0.4F, 1.0F),
    contact_normal_color: DebugColor::new(0.0F, 1.0F, 1.0F, 1.0F),
    contact_normal_length: 0.3F,
    collider_aabb_color: DebugColor::new(124.0F, 1.0F, 0.4F, 1.0F),
  }
}

///|
pub struct DebugRenderMode {
  mut bits : Int
}

///|
const DEBUG_RENDER_MODE_COLLIDER_SHAPES : Int = 1 << 0

///|
const DEBUG_RENDER_MODE_RIGID_BODY_AXES : Int = 1 << 1

///|
const DEBUG_RENDER_MODE_MULTIBODY_JOINTS : Int = 1 << 2

///|
const DEBUG_RENDER_MODE_IMPULSE_JOINTS : Int = 1 << 3

///|
const DEBUG_RENDER_MODE_SOLVER_CONTACTS : Int = 1 << 4

///|
const DEBUG_RENDER_MODE_CONTACTS : Int = 1 << 5

///|
const DEBUG_RENDER_MODE_COLLIDER_AABBS : Int = 1 << 6

///|
pub fn DebugRenderMode::empty() -> DebugRenderMode {
  { bits: 0 }
}

///|
pub fn DebugRenderMode::all() -> DebugRenderMode {
  {
    bits: DEBUG_RENDER_MODE_COLLIDER_SHAPES |
    DEBUG_RENDER_MODE_RIGID_BODY_AXES |
    DEBUG_RENDER_MODE_MULTIBODY_JOINTS |
    DEBUG_RENDER_MODE_IMPULSE_JOINTS |
    DEBUG_RENDER_MODE_SOLVER_CONTACTS |
    DEBUG_RENDER_MODE_CONTACTS |
    DEBUG_RENDER_MODE_COLLIDER_AABBS,
  }
}

///|
pub fn DebugRenderMode::collider_shapes() -> DebugRenderMode {
  { bits: DEBUG_RENDER_MODE_COLLIDER_SHAPES }
}

///|
pub fn DebugRenderMode::rigid_body_axes() -> DebugRenderMode {
  { bits: DEBUG_RENDER_MODE_RIGID_BODY_AXES }
}

///|
pub fn DebugRenderMode::multibody_joints() -> DebugRenderMode {
  { bits: DEBUG_RENDER_MODE_MULTIBODY_JOINTS }
}

///|
pub fn DebugRenderMode::impulse_joints() -> DebugRenderMode {
  { bits: DEBUG_RENDER_MODE_IMPULSE_JOINTS }
}

///|
pub fn DebugRenderMode::joints() -> DebugRenderMode {
  {
    bits: DEBUG_RENDER_MODE_MULTIBODY_JOINTS | DEBUG_RENDER_MODE_IMPULSE_JOINTS,
  }
}

///|
pub fn DebugRenderMode::solver_contacts() -> DebugRenderMode {
  { bits: DEBUG_RENDER_MODE_SOLVER_CONTACTS }
}

///|
pub fn DebugRenderMode::contacts() -> DebugRenderMode {
  { bits: DEBUG_RENDER_MODE_CONTACTS }
}

///|
pub fn DebugRenderMode::collider_aabbs() -> DebugRenderMode {
  { bits: DEBUG_RENDER_MODE_COLLIDER_AABBS }
}

///|
pub fn DebugRenderMode::default() -> DebugRenderMode {
  DebugRenderMode::collider_shapes()
  .insert(DebugRenderMode::joints())
  .insert(DebugRenderMode::rigid_body_axes())
}

///|
pub fn DebugRenderMode::contains(
  self : DebugRenderMode,
  other : DebugRenderMode,
) -> Bool {
  (self.bits & other.bits) == other.bits
}

///|
pub fn DebugRenderMode::insert(
  self : DebugRenderMode,
  other : DebugRenderMode,
) -> DebugRenderMode {
  self.bits = self.bits | other.bits
  self
}

///|
pub fn DebugRenderMode::remove(
  self : DebugRenderMode,
  other : DebugRenderMode,
) -> DebugRenderMode {
  self.bits = self.bits & Int::lnot(other.bits)
  self
}

///|
pub struct DebugRenderLine {
  object : DebugRenderObject
  a : @core.Vec2
  b : @core.Vec2
  color : DebugColor
}

///|
pub struct DebugRenderBackend {
  lines : Array[DebugRenderLine]
}

///|
pub fn DebugRenderBackend::new() -> DebugRenderBackend {
  { lines: [] }
}

///|
pub fn DebugRenderBackend::draw_line(
  self : DebugRenderBackend,
  object : DebugRenderObject,
  a : @core.Vec2,
  b : @core.Vec2,
  color : DebugColor,
) -> Unit {
  self.lines.push(DebugRenderLine::{ object, a, b, color })
}

///|
pub fn DebugRenderBackend::filter_object(
  self : DebugRenderBackend,
  object : DebugRenderObject,
) -> Bool {
  self |> ignore
  object |> ignore
  true
}

///|
pub fn DebugRenderBackend::draw_polyline(
  self : DebugRenderBackend,
  object : DebugRenderObject,
  vertices : Array[@core.Vec2],
  color : DebugColor,
  closed : Bool,
) -> Unit {
  if vertices.length() < 2 {
    return
  }
  for i in 0..<(vertices.length() - 1) {
    self.draw_line(object, vertices[i], vertices[i + 1], color)
  }
  if closed && vertices.length() > 2 {
    self.draw_line(object, vertices[vertices.length() - 1], vertices[0], color)
  }
}

///|
pub fn DebugRenderBackend::draw_line_strip(
  self : DebugRenderBackend,
  object : DebugRenderObject,
  vertices : Array[@core.Vec2],
  color : DebugColor,
  closed : Bool,
) -> Unit {
  self.draw_polyline(object, vertices, color, closed)
}

///|
pub fn DebugRenderBackend::lines(
  self : DebugRenderBackend,
) -> Array[DebugRenderLine] {
  let out : Array[DebugRenderLine] = []
  for i in 0..<self.lines.length() {
    out.push(self.lines[i])
  }
  out
}

///|
pub fn DebugRenderBackend::serialize_lines(self : DebugRenderBackend) -> String {
  // Deterministic, lossless enough snapshot for tests and consumers.
  let sb = StringBuilder::new()
  for i in 0..<self.lines.length() {
    let line = self.lines[i]
    if i > 0 {
      sb.write_char('\n')
    }
    sb.write_object(line.a.x)
    sb.write_char(',')
    sb.write_object(line.a.y)
    sb.write_char(' ')
    sb.write_object(line.b.x)
    sb.write_char(',')
    sb.write_object(line.b.y)
    sb.write_char(' ')
    sb.write_object(line.color.h)
    sb.write_char(',')
    sb.write_object(line.color.s)
    sb.write_char(',')
    sb.write_object(line.color.l)
    sb.write_char(',')
    sb.write_object(line.color.a)
    sb.write_char(' ')
    match line.object {
      DebugRenderObject::RigidBody(h) => {
        let (id, gen) = h.into_raw_parts()
        sb.write_string("rb(")
        sb.write_object(id)
        sb.write_char(',')
        sb.write_object(gen)
        sb.write_char(')')
      }
      DebugRenderObject::Collider(h) => {
        let (id, gen) = h.into_raw_parts()
        sb.write_string("co(")
        sb.write_object(id)
        sb.write_char(',')
        sb.write_object(gen)
        sb.write_char(')')
      }
      DebugRenderObject::ColliderAabb(h) => {
        let (id, gen) = h.into_raw_parts()
        sb.write_string("ca(")
        sb.write_object(id)
        sb.write_char(',')
        sb.write_object(gen)
        sb.write_char(')')
      }
      DebugRenderObject::ImpulseJoint(h) => {
        let (id, gen) = h.into_raw_parts()
        sb.write_string("ij(")
        sb.write_object(id)
        sb.write_char(',')
        sb.write_object(gen)
        sb.write_char(')')
      }
      DebugRenderObject::MultibodyJoint(h) => {
        let (id, _) = h.into_raw_parts()
        sb.write_string("mj(")
        sb.write_object(id)
        sb.write_char(')')
      }
      DebugRenderObject::ContactPair(a, b) => {
        let (a_id, a_gen) = a.into_raw_parts()
        let (b_id, b_gen) = b.into_raw_parts()
        sb.write_string("cp(")
        sb.write_object(a_id)
        sb.write_char(',')
        sb.write_object(a_gen)
        sb.write_char(';')
        sb.write_object(b_id)
        sb.write_char(',')
        sb.write_object(b_gen)
        sb.write_char(')')
      }
    }
  }
  sb.to_string()
}

///|
pub struct DebugRenderPipeline {
  style : DebugRenderStyle
  mode : DebugRenderMode
}

///|
pub fn DebugRenderPipeline::new(
  style : DebugRenderStyle,
  mode : DebugRenderMode,
) -> DebugRenderPipeline {
  { style, mode }
}

///|
pub fn DebugRenderPipeline::render_all(
  style : DebugRenderStyle,
) -> DebugRenderPipeline {
  DebugRenderPipeline::new(style, DebugRenderMode::all())
}

///|
pub fn DebugRenderPipeline::default() -> DebugRenderPipeline {
  DebugRenderPipeline::new(
    DebugRenderStyle::default(),
    DebugRenderMode::default(),
  )
}

///|
fn rigid_body_axes_lines(
  style : DebugRenderStyle,
  position : @core.Isometry2,
) -> (@core.Vec2, @core.Vec2, @core.Vec2) {
  let origin = position.translation
  let rot = position.rotation
  let len = style.rigid_body_axes_length
  let x_axis = @core.Vec2::new(rot.cos * len, rot.sin * len)
  let y_axis = @core.Vec2::new(-rot.sin * len, rot.cos * len)
  (origin, origin.add(x_axis), origin.add(y_axis))
}

///|
fn draw_cuboid_outline(
  backend : DebugRenderBackend,
  object : DebugRenderObject,
  center : @core.Vec2,
  angle : @core.Real,
  hw : @core.Real,
  hh : @core.Real,
  color : DebugColor,
) -> Unit {
  let rot = @core.Rot2::from_angle(angle)
  let ax = @core.Vec2::new(rot.cos, rot.sin)
  let ay = @core.Vec2::new(-rot.sin, rot.cos)
  let v00 = center.add(
    @core.Vec2::new(ax.x * hw + ay.x * hh, ax.y * hw + ay.y * hh),
  )
  let v10 = center.add(
    @core.Vec2::new(ax.x * hw - ay.x * hh, ax.y * hw - ay.y * hh),
  )
  let v11 = center.add(
    @core.Vec2::new(-ax.x * hw - ay.x * hh, -ax.y * hw - ay.y * hh),
  )
  let v01 = center.add(
    @core.Vec2::new(-ax.x * hw + ay.x * hh, -ax.y * hw + ay.y * hh),
  )
  backend.draw_line(object, v00, v10, color)
  backend.draw_line(object, v10, v11, color)
  backend.draw_line(object, v11, v01, color)
  backend.draw_line(object, v01, v00, color)
}

///|
fn draw_aabb_outline(
  backend : DebugRenderBackend,
  object : DebugRenderObject,
  min : @core.Vec2,
  max : @core.Vec2,
  color : DebugColor,
) -> Unit {
  let a = @core.Vec2::new(min.x, min.y)
  let b = @core.Vec2::new(max.x, min.y)
  let c = @core.Vec2::new(max.x, max.y)
  let d = @core.Vec2::new(min.x, max.y)
  backend.draw_line(object, a, b, color)
  backend.draw_line(object, b, c, color)
  backend.draw_line(object, c, d, color)
  backend.draw_line(object, d, a, color)
}

///|
pub fn DebugRenderPipeline::render_rigid_bodies(
  self : DebugRenderPipeline,
  backend : DebugRenderBackend,
  bodies : @dynamics.RigidBodySet,
) -> Unit {
  if self.mode.contains(DebugRenderMode::rigid_body_axes()) {
    for i in 0..<bodies.bodies.length() {
      let handle = @dynamics.RigidBodyHandle::from_raw_parts(
        i,
        bodies.generations[i],
      )
      if bodies.get(handle) is Some(body) {
        let obj = DebugRenderObject::RigidBody(handle)
        if backend.filter_object(obj) {
          let (o, x, y) = rigid_body_axes_lines(self.style, body.position())
          backend.draw_line(obj, o, x, self.style.contact_normal_color)
          backend.draw_line(obj, o, y, self.style.contact_depth_color)
        }
      }
    }
  }
}

///|
pub fn DebugRenderPipeline::render_colliders(
  self : DebugRenderPipeline,
  backend : DebugRenderBackend,
  colliders : @collision.ColliderSet,
) -> Unit {
  for i in 0..<colliders.colliders.length() {
    let handle = @collision.ColliderHandle::from_raw_parts(
      i,
      colliders.generations[i],
    )
    if colliders.get(handle) is Some(collider) {
      if !collider.is_enabled() {
        continue
      }
      let obj = DebugRenderObject::Collider(handle)
      if self.mode.contains(DebugRenderMode::collider_shapes()) {
        if !backend.filter_object(obj) {
          continue
        }
        match collider.shape() {
          @collision.Shape::Ball(r) => {
            // Deterministic approximation: draw two diameters.
            let c = collider.translation()
            let a = @core.Vec2::new(c.x - r, c.y)
            let b = @core.Vec2::new(c.x + r, c.y)
            let d = @core.Vec2::new(c.x, c.y - r)
            let e = @core.Vec2::new(c.x, c.y + r)
            backend.draw_line(obj, a, b, self.style.collider_dynamic_color)
            backend.draw_line(obj, d, e, self.style.collider_dynamic_color)
          }
          @collision.Shape::Cuboid(hw, hh) =>
            draw_cuboid_outline(
              backend,
              obj,
              collider.translation(),
              collider.rotation(),
              hw,
              hh,
              self.style.collider_dynamic_color,
            )
          _ => ()
        }
      }
      if self.mode.contains(DebugRenderMode::collider_aabbs()) {
        let aabb_obj = DebugRenderObject::ColliderAabb(handle)
        if !backend.filter_object(aabb_obj) {
          continue
        }
        match collider.shape() {
          @collision.Shape::Ball(r) => {
            let c = collider.translation()
            draw_aabb_outline(
              backend,
              aabb_obj,
              @core.Vec2::new(c.x - r, c.y - r),
              @core.Vec2::new(c.x + r, c.y + r),
              self.style.collider_aabb_color,
            )
          }
          @collision.Shape::Cuboid(hw, hh) => {
            // Conservative AABB for an arbitrarily rotated cuboid.
            let c = collider.translation()
            let angle = collider.rotation()
            let rot = @core.Rot2::from_angle(angle)
            let ax = @core.abs(rot.cos) * hw + @core.abs(rot.sin) * hh
            let ay = @core.abs(rot.sin) * hw + @core.abs(rot.cos) * hh
            draw_aabb_outline(
              backend,
              aabb_obj,
              @core.Vec2::new(c.x - ax, c.y - ay),
              @core.Vec2::new(c.x + ax, c.y + ay),
              self.style.collider_aabb_color,
            )
          }
          _ => ()
        }
      }
    }
  }
}

///|
pub fn DebugRenderPipeline::render_joints(
  self : DebugRenderPipeline,
  backend : DebugRenderBackend,
  bodies : @dynamics.RigidBodySet,
  impulse_joints : @dynamics.ImpulseJointSet,
  multibody_joints : @dynamics.MultibodyJointSet,
) -> Unit {
  if self.mode.contains(DebugRenderMode::impulse_joints()) {
    let graph = impulse_joints.joint_graph()
    let edges = graph.edges()
    for i in 0..<edges.length() {
      let edge = edges[i]
      let b1 = edge.0
      let b2 = edge.1
      if bodies.get(b1) is Some(rb1) && bodies.get(b2) is Some(rb2) {
        let obj = DebugRenderObject::ImpulseJoint(edge.2.handle)
        if !backend.filter_object(obj) {
          continue
        }
        backend.draw_line(
          obj,
          rb1.translation(),
          rb2.translation(),
          self.style.impulse_joint_separation_color,
        )
      }
    }
  }
  if self.mode.contains(DebugRenderMode::multibody_joints()) {
    let descs = multibody_joints.revolute_joint_descriptors()
    for i in 0..<descs.length() {
      let d = descs[i]
      let parent = d.parent
      let child = d.child
      let (child_id, child_gen) = child.into_raw_parts()
      let handle = @dynamics.MultibodyJointHandle::from_raw_parts(
        child_id, child_gen,
      )
      if bodies.get(parent) is Some(rb1) && bodies.get(child) is Some(rb2) {
        let obj = DebugRenderObject::MultibodyJoint(handle)
        if !backend.filter_object(obj) {
          continue
        }
        backend.draw_line(
          obj,
          rb1.translation(),
          rb2.translation(),
          self.style.multibody_joint_separation_color,
        )
      }
    }
  }
}

///|
pub fn DebugRenderPipeline::render_contacts(
  self : DebugRenderPipeline,
  backend : DebugRenderBackend,
  colliders : @collision.ColliderSet,
  narrow_phase : @collision.NarrowPhase,
) -> Unit {
  self |> ignore
  backend |> ignore
  colliders |> ignore
  narrow_phase |> ignore
}

///|
pub fn DebugRenderPipeline::render(
  self : DebugRenderPipeline,
  backend : DebugRenderBackend,
  bodies : @dynamics.RigidBodySet,
  colliders : @collision.ColliderSet,
  impulse_joints : @dynamics.ImpulseJointSet,
  multibody_joints : @dynamics.MultibodyJointSet,
  narrow_phase : @collision.NarrowPhase,
) -> Unit {
  self.render_rigid_bodies(backend, bodies)
  self.render_colliders(backend, colliders)
  self.render_joints(backend, bodies, impulse_joints, multibody_joints)
  self.render_contacts(backend, colliders, narrow_phase)
}
