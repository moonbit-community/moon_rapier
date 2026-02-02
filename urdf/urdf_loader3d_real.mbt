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
/// URDF loader used by the real dim3 backend parity tests.
///
/// This parses a URDF into a rigid-body tree with joints and colliders, mirroring the `rapier3d-urdf`
/// approach. For mesh geometries, this loader relies on an explicit mesh-bounds map so it can run in
/// environments without filesystem I/O or mesh parsing.

///|
pub(all) enum UrdfJointType {
  Fixed
  Revolute
  Continuous
  Prismatic
  Floating
  Planar
  Spherical
}

///|
pub struct UrdfLoaderOptions3DReal {
  apply_imported_mass_props : Bool
  create_colliders_from_visual_shapes : Bool
  create_colliders_from_collision_shapes : Bool
  enable_joint_collisions : Bool
  make_roots_fixed : Bool
  shift : @core.Isometry3
}

///|
pub fn UrdfLoaderOptions3DReal::default() -> UrdfLoaderOptions3DReal {
  {
    apply_imported_mass_props: true,
    create_colliders_from_visual_shapes: false,
    create_colliders_from_collision_shapes: true,
    enable_joint_collisions: false,
    make_roots_fixed: false,
    shift: @core.Isometry3::identity(),
  }
}

///|
pub fn UrdfLoaderOptions3DReal::apply_imported_mass_props(
  self : UrdfLoaderOptions3DReal,
  enabled : Bool,
) -> UrdfLoaderOptions3DReal {
  {
    apply_imported_mass_props: enabled,
    create_colliders_from_visual_shapes: self.create_colliders_from_visual_shapes,
    create_colliders_from_collision_shapes: self.create_colliders_from_collision_shapes,
    enable_joint_collisions: self.enable_joint_collisions,
    make_roots_fixed: self.make_roots_fixed,
    shift: self.shift,
  }
}

///|
pub fn UrdfLoaderOptions3DReal::create_colliders_from_visual_shapes(
  self : UrdfLoaderOptions3DReal,
  enabled : Bool,
) -> UrdfLoaderOptions3DReal {
  {
    apply_imported_mass_props: self.apply_imported_mass_props,
    create_colliders_from_visual_shapes: enabled,
    create_colliders_from_collision_shapes: self.create_colliders_from_collision_shapes,
    enable_joint_collisions: self.enable_joint_collisions,
    make_roots_fixed: self.make_roots_fixed,
    shift: self.shift,
  }
}

///|
pub fn UrdfLoaderOptions3DReal::create_colliders_from_collision_shapes(
  self : UrdfLoaderOptions3DReal,
  enabled : Bool,
) -> UrdfLoaderOptions3DReal {
  {
    apply_imported_mass_props: self.apply_imported_mass_props,
    create_colliders_from_visual_shapes: self.create_colliders_from_visual_shapes,
    create_colliders_from_collision_shapes: enabled,
    enable_joint_collisions: self.enable_joint_collisions,
    make_roots_fixed: self.make_roots_fixed,
    shift: self.shift,
  }
}

///|
pub fn UrdfLoaderOptions3DReal::enable_joint_collisions(
  self : UrdfLoaderOptions3DReal,
  enabled : Bool,
) -> UrdfLoaderOptions3DReal {
  {
    apply_imported_mass_props: self.apply_imported_mass_props,
    create_colliders_from_visual_shapes: self.create_colliders_from_visual_shapes,
    create_colliders_from_collision_shapes: self.create_colliders_from_collision_shapes,
    enable_joint_collisions: enabled,
    make_roots_fixed: self.make_roots_fixed,
    shift: self.shift,
  }
}

///|
pub fn UrdfLoaderOptions3DReal::make_roots_fixed(
  self : UrdfLoaderOptions3DReal,
  enabled : Bool,
) -> UrdfLoaderOptions3DReal {
  {
    apply_imported_mass_props: self.apply_imported_mass_props,
    create_colliders_from_visual_shapes: self.create_colliders_from_visual_shapes,
    create_colliders_from_collision_shapes: self.create_colliders_from_collision_shapes,
    enable_joint_collisions: self.enable_joint_collisions,
    make_roots_fixed: enabled,
    shift: self.shift,
  }
}

///|
pub fn UrdfLoaderOptions3DReal::shift(
  self : UrdfLoaderOptions3DReal,
  shift : @core.Isometry3,
) -> UrdfLoaderOptions3DReal {
  {
    apply_imported_mass_props: self.apply_imported_mass_props,
    create_colliders_from_visual_shapes: self.create_colliders_from_visual_shapes,
    create_colliders_from_collision_shapes: self.create_colliders_from_collision_shapes,
    enable_joint_collisions: self.enable_joint_collisions,
    make_roots_fixed: self.make_roots_fixed,
    shift,
  }
}

///|
pub struct UrdfVisual {
  origin : @core.Isometry3
  geometry : UrdfGeometry3DReal
}

///|
pub(all) enum UrdfGeometry3DReal {
  Box(@core.Vec3) // full extents
  Sphere(@core.Real)
  Cylinder(@core.Real, @core.Real) // radius, length (along +Z in URDF)
  Capsule(@core.Real, @core.Real) // radius, length (along +Z in URDF)
  Mesh(String, @core.Vec3) // basename, scale
}

///|
pub struct UrdfInertial3DReal {
  origin : @core.Isometry3
  mass : @core.Real
  inertia : @core.SdpMat3
}

///|
pub struct UrdfLink {
  name : String
  visuals : Array[UrdfVisual]
  collisions : Array[UrdfVisual]
  mut inertial : UrdfInertial3DReal?
}

///|
pub struct UrdfJoint {
  name : String
  joint_type : UrdfJointType
  mut parent : String
  mut child : String
  mut origin : @core.Isometry3
  mut axis : @core.Vec3
  mut limit_lower : @core.Real
  mut limit_upper : @core.Real
  mut has_limits : Bool
}

///|
pub struct UrdfRobot3DReal {
  links : Array[UrdfLink]
  joints : Array[UrdfJoint]
}

///|
fn urdf_parse_real(text : String) -> @core.Real {
  let result : Result[Double, @strconv.StrConvError] = try? @strconv.parse_double(
    text[:],
  )
  if result is Ok(v) {
    Float::from_double(v)
  } else {
    0.0F
  }
}

///|
fn urdf_split_ws(text : String) -> Array[String] {
  let parts : Array[String] = []
  let cur = StringBuilder::new()
  for c in text {
    if c == ' ' || c == '\n' || c == '\r' || c == '\t' {
      if !cur.is_empty() {
        parts.push(cur.to_string())
        cur.reset()
      }
    } else {
      cur.write_char(c)
    }
  }
  if !cur.is_empty() {
    parts.push(cur.to_string())
  }
  parts
}

///|
fn urdf_parse_vec3(text : String) -> @core.Vec3 {
  let parts = urdf_split_ws(text)
  if parts.length() < 3 {
    return @core.Vec3::zero()
  }
  @core.Vec3::new(
    urdf_parse_real(parts[0]),
    urdf_parse_real(parts[1]),
    urdf_parse_real(parts[2]),
  )
}

///|
fn urdf_quat_from_rpy(rpy : @core.Vec3) -> @core.Quat {
  let qx = @core.rotation_from_scaled_axis(@core.Vec3::new(rpy.x, 0.0F, 0.0F))
  let qy = @core.rotation_from_scaled_axis(@core.Vec3::new(0.0F, rpy.y, 0.0F))
  let qz = @core.rotation_from_scaled_axis(@core.Vec3::new(0.0F, 0.0F, rpy.z))
  // Apply roll (X), then pitch (Y), then yaw (Z).
  qz.mul(qy.mul(qx))
}

///|
fn urdf_pose_from_xyz_rpy(
  xyz : @core.Vec3,
  rpy : @core.Vec3,
) -> @core.Isometry3 {
  @core.Isometry3::new(xyz, urdf_quat_from_rpy(rpy))
}

///|
priv struct XmlTag {
  name : String
  attrs : @hashmap.HashMap[String, String]
  is_end : Bool
}

///|
fn xml_is_ws(ch : Char) -> Bool {
  ch == ' ' || ch == '\n' || ch == '\r' || ch == '\t'
}

///|
fn xml_skip_ws(s : String, i : Int) -> Int {
  let mut j = i
  while j < s.length() {
    let c = Int::unsafe_to_char(s.code_unit_at(j).to_int())
    if !xml_is_ws(c) {
      break
    }
    j = j + 1
  }
  j
}

///|
fn xml_parse_ident(s : String, i : Int) -> (String, Int) {
  let mut j = i
  let b = StringBuilder::new()
  while j < s.length() {
    let c = Int::unsafe_to_char(s.code_unit_at(j).to_int())
    if xml_is_ws(c) || c == '>' || c == '/' || c == '=' {
      break
    }
    b.write_char(c)
    j = j + 1
  }
  (b.to_string(), j)
}

///|
fn xml_parse_quoted(s : String, i : Int) -> (String, Int) {
  let mut j = i
  if j >= s.length() {
    return ("", j)
  }
  let quote = Int::unsafe_to_char(s.code_unit_at(j).to_int())
  if quote != '"' && quote != '\'' {
    return ("", j)
  }
  j = j + 1
  let b = StringBuilder::new()
  while j < s.length() {
    let c = Int::unsafe_to_char(s.code_unit_at(j).to_int())
    if c == quote {
      j = j + 1
      break
    }
    b.write_char(c)
    j = j + 1
  }
  (b.to_string(), j)
}

///|
fn xml_parse_tag(s : String, start : Int) -> (XmlTag?, Int) {
  let mut i = start
  if i >= s.length() || Int::unsafe_to_char(s.code_unit_at(i).to_int()) != '<' {
    return (None, i + 1)
  }
  i = i + 1
  i = xml_skip_ws(s, i)
  let mut is_end = false
  if i < s.length() && Int::unsafe_to_char(s.code_unit_at(i).to_int()) == '/' {
    is_end = true
    i = i + 1
    i = xml_skip_ws(s, i)
  }
  let (name, j0) = xml_parse_ident(s, i)
  let mut j = j0
  if name == "" {
    return (None, start + 1)
  }
  let attrs : @hashmap.HashMap[String, String] = @hashmap.new()
  j = xml_skip_ws(s, j)
  while j < s.length() {
    let c = Int::unsafe_to_char(s.code_unit_at(j).to_int())
    if c == '>' {
      j = j + 1
      break
    }
    if c == '/' {
      // '/>' self-closing tag.
      j = j + 1
      j = xml_skip_ws(s, j)
      if j < s.length() &&
        Int::unsafe_to_char(s.code_unit_at(j).to_int()) == '>' {
        j = j + 1
      }
      break
    }
    // Attribute key.
    let (key, k0) = xml_parse_ident(s, j)
    let mut k = xml_skip_ws(s, k0)
    if k < s.length() && Int::unsafe_to_char(s.code_unit_at(k).to_int()) == '=' {
      k = k + 1
      k = xml_skip_ws(s, k)
      let (value, k2) = xml_parse_quoted(s, k)
      attrs.set(key, value)
      j = xml_skip_ws(s, k2)
    } else {
      // Malformed; skip.
      j = xml_skip_ws(s, k)
    }
  }
  (Some({ name, attrs, is_end }), j)
}

///|
fn xml_iter_tags(s : String) -> Array[XmlTag] {
  let tags : Array[XmlTag] = []
  let mut i = 0
  while i < s.length() {
    let c = Int::unsafe_to_char(s.code_unit_at(i).to_int())
    if c == '<' {
      let (tag_opt, next) = xml_parse_tag(s, i)
      if tag_opt is Some(tag) {
        tags.push(tag)
        i = next
        continue
      }
    }
    i = i + 1
  }
  tags
}

///|
fn attr_get(attrs : @hashmap.HashMap[String, String], key : String) -> String? {
  attrs.get(key)
}

///|
fn basename(path : String) -> String {
  let mut last = -1
  for i in 0..<path.length() {
    if Int::unsafe_to_char(path.code_unit_at(i).to_int()) == '/' {
      last = i
    }
  }
  if last < 0 {
    path
  } else {
    // Safe: indices are within bounds.
    (try! path[last + 1:path.length()]).to_string()
  }
}

///|
fn parse_joint_type(s : String) -> UrdfJointType {
  if s == "revolute" {
    UrdfJointType::Revolute
  } else if s == "continuous" {
    UrdfJointType::Continuous
  } else if s == "prismatic" {
    UrdfJointType::Prismatic
  } else if s == "fixed" {
    UrdfJointType::Fixed
  } else if s == "floating" {
    UrdfJointType::Floating
  } else if s == "planar" {
    UrdfJointType::Planar
  } else if s == "spherical" {
    UrdfJointType::Spherical
  } else {
    UrdfJointType::Fixed
  }
}

///|
fn urdf_parse_positive_scale(text : String) -> @core.Vec3 {
  let s = urdf_parse_vec3(text)
  let x = if s.x <= 0.0F { 1.0F } else { s.x }
  let y = if s.y <= 0.0F { 1.0F } else { s.y }
  let z = if s.z <= 0.0F { 1.0F } else { s.z }
  @core.Vec3::new(x, y, z)
}

///|
pub fn UrdfRobot3DReal::from_xml(xml : String) -> UrdfRobot3DReal? {
  let tags = xml_iter_tags(xml)
  let links : Array[UrdfLink] = []
  let joints : Array[UrdfJoint] = []
  let mut cur_link : UrdfLink? = None
  let mut cur_joint : UrdfJoint? = None
  let mut in_visual = false
  let mut in_collision = false
  let mut in_inertial = false
  let mut cur_geom : UrdfGeometry3DReal? = None
  let mut cur_origin = @core.Isometry3::identity()
  let mut cur_inertial_origin = @core.Isometry3::identity()
  let mut cur_inertial_mass = 0.0F
  let mut cur_inertial_inertia = @core.SdpMat3::zero()
  fn push_shape(
    cur_link : UrdfLink?,
    in_visual : Bool,
    in_collision : Bool,
    cur_origin : @core.Isometry3,
    cur_geom : UrdfGeometry3DReal?,
  ) -> UrdfLink? {
    if cur_link is Some(l) {
      if (in_visual || in_collision) && cur_geom is Some(g) {
        if in_visual {
          l.visuals.push({ origin: cur_origin, geometry: g })
        } else {
          l.collisions.push({ origin: cur_origin, geometry: g })
        }
      }
      Some(l)
    } else {
      None
    }
  }

  for tag in tags {
    if tag.is_end {
      if tag.name == "link" {
        if cur_link is Some(l) {
          links.push(l)
        }
        cur_link = None
        in_visual = false
        in_collision = false
        in_inertial = false
        cur_geom = None
        cur_origin = @core.Isometry3::identity()
        cur_inertial_origin = @core.Isometry3::identity()
        cur_inertial_mass = 0.0F
        cur_inertial_inertia = @core.SdpMat3::zero()
      } else if tag.name == "joint" {
        if cur_joint is Some(j) {
          joints.push(j)
        }
        cur_joint = None
      } else if tag.name == "visual" {
        cur_link = push_shape(cur_link, true, false, cur_origin, cur_geom)
        cur_geom = None
        cur_origin = @core.Isometry3::identity()
        in_visual = false
      } else if tag.name == "collision" {
        cur_link = push_shape(cur_link, false, true, cur_origin, cur_geom)
        cur_geom = None
        cur_origin = @core.Isometry3::identity()
        in_collision = false
      } else if tag.name == "inertial" {
        if cur_link is Some(l) {
          if cur_inertial_mass > 0.0F {
            l.inertial = Some({
              origin: cur_inertial_origin,
              mass: cur_inertial_mass,
              inertia: cur_inertial_inertia,
            })
          }
          cur_link = Some(l)
        }
        in_inertial = false
        cur_inertial_origin = @core.Isometry3::identity()
        cur_inertial_mass = 0.0F
        cur_inertial_inertia = @core.SdpMat3::zero()
      }
      continue
    }
    if tag.name == "link" {
      if attr_get(tag.attrs, "name") is Some(name) {
        cur_link = Some({ name, visuals: [], collisions: [], inertial: None })
      }
      continue
    }
    if tag.name == "joint" {
      let name = if attr_get(tag.attrs, "name") is Some(n) { n } else { "" }
      let jt = if attr_get(tag.attrs, "type") is Some(t) {
        parse_joint_type(t)
      } else {
        UrdfJointType::Fixed
      }
      cur_joint = Some({
        name,
        joint_type: jt,
        parent: "",
        child: "",
        origin: @core.Isometry3::identity(),
        axis: @core.Vec3::new(1.0F, 0.0F, 0.0F),
        limit_lower: 0.0F,
        limit_upper: 0.0F,
        has_limits: false,
      })
      continue
    }
    if tag.name == "visual" {
      in_visual = true
      cur_geom = None
      cur_origin = @core.Isometry3::identity()
      continue
    }
    if tag.name == "collision" {
      in_collision = true
      cur_geom = None
      cur_origin = @core.Isometry3::identity()
      continue
    }
    if tag.name == "inertial" {
      in_inertial = true
      cur_inertial_origin = @core.Isometry3::identity()
      cur_inertial_mass = 0.0F
      cur_inertial_inertia = @core.SdpMat3::zero()
      continue
    }
    if tag.name == "origin" {
      let xyz = if attr_get(tag.attrs, "xyz") is Some(v) {
        urdf_parse_vec3(v)
      } else {
        @core.Vec3::zero()
      }
      let rpy = if attr_get(tag.attrs, "rpy") is Some(v) {
        urdf_parse_vec3(v)
      } else {
        @core.Vec3::zero()
      }
      let pose = urdf_pose_from_xyz_rpy(xyz, rpy)
      if cur_joint is Some(j) {
        j.origin = pose
        cur_joint = Some(j)
      } else if in_inertial {
        cur_inertial_origin = pose
      } else if in_visual || in_collision {
        cur_origin = pose
      }
      continue
    }
    if tag.name == "mass" && in_inertial {
      if attr_get(tag.attrs, "value") is Some(v) {
        cur_inertial_mass = urdf_parse_real(v)
      }
      continue
    }
    if tag.name == "inertia" && in_inertial {
      fn a(
        attrs : @hashmap.HashMap[String, String],
        key : String,
      ) -> @core.Real {
        if attr_get(attrs, key) is Some(v) {
          urdf_parse_real(v)
        } else {
          0.0F
        }
      }

      let ixx = a(tag.attrs, "ixx")
      let ixy = a(tag.attrs, "ixy")
      let ixz = a(tag.attrs, "ixz")
      let iyy = a(tag.attrs, "iyy")
      let iyz = a(tag.attrs, "iyz")
      let izz = a(tag.attrs, "izz")
      cur_inertial_inertia = @core.SdpMat3::new(ixx, ixy, ixz, iyy, iyz, izz)
      continue
    }
    if tag.name == "mesh" {
      if (in_visual || in_collision) &&
        attr_get(tag.attrs, "filename") is Some(f) {
        let scale = if attr_get(tag.attrs, "scale") is Some(s) {
          urdf_parse_positive_scale(s)
        } else {
          @core.Vec3::new(1.0F, 1.0F, 1.0F)
        }
        cur_geom = Some(UrdfGeometry3DReal::Mesh(basename(f), scale))
      }
      continue
    }
    if tag.name == "box" {
      if (in_visual || in_collision) && attr_get(tag.attrs, "size") is Some(v) {
        cur_geom = Some(UrdfGeometry3DReal::Box(urdf_parse_vec3(v)))
      }
      continue
    }
    if tag.name == "sphere" {
      if (in_visual || in_collision) && attr_get(tag.attrs, "radius") is Some(v) {
        cur_geom = Some(UrdfGeometry3DReal::Sphere(urdf_parse_real(v)))
      }
      continue
    }
    if tag.name == "cylinder" {
      if (in_visual || in_collision) &&
        attr_get(tag.attrs, "radius") is Some(r) &&
        attr_get(tag.attrs, "length") is Some(l) {
        cur_geom = Some(
          UrdfGeometry3DReal::Cylinder(urdf_parse_real(r), urdf_parse_real(l)),
        )
      }
      continue
    }
    if tag.name == "capsule" {
      if (in_visual || in_collision) &&
        attr_get(tag.attrs, "radius") is Some(r) &&
        attr_get(tag.attrs, "length") is Some(l) {
        cur_geom = Some(
          UrdfGeometry3DReal::Capsule(urdf_parse_real(r), urdf_parse_real(l)),
        )
      }
      continue
    }
    if tag.name == "parent" && cur_joint is Some(j) {
      if attr_get(tag.attrs, "link") is Some(p) {
        j.parent = p
        cur_joint = Some(j)
      }
      continue
    }
    if tag.name == "child" && cur_joint is Some(j) {
      if attr_get(tag.attrs, "link") is Some(c) {
        j.child = c
        cur_joint = Some(j)
      }
      continue
    }
    if tag.name == "axis" && cur_joint is Some(j) {
      if attr_get(tag.attrs, "xyz") is Some(v) {
        j.axis = urdf_parse_vec3(v)
        cur_joint = Some(j)
      }
      continue
    }
    if tag.name == "limit" && cur_joint is Some(j) {
      if attr_get(tag.attrs, "lower") is Some(lo) &&
        attr_get(tag.attrs, "upper") is Some(hi) {
        j.limit_lower = urdf_parse_real(lo)
        j.limit_upper = urdf_parse_real(hi)
        j.has_limits = true
        cur_joint = Some(j)
      }
      continue
    }
  }
  if links.length() == 0 {
    None
  } else {
    Some({ links, joints })
  }
}

///|
pub fn UrdfRobot3DReal::insert_3d_real(
  self : UrdfRobot3DReal,
  options : UrdfLoaderOptions3DReal,
  mesh_bounds : @hashmap.HashMap[String, (@core.Vec3, @core.Vec3)],
  bodies : @dynamics.RigidBodySet3D,
  colliders : @collision.ColliderSet3D,
  joints : @dynamics.JointSet3DReal,
  base_transform : @core.Isometry3,
) -> Unit {
  let link_to_body : @hashmap.HashMap[String, @dynamics.RigidBodyHandle] = @hashmap.new(
    capacity=self.links.length(),
  )
  let link_pose_urdf : @hashmap.HashMap[String, @core.Isometry3] = @hashmap.new(
    capacity=self.links.length(),
  )
  let link_visuals : @hashmap.HashMap[String, Array[UrdfVisual]] = @hashmap.new(
    capacity=self.links.length(),
  )
  let link_collisions : @hashmap.HashMap[String, Array[UrdfVisual]] = @hashmap.new(
    capacity=self.links.length(),
  )
  let link_inertials : @hashmap.HashMap[String, UrdfInertial3DReal?] = @hashmap.new(
    capacity=self.links.length(),
  )
  for i in 0..<self.links.length() {
    let l = self.links[i]
    link_visuals.set(l.name, l.visuals)
    link_collisions.set(l.name, l.collisions)
    link_inertials.set(l.name, l.inertial)
  }
  let children : @hashset.HashSet[String] = @hashset.new(
    capacity=self.joints.length(),
  )
  for j in self.joints {
    children.add(j.child)
  }
  let roots : Array[String] = []
  for i in 0..<self.links.length() {
    let name = self.links[i].name
    if !children.contains(name) {
      roots.push(name)
    }
  }
  let shift = options.shift

  // Insert roots.
  for i in 0..<roots.length() {
    let name = roots[i]
    let fixed_root = options.make_roots_fixed
    let mut rb_builder = if fixed_root {
      @dynamics.RigidBodyBuilder3D::fixed()
    } else {
      @dynamics.RigidBodyBuilder3D::dynamic()
    }
    if !fixed_root &&
      options.apply_imported_mass_props &&
      link_inertials.get(name) is Some(inert) &&
      inert is Some(ine) &&
      ine.mass > 0.0F {
      let mp0 = @core.MassProperties3::new(
        ine.mass,
        ine.inertia,
        @core.Vec3::zero(),
      )
      rb_builder = rb_builder.mass_properties(mp0.transform_by(ine.origin))
    }
    let h = bodies.insert(rb_builder.build())
    if bodies.get(h) is Some(b) {
      // Root pose is `base_transform` in URDF space, then a global `shift` (e.g. Z-up -> Y-up).
      let pose = shift.mul(base_transform)
      b.set_translation(pose.translation)
      b.set_rotation(pose.rotation)
    }
    link_to_body.set(name, h)
    link_pose_urdf.set(name, base_transform)
  }
  fn attach_visuals(
    link_name : String,
    body : @dynamics.RigidBodyHandle,
    options : UrdfLoaderOptions3DReal,
    mesh_bounds : @hashmap.HashMap[String, (@core.Vec3, @core.Vec3)],
    link_visuals : @hashmap.HashMap[String, Array[UrdfVisual]],
    link_collisions : @hashmap.HashMap[String, Array[UrdfVisual]],
    colliders : @collision.ColliderSet3D,
    bodies : @dynamics.RigidBodySet3D,
  ) -> Unit {
    let mut list : Array[UrdfVisual] = []
    if options.create_colliders_from_visual_shapes &&
      link_visuals.get(link_name) is Some(vs) {
      list = vs
    } else if options.create_colliders_from_collision_shapes &&
      link_collisions.get(link_name) is Some(cs) {
      list = cs
    }
    for i in 0..<list.length() {
      let v = list[i]
      match v.geometry {
        UrdfGeometry3DReal::Box(size) => {
          let he = size.scale(0.5F)
          colliders.insert_with_parent(
            @collision.ColliderBuilder3D::cuboid(he.x, he.y, he.z)
            .position(v.origin)
            .build(),
            body,
            bodies,
          )
          |> ignore
        }
        UrdfGeometry3DReal::Sphere(r) =>
          colliders.insert_with_parent(
            @collision.ColliderBuilder3D::ball(r).position(v.origin).build(),
            body,
            bodies,
          )
          |> ignore
        UrdfGeometry3DReal::Cylinder(r, len) => {
          // URDF cylinders are aligned along +Z. Our cylinder is aligned along +Y.
          let rot = @core.rotation_from_scaled_axis(
            @core.Vec3::new(-@core.pi() * 0.5F, 0.0F, 0.0F),
          )
          let extra = @core.Isometry3::new(@core.Vec3::zero(), rot)
          colliders.insert_with_parent(
            @collision.ColliderBuilder3D::cylinder(r, len * 0.5F)
            .position(v.origin.mul(extra))
            .build(),
            body,
            bodies,
          )
          |> ignore
        }
        UrdfGeometry3DReal::Capsule(r, len) => {
          // URDF capsules are aligned along +Z. Our capsule is aligned along +Y.
          let rot = @core.rotation_from_scaled_axis(
            @core.Vec3::new(-@core.pi() * 0.5F, 0.0F, 0.0F),
          )
          let extra = @core.Isometry3::new(@core.Vec3::zero(), rot)
          colliders.insert_with_parent(
            @collision.ColliderBuilder3D::capsule_y(r, len * 0.5F)
            .position(v.origin.mul(extra))
            .build(),
            body,
            bodies,
          )
          |> ignore
        }
        UrdfGeometry3DReal::Mesh(mesh, scale) =>
          if mesh_bounds.get(mesh) is Some((mins0, maxs0)) {
            let mins = @core.Vec3::new(
              mins0.x * scale.x,
              mins0.y * scale.y,
              mins0.z * scale.z,
            )
            let maxs = @core.Vec3::new(
              maxs0.x * scale.x,
              maxs0.y * scale.y,
              maxs0.z * scale.z,
            )
            let center = mins.add(maxs).scale(0.5F)
            let he = maxs.sub(mins).scale(0.5F)
            let local_pose = v.origin.mul(
              @core.Isometry3::from_translation(center),
            )
            colliders.insert_with_parent(
              @collision.ColliderBuilder3D::cuboid(he.x, he.y, he.z)
              .position(local_pose)
              .build(),
              body,
              bodies,
            )
            |> ignore
          } else {
            ()
          }
      }
    }
  }

  // Attach visuals for roots immediately.
  for i in 0..<roots.length() {
    let name = roots[i]
    if link_to_body.get(name) is Some(h) {
      attach_visuals(
        name, h, options, mesh_bounds, link_visuals, link_collisions, colliders,
        bodies,
      )
    }
  }

  // Insert joints/children in a simple topological pass.
  let remaining : Array[UrdfJoint] = []
  for j in self.joints {
    remaining.push(j)
  }
  let mut iter_guard : Int = 0
  while remaining.length() > 0 && iter_guard < 2048 {
    iter_guard = iter_guard + 1
    let mut progressed = false
    let mut i = 0
    while i < remaining.length() {
      let j = remaining[i]
      if link_to_body.get(j.parent) is Some(parent_h) &&
        link_pose_urdf.get(j.parent) is Some(parent_pose_urdf) {
        // Compute child pose in URDF space, then apply the global shift.
        let child_pose_urdf = parent_pose_urdf.mul(j.origin)
        let child_pose = shift.mul(child_pose_urdf)
        let mut child_builder = @dynamics.RigidBodyBuilder3D::dynamic()
        if options.apply_imported_mass_props &&
          link_inertials.get(j.child) is Some(inert) &&
          inert is Some(ine) &&
          ine.mass > 0.0F {
          let mp0 = @core.MassProperties3::new(
            ine.mass,
            ine.inertia,
            @core.Vec3::zero(),
          )
          child_builder = child_builder.mass_properties(
            mp0.transform_by(ine.origin),
          )
        }
        let child_h = bodies.insert(child_builder.build())
        if bodies.get(child_h) is Some(rb) {
          rb.set_translation(child_pose.translation)
          rb.set_rotation(child_pose.rotation)
        }
        link_to_body.set(j.child, child_h)
        link_pose_urdf.set(j.child, child_pose_urdf)

        // Build the joint configuration.
        let axis = if j.axis.length_squared() <= 1.0e-12F {
          @core.Vec3::new(1.0F, 0.0F, 0.0F)
        } else {
          j.axis.normalize()
        }
        let axis1 = j.origin.rotation.rotate_vec3(axis)
        let locked = match j.joint_type {
          UrdfJointType::Fixed => @dynamics.JointAxesMask3DReal::all()
          UrdfJointType::Revolute =>
            @dynamics.JointAxesMask3DReal::lin_axes()
            .or(@dynamics.JointAxesMask3DReal::ang_y())
            .or(@dynamics.JointAxesMask3DReal::ang_z())
          UrdfJointType::Continuous =>
            @dynamics.JointAxesMask3DReal::lin_axes()
            .or(@dynamics.JointAxesMask3DReal::ang_y())
            .or(@dynamics.JointAxesMask3DReal::ang_z())
          UrdfJointType::Prismatic =>
            @dynamics.JointAxesMask3DReal::lin_y()
            .or(@dynamics.JointAxesMask3DReal::lin_z())
            .or(@dynamics.JointAxesMask3DReal::ang_axes())
          UrdfJointType::Spherical => @dynamics.JointAxesMask3DReal::lin_axes()
          UrdfJointType::Planar =>
            @dynamics.JointAxesMask3DReal::lin_x()
            .or(@dynamics.JointAxesMask3DReal::ang_y())
            .or(@dynamics.JointAxesMask3DReal::ang_z())
          UrdfJointType::Floating => @dynamics.JointAxesMask3DReal::empty()
        }
        let mut builder = @dynamics.GenericJoint3DRealBuilder::new(locked)
          .contacts_enabled(options.enable_joint_collisions)
          .local_anchor1(j.origin.translation)
          .local_anchor2(@core.Vec3::zero())
          .local_basis1(j.origin.rotation)
          .local_basis2(@core.Quat::identity())
          .local_axis1(axis1)
          .local_axis2(axis)
        match j.joint_type {
          UrdfJointType::Revolute =>
            if j.has_limits {
              builder = builder.limits(
                @dynamics.JointAxis3DReal::AngX,
                j.limit_lower,
                j.limit_upper,
              )
            }
          UrdfJointType::Continuous => ()
          UrdfJointType::Prismatic =>
            if j.has_limits {
              builder = builder.limits(
                @dynamics.JointAxis3DReal::LinX,
                j.limit_lower,
                j.limit_upper,
              )
            }
          _ => ()
        }
        joints.insert_generic(parent_h, child_h, builder.build(), true)
        |> ignore
        attach_visuals(
          j.child,
          child_h,
          options,
          mesh_bounds,
          link_visuals,
          link_collisions,
          colliders,
          bodies,
        )

        // Remove by swap-pop.
        remaining[i] = remaining[remaining.length() - 1]
        remaining.pop() |> ignore
        progressed = true
        continue
      }
      i = i + 1
    }
    if !progressed {
      break
    }
  }
}
