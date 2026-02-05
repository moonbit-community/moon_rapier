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
pub struct ContactPair3D {
  manifolds : Array[ContactPoint3D]
  impulses : Array[@core.Real]
}

///|
pub fn ContactPair3D::manifolds_len(self : ContactPair3D) -> Int {
  self.manifolds.length()
}

///|
pub fn ContactPair3D::manifolds(self : ContactPair3D) -> Array[ContactPoint3D] {
  self.manifolds
}

///|
pub fn ContactPair3D::set_normal_impulse_at(
  self : ContactPair3D,
  idx : Int,
  impulse : @core.Real,
) -> Unit {
  if idx >= 0 && idx < self.impulses.length() {
    self.impulses[idx] = impulse
  }
}

///|
pub fn ContactPair3D::total_impulse(self : ContactPair3D) -> @core.Vec3 {
  // Matches Rapier: sum(manifold_total_impulse * manifold_normal).
  let mut out = @core.Vec3::zero()
  let n = self.manifolds.length()
  for i in 0..<n {
    let imp = if i < self.impulses.length() { self.impulses[i] } else { 0.0F }
    out = out.add(self.manifolds[i].normal.scale(imp))
  }
  out
}

///|
pub fn ContactPair3D::total_impulse_magnitude(
  self : ContactPair3D,
) -> @core.Real {
  // Matches Rapier: sum(manifold_total_impulse) (not |total_impulse|).
  let mut out = 0.0F
  for i in 0..<self.impulses.length() {
    out = out + self.impulses[i]
  }
  out
}

///|
pub fn ContactPair3D::max_impulse(
  self : ContactPair3D,
) -> (@core.Real, @core.Vec3) {
  // Matches Rapier: strongest manifold impulse and its normal direction.
  let mut best = 0.0F
  let mut dir = @core.Vec3::zero()
  let n = self.manifolds.length()
  let m = self.impulses.length()
  let k = if n < m { n } else { m }
  for i in 0..<k {
    let imp = self.impulses[i]
    if imp > best {
      best = imp
      dir = self.manifolds[i].normal
    }
  }
  (best, dir)
}

///|
pub struct IntersectionPair3D {
  collider1 : ColliderHandle3D
  collider2 : ColliderHandle3D
  intersecting : Bool
}

///|
pub fn IntersectionPair3D::collider1(
  self : IntersectionPair3D,
) -> ColliderHandle3D {
  self.collider1
}

///|
pub fn IntersectionPair3D::collider2(
  self : IntersectionPair3D,
) -> ColliderHandle3D {
  self.collider2
}

///|
pub fn IntersectionPair3D::intersecting(self : IntersectionPair3D) -> Bool {
  self.intersecting
}

///|
fn np_pair_key(
  a : ColliderHandle3D,
  b : ColliderHandle3D,
) -> (Int, Int, Int, Int) {
  let (ai, ag) = a.into_raw_parts()
  let (bi, bg) = b.into_raw_parts()
  if ai < bi || (ai == bi && ag <= bg) {
    (ai, ag, bi, bg)
  } else {
    (bi, bg, ai, ag)
  }
}

///|
fn canonical_pair(
  a : ColliderHandle3D,
  b : ColliderHandle3D,
) -> (ColliderHandle3D, ColliderHandle3D) {
  let (ai, ag, bi, bg) = np_pair_key(a, b)
  (
    ColliderHandle3D::from_raw_parts(ai, ag),
    ColliderHandle3D::from_raw_parts(bi, bg),
  )
}

///|
pub struct NarrowPhase3D {
  contact_pairs : Array[((ColliderHandle3D, ColliderHandle3D), ContactPair3D)]
  intersection_pairs : Array[
    ((ColliderHandle3D, ColliderHandle3D), IntersectionPair3D),
  ]
}

///|
pub fn NarrowPhase3D::new() -> NarrowPhase3D {
  { contact_pairs: [], intersection_pairs: [] }
}

///|
pub fn NarrowPhase3D::clear(self : NarrowPhase3D) -> Unit {
  self.contact_pairs.clear()
  self.intersection_pairs.clear()
}

///|
pub fn NarrowPhase3D::contact_pair(
  self : NarrowPhase3D,
  a : ColliderHandle3D,
  b : ColliderHandle3D,
) -> ContactPair3D? {
  let key = canonical_pair(a, b)
  for i in 0..<self.contact_pairs.length() {
    let entry = self.contact_pairs[i]
    if entry.0.0.equals(key.0) && entry.0.1.equals(key.1) {
      return Some(entry.1)
    }
  }
  None
}

///|
/// Update the normal impulses stored for a given contact pair.
///
/// This is written by the 3D contact solver after each step so `ContactPair3D::{total_impulse,*}`
/// matches Rapier semantics.
pub fn NarrowPhase3D::set_contact_pair_normal_impulses(
  self : NarrowPhase3D,
  a : ColliderHandle3D,
  b : ColliderHandle3D,
  impulses : Array[@core.Real],
) -> Unit {
  let key = canonical_pair(a, b)
  for i in 0..<self.contact_pairs.length() {
    let entry = self.contact_pairs[i]
    if entry.0.0.equals(key.0) && entry.0.1.equals(key.1) {
      let pair = entry.1
      let n = pair.impulses.length()
      let m = impulses.length()
      let k = if n < m { n } else { m }
      for j in 0..<k {
        pair.impulses[j] = impulses[j]
      }
      for j in k..<n {
        pair.impulses[j] = 0.0F
      }
      self.contact_pairs[i] = (entry.0, pair)
      return
    }
  }
}

///|
pub fn NarrowPhase3D::intersection_pair(
  self : NarrowPhase3D,
  a : ColliderHandle3D,
  b : ColliderHandle3D,
) -> IntersectionPair3D? {
  let key = canonical_pair(a, b)
  for i in 0..<self.intersection_pairs.length() {
    let entry = self.intersection_pairs[i]
    if entry.0.0.equals(key.0) && entry.0.1.equals(key.1) {
      return Some(entry.1)
    }
  }
  None
}

///|
pub fn NarrowPhase3D::update(
  self : NarrowPhase3D,
  pairs : Array[(ColliderHandle3D, ColliderHandle3D)],
  bodies : @dynamics.RigidBodySet3D,
  colliders : ColliderSet3D,
) -> Unit {
  bodies |> ignore
  self.clear()
  for i in 0..<pairs.length() {
    let pa = pairs[i].0
    let pb = pairs[i].1
    let (a, b) = canonical_pair(pa, pb)
    if colliders.get(a) is Some(co1) && colliders.get(b) is Some(co2) {
      if !co1.enabled() || !co2.enabled() {
        continue
      }
      if !co1.collision_groups.test_groups(co2.collision_groups) {
        self.contact_pairs.push(((a, b), { manifolds: [], impulses: [] }))
        self.intersection_pairs.push(
          ((a, b), { collider1: a, collider2: b, intersecting: false }),
        )
        continue
      }

      // Keep the pair, but avoid generating contacts between colliders attached to the same body.
      if co1.parent() is Some(p1) && co2.parent() is Some(p2) {
        if p1.equals(p2) {
          self.contact_pairs.push(((a, b), { manifolds: [], impulses: [] }))
          self.intersection_pairs.push(
            ((a, b), { collider1: a, collider2: b, intersecting: false }),
          )
          continue
        }
      }
      let p1 = co1.position()
      let p2 = co2.position()
      let s1 = co1.shape()
      let s2 = co2.shape()
      let contacts : Array[ContactPoint3D] = []
      fn push_contacts_for_pair(
        p1 : @core.Isometry3,
        s1 : Shape3D,
        p2 : @core.Isometry3,
        s2 : Shape3D,
        contacts : Array[ContactPoint3D],
      ) -> Unit {
        match (s1, s2) {
          (Shape3D::Compound(parts), _) =>
            for i in 0..<parts.length() {
              let (iso, sh) = parts[i]
              push_contacts_for_pair(p1.mul(iso), sh, p2, s2, contacts)
            }
          (_, Shape3D::Compound(parts)) =>
            for i in 0..<parts.length() {
              let (iso, sh) = parts[i]
              push_contacts_for_pair(p1, s1, p2.mul(iso), sh, contacts)
            }
          _ =>
            match (s1, s2) {
              (Shape3D::HalfSpace(n), other) =>
                if compute_halfspace_contact(p1, n, p2, other) is Some(cp) {
                  contacts.push(cp)
                }
              (other, Shape3D::HalfSpace(n)) =>
                if compute_halfspace_contact(p2, n, p1, other) is Some(cp) {
                  contacts.push({
                    point1: cp.point2,
                    point2: cp.point1,
                    normal: cp.normal.scale(-1.0F),
                    penetration: cp.penetration,
                  })
                }
              (Shape3D::Ball(r1), Shape3D::Ball(r2)) =>
                if compute_ball_ball_contact(
                    p1.translation,
                    r1,
                    p2.translation,
                    r2,
                  )
                  is Some(cp) {
                  contacts.push(cp)
                }
              (Shape3D::Ball(r), Shape3D::Cuboid(he)) =>
                if compute_ball_cuboid_contact(p1.translation, r, p2, he)
                  is Some(cp) {
                  contacts.push(cp)
                }
              (Shape3D::Cuboid(he), Shape3D::Ball(r)) =>
                if compute_ball_cuboid_contact(p2.translation, r, p1, he)
                  is Some(cp) {
                  // Flip the contact so it matches the (cuboid, ball) ordering.
                  contacts.push({
                    point1: cp.point2,
                    point2: cp.point1,
                    normal: cp.normal.scale(-1.0F),
                    penetration: cp.penetration,
                  })
                }
              (Shape3D::Cuboid(he1), Shape3D::Cuboid(he2)) =>
                for cp in compute_cuboid_cuboid_contacts(p1, he1, p2, he2) {
                  contacts.push(cp)
                }
              (Shape3D::Ball(r), Shape3D::CapsuleY(cr, ch)) =>
                if compute_ball_capsule_contact(p1.translation, r, p2, cr, ch)
                  is Some(cp) {
                  contacts.push(cp)
                }
              (Shape3D::CapsuleY(cr, ch), Shape3D::Ball(r)) =>
                if compute_ball_capsule_contact(p2.translation, r, p1, cr, ch)
                  is Some(cp) {
                  contacts.push({
                    point1: cp.point2,
                    point2: cp.point1,
                    normal: cp.normal.scale(-1.0F),
                    penetration: cp.penetration,
                  })
                }
              (Shape3D::CapsuleY(r1, h1), Shape3D::CapsuleY(r2, h2)) =>
                if compute_capsule_capsule_contact(p1, r1, h1, p2, r2, h2)
                  is Some(cp) {
                  contacts.push(cp)
                }
              (Shape3D::CapsuleY(r, h), Shape3D::Cuboid(he)) =>
                if compute_capsule_cuboid_contact(p1, r, h, p2, he) is Some(cp) {
                  contacts.push(cp)
                }
              (Shape3D::Cuboid(he), Shape3D::CapsuleY(r, h)) =>
                if compute_capsule_cuboid_contact(p2, r, h, p1, he) is Some(cp) {
                  contacts.push({
                    point1: cp.point2,
                    point2: cp.point1,
                    normal: cp.normal.scale(-1.0F),
                    penetration: cp.penetration,
                  })
                }
              (Shape3D::Cylinder(cr, ch), Shape3D::Cuboid(he)) =>
                if compute_cylinder_cuboid_contact(p1, cr, ch, p2, he)
                  is Some(cp) {
                  contacts.push(cp)
                }
              (Shape3D::Cuboid(he), Shape3D::Cylinder(cr, ch)) =>
                if compute_cylinder_cuboid_contact(p2, cr, ch, p1, he)
                  is Some(cp) {
                  contacts.push({
                    point1: cp.point2,
                    point2: cp.point1,
                    normal: cp.normal.scale(-1.0F),
                    penetration: cp.penetration,
                  })
                }
              (Shape3D::RoundCylinder(cr, ch, br), Shape3D::Cuboid(he)) =>
                if compute_cylinder_cuboid_contact(p1, cr + br, ch + br, p2, he)
                  is Some(cp) {
                  contacts.push(cp)
                }
              (Shape3D::Cuboid(he), Shape3D::RoundCylinder(cr, ch, br)) =>
                if compute_cylinder_cuboid_contact(p2, cr + br, ch + br, p1, he)
                  is Some(cp) {
                  contacts.push({
                    point1: cp.point2,
                    point2: cp.point1,
                    normal: cp.normal.scale(-1.0F),
                    penetration: cp.penetration,
                  })
                }
              (Shape3D::Cylinder(r1, h1), Shape3D::Cylinder(r2, h2)) =>
                if compute_cylinder_cylinder_contact(p1, r1, h1, p2, r2, h2)
                  is Some(cp) {
                  contacts.push(cp)
                }
              (Shape3D::Cylinder(r1, h1), Shape3D::RoundCylinder(r2, h2, br2)) =>
                if compute_cylinder_cylinder_contact(
                    p1,
                    r1,
                    h1,
                    p2,
                    r2 + br2,
                    h2 + br2,
                  )
                  is Some(cp) {
                  contacts.push(cp)
                }
              (Shape3D::RoundCylinder(r1, h1, br1), Shape3D::Cylinder(r2, h2)) =>
                if compute_cylinder_cylinder_contact(
                    p1,
                    r1 + br1,
                    h1 + br1,
                    p2,
                    r2,
                    h2,
                  )
                  is Some(cp) {
                  contacts.push(cp)
                }
              (
                Shape3D::RoundCylinder(r1, h1, br1),
                Shape3D::RoundCylinder(r2, h2, br2),
              ) =>
                if compute_cylinder_cylinder_contact(
                    p1,
                    r1 + br1,
                    h1 + br1,
                    p2,
                    r2 + br2,
                    h2 + br2,
                  )
                  is Some(cp) {
                  contacts.push(cp)
                }
              (Shape3D::Cone(cr, ch), Shape3D::Cuboid(he)) =>
                if compute_cone_cuboid_contact(p1, cr, ch, p2, he) is Some(cp) {
                  contacts.push(cp)
                }
              (Shape3D::Cuboid(he), Shape3D::Cone(cr, ch)) =>
                if compute_cone_cuboid_contact(p2, cr, ch, p1, he) is Some(cp) {
                  contacts.push({
                    point1: cp.point2,
                    point2: cp.point1,
                    normal: cp.normal.scale(-1.0F),
                    penetration: cp.penetration,
                  })
                }
              (Shape3D::Cone(r1, h1), Shape3D::Cylinder(r2, h2)) =>
                if compute_cone_cylinder_contact(p1, r1, h1, p2, r2, h2)
                  is Some(cp) {
                  contacts.push(cp)
                }
              (Shape3D::Cone(r1, h1), Shape3D::RoundCylinder(r2, h2, br2)) =>
                if compute_cone_cylinder_contact(
                    p1,
                    r1,
                    h1,
                    p2,
                    r2 + br2,
                    h2 + br2,
                  )
                  is Some(cp) {
                  contacts.push(cp)
                }
              (Shape3D::Cylinder(r1, h1), Shape3D::Cone(r2, h2)) =>
                if compute_cone_cylinder_contact(p2, r2, h2, p1, r1, h1)
                  is Some(cp) {
                  contacts.push({
                    point1: cp.point2,
                    point2: cp.point1,
                    normal: cp.normal.scale(-1.0F),
                    penetration: cp.penetration,
                  })
                }
              (Shape3D::RoundCylinder(r1, h1, br1), Shape3D::Cone(r2, h2)) =>
                if compute_cone_cylinder_contact(
                    p2,
                    r2,
                    h2,
                    p1,
                    r1 + br1,
                    h1 + br1,
                  )
                  is Some(cp) {
                  contacts.push({
                    point1: cp.point2,
                    point2: cp.point1,
                    normal: cp.normal.scale(-1.0F),
                    penetration: cp.penetration,
                  })
                }
              (Shape3D::Cone(r1, h1), Shape3D::Cone(r2, h2)) =>
                if compute_cone_cone_contact(p1, r1, h1, p2, r2, h2) is Some(cp) {
                  contacts.push(cp)
                }
              (Shape3D::Ball(r), Shape3D::Cylinder(cr, ch)) =>
                if compute_ball_cylinder_contact(p1.translation, r, p2, cr, ch)
                  is Some(cp) {
                  contacts.push(cp)
                }
              (Shape3D::Cylinder(cr, ch), Shape3D::Ball(r)) =>
                if compute_ball_cylinder_contact(p2.translation, r, p1, cr, ch)
                  is Some(cp) {
                  contacts.push({
                    point1: cp.point2,
                    point2: cp.point1,
                    normal: cp.normal.scale(-1.0F),
                    penetration: cp.penetration,
                  })
                }
              (Shape3D::Ball(r), Shape3D::RoundCylinder(cr, ch, br)) =>
                if compute_ball_cylinder_contact(
                    p1.translation,
                    r,
                    p2,
                    cr + br,
                    ch + br,
                  )
                  is Some(cp) {
                  contacts.push(cp)
                }
              (Shape3D::RoundCylinder(cr, ch, br), Shape3D::Ball(r)) =>
                if compute_ball_cylinder_contact(
                    p2.translation,
                    r,
                    p1,
                    cr + br,
                    ch + br,
                  )
                  is Some(cp) {
                  contacts.push({
                    point1: cp.point2,
                    point2: cp.point1,
                    normal: cp.normal.scale(-1.0F),
                    penetration: cp.penetration,
                  })
                }
              (Shape3D::Ball(r), Shape3D::Cone(cr, ch)) =>
                if compute_ball_cone_contact(p1.translation, r, p2, cr, ch)
                  is Some(cp) {
                  contacts.push(cp)
                }
              (Shape3D::Cone(cr, ch), Shape3D::Ball(r)) =>
                if compute_ball_cone_contact(p2.translation, r, p1, cr, ch)
                  is Some(cp) {
                  contacts.push({
                    point1: cp.point2,
                    point2: cp.point1,
                    normal: cp.normal.scale(-1.0F),
                    penetration: cp.penetration,
                  })
                }
              (Shape3D::Ball(r), Shape3D::TriMesh(vertices, indices)) =>
                if compute_ball_trimesh_contact(
                    p1.translation,
                    r,
                    p2,
                    vertices,
                    indices,
                  )
                  is Some(cp) {
                  contacts.push(cp)
                }
              (Shape3D::Ball(r), Shape3D::Voxels(v)) =>
                if compute_ball_voxels_contact(p1.translation, r, p2, v)
                  is Some(cp) {
                  contacts.push(cp)
                }
              (Shape3D::Ball(r), Shape3D::Triangle(a, b, c)) => {
                let vtx = [a, b, c]
                let idx = [(0, 1, 2)]
                if compute_ball_trimesh_contact(p1.translation, r, p2, vtx, idx)
                  is Some(cp) {
                  contacts.push(cp)
                }
              }
              (
                Shape3D::Ball(r),
                Shape3D::Heightfield(vertices, _, rows, cols, flags),
              ) =>
                if compute_ball_heightfield_contact_with_flags(
                    p1.translation,
                    r,
                    p2,
                    vertices,
                    rows,
                    cols,
                    flags,
                  )
                  is Some(cp) {
                  contacts.push(cp)
                }
              (Shape3D::TriMesh(vertices, indices), Shape3D::Ball(r)) =>
                if compute_ball_trimesh_contact(
                    p2.translation,
                    r,
                    p1,
                    vertices,
                    indices,
                  )
                  is Some(cp) {
                  contacts.push({
                    point1: cp.point2,
                    point2: cp.point1,
                    normal: cp.normal.scale(-1.0F),
                    penetration: cp.penetration,
                  })
                }
              (Shape3D::Voxels(v), Shape3D::Ball(r)) =>
                if compute_ball_voxels_contact(p2.translation, r, p1, v)
                  is Some(cp) {
                  contacts.push({
                    point1: cp.point2,
                    point2: cp.point1,
                    normal: cp.normal.scale(-1.0F),
                    penetration: cp.penetration,
                  })
                }
              (Shape3D::Triangle(a, b, c), Shape3D::Ball(r)) => {
                let vtx = [a, b, c]
                let idx = [(0, 1, 2)]
                if compute_ball_trimesh_contact(p2.translation, r, p1, vtx, idx)
                  is Some(cp) {
                  contacts.push({
                    point1: cp.point2,
                    point2: cp.point1,
                    normal: cp.normal.scale(-1.0F),
                    penetration: cp.penetration,
                  })
                }
              }
              (
                Shape3D::Heightfield(vertices, _, rows, cols, flags),
                Shape3D::Ball(r),
              ) =>
                if compute_ball_heightfield_contact_with_flags(
                    p2.translation,
                    r,
                    p1,
                    vertices,
                    rows,
                    cols,
                    flags,
                  )
                  is Some(cp) {
                  contacts.push({
                    point1: cp.point2,
                    point2: cp.point1,
                    normal: cp.normal.scale(-1.0F),
                    penetration: cp.penetration,
                  })
                }
              (Shape3D::Cuboid(he), Shape3D::TriMesh(vertices, indices)) =>
                if compute_cuboid_trimesh_contact(p1, he, p2, vertices, indices)
                  is Some(cp) {
                  contacts.push(cp)
                }
              (Shape3D::Cuboid(he), Shape3D::Voxels(v)) =>
                if compute_cuboid_voxels_contact(p1, he, p2, v) is Some(cp) {
                  contacts.push(cp)
                }
              (Shape3D::Cuboid(he), Shape3D::Triangle(a, b, c)) => {
                let vtx = [a, b, c]
                let idx = [(0, 1, 2)]
                if compute_cuboid_trimesh_contact(p1, he, p2, vtx, idx)
                  is Some(cp) {
                  contacts.push(cp)
                }
              }
              (
                Shape3D::Cuboid(he),
                Shape3D::Heightfield(vertices, _, rows, cols, flags),
              ) =>
                if compute_cuboid_heightfield_contact_with_flags(
                    p1, he, p2, vertices, rows, cols, flags,
                  )
                  is Some(cp) {
                  contacts.push(cp)
                }
              (Shape3D::TriMesh(vertices, indices), Shape3D::Cuboid(he)) =>
                if compute_cuboid_trimesh_contact(p2, he, p1, vertices, indices)
                  is Some(cp) {
                  contacts.push({
                    point1: cp.point2,
                    point2: cp.point1,
                    normal: cp.normal.scale(-1.0F),
                    penetration: cp.penetration,
                  })
                }
              (Shape3D::Voxels(v), Shape3D::Cuboid(he)) =>
                if compute_cuboid_voxels_contact(p2, he, p1, v) is Some(cp) {
                  contacts.push({
                    point1: cp.point2,
                    point2: cp.point1,
                    normal: cp.normal.scale(-1.0F),
                    penetration: cp.penetration,
                  })
                }
              (Shape3D::Triangle(a, b, c), Shape3D::Cuboid(he)) => {
                let vtx = [a, b, c]
                let idx = [(0, 1, 2)]
                if compute_cuboid_trimesh_contact(p2, he, p1, vtx, idx)
                  is Some(cp) {
                  contacts.push({
                    point1: cp.point2,
                    point2: cp.point1,
                    normal: cp.normal.scale(-1.0F),
                    penetration: cp.penetration,
                  })
                }
              }
              (
                Shape3D::Heightfield(vertices, _, rows, cols, flags),
                Shape3D::Cuboid(he),
              ) =>
                if compute_cuboid_heightfield_contact_with_flags(
                    p2, he, p1, vertices, rows, cols, flags,
                  )
                  is Some(cp) {
                  contacts.push({
                    point1: cp.point2,
                    point2: cp.point1,
                    normal: cp.normal.scale(-1.0F),
                    penetration: cp.penetration,
                  })
                }
              (Shape3D::CapsuleY(r, hh), Shape3D::TriMesh(vertices, indices)) =>
                if compute_capsule_trimesh_contact(
                    p1, r, hh, p2, vertices, indices,
                  )
                  is Some(cp) {
                  contacts.push(cp)
                }
              (Shape3D::CapsuleY(r, hh), Shape3D::Voxels(v)) =>
                if compute_capsule_voxels_contact(p1, r, hh, p2, v) is Some(cp) {
                  contacts.push(cp)
                }
              (Shape3D::CapsuleY(r, hh), Shape3D::Triangle(a, b, c)) => {
                let vtx = [a, b, c]
                let idx = [(0, 1, 2)]
                if compute_capsule_trimesh_contact(p1, r, hh, p2, vtx, idx)
                  is Some(cp) {
                  contacts.push(cp)
                }
              }
              (
                Shape3D::CapsuleY(r, hh),
                Shape3D::Heightfield(vertices, _, rows, cols, flags),
              ) =>
                if compute_capsule_heightfield_contact_with_flags(
                    p1, r, hh, p2, vertices, rows, cols, flags,
                  )
                  is Some(cp) {
                  contacts.push(cp)
                }
              (Shape3D::TriMesh(vertices, indices), Shape3D::CapsuleY(r, hh)) =>
                if compute_capsule_trimesh_contact(
                    p2, r, hh, p1, vertices, indices,
                  )
                  is Some(cp) {
                  contacts.push({
                    point1: cp.point2,
                    point2: cp.point1,
                    normal: cp.normal.scale(-1.0F),
                    penetration: cp.penetration,
                  })
                }
              (Shape3D::Voxels(v), Shape3D::CapsuleY(r, hh)) =>
                if compute_capsule_voxels_contact(p2, r, hh, p1, v) is Some(cp) {
                  contacts.push({
                    point1: cp.point2,
                    point2: cp.point1,
                    normal: cp.normal.scale(-1.0F),
                    penetration: cp.penetration,
                  })
                }
              (Shape3D::Triangle(a, b, c), Shape3D::CapsuleY(r, hh)) => {
                let vtx = [a, b, c]
                let idx = [(0, 1, 2)]
                if compute_capsule_trimesh_contact(p2, r, hh, p1, vtx, idx)
                  is Some(cp) {
                  contacts.push({
                    point1: cp.point2,
                    point2: cp.point1,
                    normal: cp.normal.scale(-1.0F),
                    penetration: cp.penetration,
                  })
                }
              }
              (
                Shape3D::Heightfield(vertices, _, rows, cols, flags),
                Shape3D::CapsuleY(r, hh),
              ) =>
                if compute_capsule_heightfield_contact_with_flags(
                    p2, r, hh, p1, vertices, rows, cols, flags,
                  )
                  is Some(cp) {
                  contacts.push({
                    point1: cp.point2,
                    point2: cp.point1,
                    normal: cp.normal.scale(-1.0F),
                    penetration: cp.penetration,
                  })
                }
              (Shape3D::Cylinder(r, hh), Shape3D::TriMesh(vertices, indices)) =>
                if compute_cylinder_trimesh_contact(
                    p1, r, hh, p2, vertices, indices,
                  )
                  is Some(cp) {
                  contacts.push(cp)
                }
              (Shape3D::Cylinder(r, hh), Shape3D::Voxels(v)) =>
                if compute_cylinder_voxels_contact(p1, r, hh, p2, v) is Some(cp) {
                  contacts.push(cp)
                }
              (Shape3D::Cylinder(r, hh), Shape3D::Triangle(a, b, c)) => {
                let vtx = [a, b, c]
                let idx = [(0, 1, 2)]
                if compute_cylinder_trimesh_contact(p1, r, hh, p2, vtx, idx)
                  is Some(cp) {
                  contacts.push(cp)
                }
              }
              (
                Shape3D::Cylinder(r, hh),
                Shape3D::Heightfield(vertices, _, rows, cols, flags),
              ) =>
                if compute_cylinder_heightfield_contact_with_flags(
                    p1, r, hh, p2, vertices, rows, cols, flags,
                  )
                  is Some(cp) {
                  contacts.push(cp)
                }
              (Shape3D::TriMesh(vertices, indices), Shape3D::Cylinder(r, hh)) =>
                if compute_cylinder_trimesh_contact(
                    p2, r, hh, p1, vertices, indices,
                  )
                  is Some(cp) {
                  contacts.push({
                    point1: cp.point2,
                    point2: cp.point1,
                    normal: cp.normal.scale(-1.0F),
                    penetration: cp.penetration,
                  })
                }
              (Shape3D::Voxels(v), Shape3D::Cylinder(r, hh)) =>
                if compute_cylinder_voxels_contact(p2, r, hh, p1, v) is Some(cp) {
                  contacts.push({
                    point1: cp.point2,
                    point2: cp.point1,
                    normal: cp.normal.scale(-1.0F),
                    penetration: cp.penetration,
                  })
                }
              (Shape3D::Triangle(a, b, c), Shape3D::Cylinder(r, hh)) => {
                let vtx = [a, b, c]
                let idx = [(0, 1, 2)]
                if compute_cylinder_trimesh_contact(p2, r, hh, p1, vtx, idx)
                  is Some(cp) {
                  contacts.push({
                    point1: cp.point2,
                    point2: cp.point1,
                    normal: cp.normal.scale(-1.0F),
                    penetration: cp.penetration,
                  })
                }
              }
              (
                Shape3D::Heightfield(vertices, _, rows, cols, flags),
                Shape3D::Cylinder(r, hh),
              ) =>
                if compute_cylinder_heightfield_contact_with_flags(
                    p2, r, hh, p1, vertices, rows, cols, flags,
                  )
                  is Some(cp) {
                  contacts.push({
                    point1: cp.point2,
                    point2: cp.point1,
                    normal: cp.normal.scale(-1.0F),
                    penetration: cp.penetration,
                  })
                }
              (
                Shape3D::RoundCylinder(r, hh, br),
                Shape3D::TriMesh(vertices, indices),
              ) =>
                if compute_round_cylinder_trimesh_contact(
                    p1, r, hh, br, p2, vertices, indices,
                  )
                  is Some(cp) {
                  contacts.push(cp)
                }
              (Shape3D::RoundCylinder(r, hh, br), Shape3D::Voxels(v)) =>
                if compute_cylinder_voxels_contact(p1, r + br, hh + br, p2, v)
                  is Some(cp) {
                  contacts.push(cp)
                }
              (Shape3D::RoundCylinder(r, hh, br), Shape3D::Triangle(a, b, c)) => {
                let vtx = [a, b, c]
                let idx = [(0, 1, 2)]
                if compute_round_cylinder_trimesh_contact(
                    p1, r, hh, br, p2, vtx, idx,
                  )
                  is Some(cp) {
                  contacts.push(cp)
                }
              }
              (
                Shape3D::RoundCylinder(r, hh, br),
                Shape3D::Heightfield(vertices, _, rows, cols, flags),
              ) =>
                if compute_round_cylinder_heightfield_contact_with_flags(
                    p1, r, hh, br, p2, vertices, rows, cols, flags,
                  )
                  is Some(cp) {
                  contacts.push(cp)
                }
              (
                Shape3D::TriMesh(vertices, indices),
                Shape3D::RoundCylinder(r, hh, br),
              ) =>
                if compute_round_cylinder_trimesh_contact(
                    p2, r, hh, br, p1, vertices, indices,
                  )
                  is Some(cp) {
                  contacts.push({
                    point1: cp.point2,
                    point2: cp.point1,
                    normal: cp.normal.scale(-1.0F),
                    penetration: cp.penetration,
                  })
                }
              (Shape3D::Voxels(v), Shape3D::RoundCylinder(r, hh, br)) =>
                if compute_cylinder_voxels_contact(p2, r + br, hh + br, p1, v)
                  is Some(cp) {
                  contacts.push({
                    point1: cp.point2,
                    point2: cp.point1,
                    normal: cp.normal.scale(-1.0F),
                    penetration: cp.penetration,
                  })
                }
              (Shape3D::Triangle(a, b, c), Shape3D::RoundCylinder(r, hh, br)) => {
                let vtx = [a, b, c]
                let idx = [(0, 1, 2)]
                if compute_round_cylinder_trimesh_contact(
                    p2, r, hh, br, p1, vtx, idx,
                  )
                  is Some(cp) {
                  contacts.push({
                    point1: cp.point2,
                    point2: cp.point1,
                    normal: cp.normal.scale(-1.0F),
                    penetration: cp.penetration,
                  })
                }
              }
              (
                Shape3D::Heightfield(vertices, _, rows, cols, flags),
                Shape3D::RoundCylinder(r, hh, br),
              ) =>
                if compute_round_cylinder_heightfield_contact_with_flags(
                    p2, r, hh, br, p1, vertices, rows, cols, flags,
                  )
                  is Some(cp) {
                  contacts.push({
                    point1: cp.point2,
                    point2: cp.point1,
                    normal: cp.normal.scale(-1.0F),
                    penetration: cp.penetration,
                  })
                }
              (Shape3D::Cone(r, hh), Shape3D::TriMesh(vertices, indices)) =>
                if compute_cone_trimesh_contact(
                    p1, r, hh, p2, vertices, indices,
                  )
                  is Some(cp) {
                  contacts.push(cp)
                }
              (Shape3D::Cone(r, hh), Shape3D::Voxels(v)) =>
                if compute_cone_voxels_contact(p1, r, hh, p2, v) is Some(cp) {
                  contacts.push(cp)
                }
              (Shape3D::Cone(r, hh), Shape3D::Triangle(a, b, c)) => {
                let vtx = [a, b, c]
                let idx = [(0, 1, 2)]
                if compute_cone_trimesh_contact(p1, r, hh, p2, vtx, idx)
                  is Some(cp) {
                  contacts.push(cp)
                }
              }
              (
                Shape3D::Cone(r, hh),
                Shape3D::Heightfield(vertices, _, rows, cols, flags),
              ) =>
                if compute_cone_heightfield_contact_with_flags(
                    p1, r, hh, p2, vertices, rows, cols, flags,
                  )
                  is Some(cp) {
                  contacts.push(cp)
                }
              (Shape3D::TriMesh(vertices, indices), Shape3D::Cone(r, hh)) =>
                if compute_cone_trimesh_contact(
                    p2, r, hh, p1, vertices, indices,
                  )
                  is Some(cp) {
                  contacts.push({
                    point1: cp.point2,
                    point2: cp.point1,
                    normal: cp.normal.scale(-1.0F),
                    penetration: cp.penetration,
                  })
                }
              (Shape3D::Voxels(v), Shape3D::Cone(r, hh)) =>
                if compute_cone_voxels_contact(p2, r, hh, p1, v) is Some(cp) {
                  contacts.push({
                    point1: cp.point2,
                    point2: cp.point1,
                    normal: cp.normal.scale(-1.0F),
                    penetration: cp.penetration,
                  })
                }
              (Shape3D::Triangle(a, b, c), Shape3D::Cone(r, hh)) => {
                let vtx = [a, b, c]
                let idx = [(0, 1, 2)]
                if compute_cone_trimesh_contact(p2, r, hh, p1, vtx, idx)
                  is Some(cp) {
                  contacts.push({
                    point1: cp.point2,
                    point2: cp.point1,
                    normal: cp.normal.scale(-1.0F),
                    penetration: cp.penetration,
                  })
                }
              }
              (
                Shape3D::Heightfield(vertices, _, rows, cols, flags),
                Shape3D::Cone(r, hh),
              ) =>
                if compute_cone_heightfield_contact_with_flags(
                    p2, r, hh, p1, vertices, rows, cols, flags,
                  )
                  is Some(cp) {
                  contacts.push({
                    point1: cp.point2,
                    point2: cp.point1,
                    normal: cp.normal.scale(-1.0F),
                    penetration: cp.penetration,
                  })
                }
              _ =>
                if compute_convex_contact(p1, s1, p2, s2) is Some(cp) {
                  contacts.push(cp)
                }
            }
        }
      }

      push_contacts_for_pair(p1, s1, p2, s2, contacts)
      // Keep up to 4 contact points per pair (a lightweight manifold).
      if contacts.length() > 4 {
        // Sort by penetration descending (insertion sort for small N).
        for i in 1..<contacts.length() {
          let key = contacts[i]
          let mut j = i - 1
          while j >= 0 && contacts[j].penetration < key.penetration {
            contacts[j + 1] = contacts[j]
            j = j - 1
          }
          contacts[j + 1] = key
        }
        let unique_contacts : Array[ContactPoint3D] = []
        let eps2 = 1.0e-8F
        for c in contacts {
          let mut dup = false
          for k in 0..<unique_contacts.length() {
            if unique_contacts[k].point1.sub(c.point1).length_squared() <= eps2 {
              dup = true
              break
            }
          }
          if !dup {
            unique_contacts.push(c)
            if unique_contacts.length() >= 4 {
              break
            }
          }
        }
        contacts.clear()
        for c in unique_contacts {
          contacts.push(c)
        }
      }
      let impulses : Array[@core.Real] = []
      for _ in 0..<contacts.length() {
        impulses.push(0.0F)
      }
      self.contact_pairs.push(((a, b), { manifolds: contacts, impulses }))
      self.intersection_pairs.push(
        (
          (a, b),
          { collider1: a, collider2: b, intersecting: contacts.length() > 0 },
        ),
      )
    }
  }
}

///|
pub fn NarrowPhase3D::all_contact_pairs(
  self : NarrowPhase3D,
) -> Array[((ColliderHandle3D, ColliderHandle3D), ContactPair3D)] {
  self.contact_pairs
}

///|
pub fn NarrowPhase3D::all_intersection_pairs(
  self : NarrowPhase3D,
) -> Array[((ColliderHandle3D, ColliderHandle3D), IntersectionPair3D)] {
  self.intersection_pairs
}
