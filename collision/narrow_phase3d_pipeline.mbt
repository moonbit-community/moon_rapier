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
}

///|
pub fn NarrowPhase3D::new() -> NarrowPhase3D {
  { contact_pairs: [] }
}

///|
pub fn NarrowPhase3D::clear(self : NarrowPhase3D) -> Unit {
  self.contact_pairs.clear()
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
pub fn NarrowPhase3D::intersection_pair(
  self : NarrowPhase3D,
  a : ColliderHandle3D,
  b : ColliderHandle3D,
) -> Unit? {
  self |> ignore
  a |> ignore
  b |> ignore
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
        self.contact_pairs.push(((a, b), { manifolds: [] }))
        continue
      }

      // Keep the pair, but avoid generating contacts between colliders attached to the same body.
      if co1.parent() is Some(p1) && co2.parent() is Some(p2) {
        if p1.equals(p2) {
          self.contact_pairs.push(((a, b), { manifolds: [] }))
          continue
        }
      }
      let p1 = co1.position()
      let p2 = co2.position()
      let s1 = co1.shape()
      let s2 = co2.shape()
      let contacts : Array[ContactPoint3D] = []
      match (s1, s2) {
        (Shape3D::Ball(r1), Shape3D::Ball(r2)) =>
          if compute_ball_ball_contact(p1.translation, r1, p2.translation, r2)
            is Some(cp) {
            contacts.push(cp)
          }
        (Shape3D::Ball(r), Shape3D::Cuboid(he)) =>
          if compute_ball_cuboid_contact(p1.translation, r, p2, he) is Some(cp) {
            contacts.push(cp)
          }
        (Shape3D::Cuboid(he), Shape3D::Ball(r)) =>
          if compute_ball_cuboid_contact(p2.translation, r, p1, he) is Some(cp) {
            // Flip the contact so it matches the (cuboid, ball) ordering.
            contacts.push({
              point1: cp.point2,
              point2: cp.point1,
              normal: cp.normal.scale(-1.0F),
              penetration: cp.penetration,
            })
          }
        (Shape3D::Cuboid(he1), Shape3D::Cuboid(he2)) =>
          if compute_cuboid_cuboid_contact(p1, he1, p2, he2) is Some(cp) {
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
          if compute_capsule_capsule_contact(p1, r1, h1, p2, r2, h2) is Some(cp) {
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
          if compute_cylinder_cuboid_contact(p1, cr, ch, p2, he) is Some(cp) {
            contacts.push(cp)
          }
        (Shape3D::Cuboid(he), Shape3D::Cylinder(cr, ch)) =>
          if compute_cylinder_cuboid_contact(p2, cr, ch, p1, he) is Some(cp) {
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
          if compute_cone_cylinder_contact(p1, r1, h1, p2, r2, h2) is Some(cp) {
            contacts.push(cp)
          }
        (Shape3D::Cone(r1, h1), Shape3D::RoundCylinder(r2, h2, br2)) =>
          if compute_cone_cylinder_contact(p1, r1, h1, p2, r2 + br2, h2 + br2)
            is Some(cp) {
            contacts.push(cp)
          }
        (Shape3D::Cylinder(r1, h1), Shape3D::Cone(r2, h2)) =>
          if compute_cone_cylinder_contact(p2, r2, h2, p1, r1, h1) is Some(cp) {
            contacts.push({
              point1: cp.point2,
              point2: cp.point1,
              normal: cp.normal.scale(-1.0F),
              penetration: cp.penetration,
            })
          }
        (Shape3D::RoundCylinder(r1, h1, br1), Shape3D::Cone(r2, h2)) =>
          if compute_cone_cylinder_contact(p2, r2, h2, p1, r1 + br1, h1 + br1)
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
        (Shape3D::Cuboid(he), Shape3D::TriMesh(vertices, indices)) =>
          if compute_cuboid_trimesh_contact(p1, he, p2, vertices, indices)
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
        (Shape3D::CapsuleY(r, hh), Shape3D::TriMesh(vertices, indices)) =>
          if compute_capsule_trimesh_contact(p1, r, hh, p2, vertices, indices)
            is Some(cp) {
            contacts.push(cp)
          }
        (Shape3D::TriMesh(vertices, indices), Shape3D::CapsuleY(r, hh)) =>
          if compute_capsule_trimesh_contact(p2, r, hh, p1, vertices, indices)
            is Some(cp) {
            contacts.push({
              point1: cp.point2,
              point2: cp.point1,
              normal: cp.normal.scale(-1.0F),
              penetration: cp.penetration,
            })
          }
        (Shape3D::Cylinder(r, hh), Shape3D::TriMesh(vertices, indices)) =>
          if compute_cylinder_trimesh_contact(p1, r, hh, p2, vertices, indices)
            is Some(cp) {
            contacts.push(cp)
          }
        (Shape3D::TriMesh(vertices, indices), Shape3D::Cylinder(r, hh)) =>
          if compute_cylinder_trimesh_contact(p2, r, hh, p1, vertices, indices)
            is Some(cp) {
            contacts.push({
              point1: cp.point2,
              point2: cp.point1,
              normal: cp.normal.scale(-1.0F),
              penetration: cp.penetration,
            })
          }
        (Shape3D::RoundCylinder(r, hh, br), Shape3D::TriMesh(vertices, indices)) =>
          if compute_round_cylinder_trimesh_contact(
              p1, r, hh, br, p2, vertices, indices,
            )
            is Some(cp) {
            contacts.push(cp)
          }
        (Shape3D::TriMesh(vertices, indices), Shape3D::RoundCylinder(r, hh, br)) =>
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
        (Shape3D::Cone(r, hh), Shape3D::TriMesh(vertices, indices)) =>
          if compute_cone_trimesh_contact(p1, r, hh, p2, vertices, indices)
            is Some(cp) {
            contacts.push(cp)
          }
        (Shape3D::TriMesh(vertices, indices), Shape3D::Cone(r, hh)) =>
          if compute_cone_trimesh_contact(p2, r, hh, p1, vertices, indices)
            is Some(cp) {
            contacts.push({
              point1: cp.point2,
              point2: cp.point1,
              normal: cp.normal.scale(-1.0F),
              penetration: cp.penetration,
            })
          }
        _ => ()
      }
      self.contact_pairs.push(((a, b), { manifolds: contacts }))
    }
  }
}

///|
pub fn NarrowPhase3D::all_contact_pairs(
  self : NarrowPhase3D,
) -> Array[((ColliderHandle3D, ColliderHandle3D), ContactPair3D)] {
  self.contact_pairs
}
