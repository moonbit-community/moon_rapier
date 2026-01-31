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
pub(all) enum CollisionEvent3D {
  Started(ColliderHandle3D, ColliderHandle3D, CollisionEventFlags)
  Stopped(ColliderHandle3D, ColliderHandle3D, CollisionEventFlags)
}

///|
pub fn CollisionEvent3D::started(self : CollisionEvent3D) -> Bool {
  match self {
    CollisionEvent3D::Started(_, _, _) => true
    _ => false
  }
}

///|
pub fn CollisionEvent3D::stopped(self : CollisionEvent3D) -> Bool {
  match self {
    CollisionEvent3D::Stopped(_, _, _) => true
    _ => false
  }
}

///|
pub fn CollisionEvent3D::collider1(self : CollisionEvent3D) -> ColliderHandle3D {
  match self {
    CollisionEvent3D::Started(h, _, _) => h
    CollisionEvent3D::Stopped(h, _, _) => h
  }
}

///|
pub fn CollisionEvent3D::collider2(self : CollisionEvent3D) -> ColliderHandle3D {
  match self {
    CollisionEvent3D::Started(_, h, _) => h
    CollisionEvent3D::Stopped(_, h, _) => h
  }
}

///|
pub fn CollisionEvent3D::sensor(self : CollisionEvent3D) -> Bool {
  match self {
    CollisionEvent3D::Started(_, _, flags) =>
      flags.contains(CollisionEventFlags::sensor())
    CollisionEvent3D::Stopped(_, _, flags) =>
      flags.contains(CollisionEventFlags::sensor())
  }
}

///|
pub fn CollisionEvent3D::removed(self : CollisionEvent3D) -> Bool {
  match self {
    CollisionEvent3D::Started(_, _, flags) =>
      flags.contains(CollisionEventFlags::removed())
    CollisionEvent3D::Stopped(_, _, flags) =>
      flags.contains(CollisionEventFlags::removed())
  }
}
