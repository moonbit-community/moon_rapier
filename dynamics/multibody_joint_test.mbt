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
const TEST_SPATIAL_DIM : Int = 3

///|
fn mb_lcg_next(seed : Int) -> Int {
  let modulus = 65536
  let next = (seed * 25173 + 13849) % modulus
  if next < 0 {
    next + modulus
  } else {
    next
  }
}

///|
fn mb_rand_range(seed : Int, max_exclusive : Int) -> (Int, Int) {
  let next = mb_lcg_next(seed)
  if max_exclusive <= 0 {
    (next, 0)
  } else {
    (next, next % max_exclusive)
  }
}

///|
fn reverse_int_array(values : Array[Int]) -> Unit {
  let len = values.length()
  for index in 0..<(len / 2) {
    let swap_index = len - 1 - index
    let temp = values[index]
    values[index] = values[swap_index]
    values[swap_index] = temp
  }
}

///|
fn shuffle_int_array(values : Array[Int], seed : Int) -> Int {
  let mut current_seed = seed
  let len = values.length()
  for index in 0..<len {
    let (next_seed, swap_index) = mb_rand_range(current_seed, len)
    current_seed = next_seed
    let temp = values[index]
    values[index] = values[swap_index]
    values[swap_index] = temp
  }
  current_seed
}

///|
fn reverse_handle_array(values : Array[RigidBodyHandle]) -> Unit {
  let len = values.length()
  for index in 0..<(len / 2) {
    let swap_index = len - 1 - index
    let temp = values[index]
    values[index] = values[swap_index]
    values[swap_index] = temp
  }
}

///|
fn shuffle_handle_array(values : Array[RigidBodyHandle], seed : Int) -> Int {
  let mut current_seed = seed
  let len = values.length()
  for index in 0..<len {
    let (next_seed, swap_index) = mb_rand_range(current_seed, len)
    current_seed = next_seed
    let temp = values[index]
    values[index] = values[swap_index]
    values[swap_index] = temp
  }
  current_seed
}

///|
/// Ported from rapier/src/dynamics/joint/multibody_joint/multibody.rs (test_multibody_append).
test "multibody append" {
  let spatial_dim : Int = TEST_SPATIAL_DIM
  let bodies : RigidBodySet = RigidBodySet::new()
  let joints : MultibodyJointSet = MultibodyJointSet::new()
  let a : RigidBodyHandle = bodies.insert(RigidBodyBuilder::dynamic().build())
  let b : RigidBodyHandle = bodies.insert(RigidBodyBuilder::dynamic().build())
  let c : RigidBodyHandle = bodies.insert(RigidBodyBuilder::dynamic().build())
  let d : RigidBodyHandle = bodies.insert(RigidBodyBuilder::dynamic().build())
  let joint = RevoluteJointBuilder::new().build()
  let mb_handle = joints.insert(a, b, joint, true)
  joints.insert(c, d, joint, true) |> ignore
  joints.insert(b, c, joint, true) |> ignore
  if mb_handle is Some(handle) {
    if joints.get(handle) is Some(pair) {
      let (multibody, _) = pair
      inspect(multibody.ndofs() == spatial_dim + 3, content="true")
    }
  }
}

///|
/// Ported from rapier/src/dynamics/joint/multibody_joint/multibody.rs (test_multibody_insert).
test "multibody insert" {
  let spatial_dim : Int = TEST_SPATIAL_DIM
  let mut seed = 1234
  let num_links = 100
  for scenario in 0..<10 {
    let bodies : RigidBodySet = RigidBodySet::new()
    let multibody_joints : MultibodyJointSet = MultibodyJointSet::new()
    let handles : Array[RigidBodyHandle] = []
    for _ in 0..<num_links {
      handles.push(bodies.insert(RigidBodyBuilder::dynamic().build()))
    }
    let insertion_ids : Array[Int] = []
    for index in 0..<(num_links - 1) {
      insertion_ids.push(index)
    }
    if scenario == 1 {
      reverse_int_array(insertion_ids)
    } else if scenario > 1 {
      seed = shuffle_int_array(insertion_ids, seed)
    }
    let joint = RevoluteJointBuilder::new().build()
    let mut mb_handle : MultibodyJointHandle = MultibodyJointHandle::invalid()
    for index in insertion_ids {
      let inserted = multibody_joints.insert(
        handles[index],
        handles[index + 1],
        joint,
        true,
      )
      if inserted is Some(handle) {
        mb_handle = handle
      }
    }
    if multibody_joints.get(mb_handle) is Some(pair) {
      let (multibody, _) = pair
      inspect(multibody.ndofs() == spatial_dim + num_links - 1, content="true")
    }
  }
}

///|
/// Ported from rapier/src/dynamics/joint/multibody_joint/multibody.rs (test_multibody_remove).
test "multibody remove" {
  let mut seed = 1234
  let num_links = 100
  for scenario in 0..<10 {
    let bodies : RigidBodySet = RigidBodySet::new()
    let multibody_joints : MultibodyJointSet = MultibodyJointSet::new()
    let colliders = @collision.ColliderSet::new()
    let impulse_joints = ImpulseJointSet::new()
    let islands = IslandManager::new()
    let handles : Array[RigidBodyHandle] = []
    for _ in 0..<num_links {
      handles.push(bodies.insert(RigidBodyBuilder::dynamic().build()))
    }
    let joint = RevoluteJointBuilder::new().build()
    for index in 0..<(num_links - 1) {
      multibody_joints.insert(handles[index], handles[index + 1], joint, true)
      |> ignore
    }
    if scenario == 1 {
      reverse_handle_array(handles)
    } else if scenario > 1 {
      seed = shuffle_handle_array(handles, seed)
    }
    for handle in handles {
      bodies.remove(handle, islands, (), impulse_joints, multibody_joints, true)
      |> ignore
    }
    colliders |> ignore
  }
}

///|
/// Ported from rapier/src/dynamics/joint/multibody_joint/multibody_joint_set.rs behavior.
test "multibody remove wakes links" {
  let bodies : RigidBodySet = RigidBodySet::new()
  let multibody_joints : MultibodyJointSet = MultibodyJointSet::new()
  let islands = IslandManager::new()
  let impulse_joints = ImpulseJointSet::new()
  let joint = RevoluteJointBuilder::new().build()
  let handle1 = bodies.insert(RigidBodyBuilder::dynamic().build())
  let handle2 = bodies.insert(RigidBodyBuilder::dynamic().build())
  let handle3 = bodies.insert(RigidBodyBuilder::dynamic().build())
  let joint1 = multibody_joints.insert(handle1, handle2, joint, false)
  let joint2 = multibody_joints.insert(handle2, handle3, joint, false)
  bodies.remove(handle2, islands, (), impulse_joints, multibody_joints, true)
  |> ignore
  let wake_handles = multibody_joints.take_wake_up()
  inspect(wake_handles.length() == 3, content="true")
  if joint1 is Some(joint_handle1) {
    inspect(multibody_joints.get(joint_handle1) is None, content="true")
  } else {
    inspect(false, content="false")
  }
  if joint2 is Some(joint_handle2) {
    inspect(multibody_joints.get(joint_handle2) is None, content="true")
  } else {
    inspect(false, content="false")
  }
}

///|
/// Ported from rapier/src/dynamics/joint/multibody_joint/multibody.rs (index_sequence_rearrange_columns).
test "index sequence rearrange columns" {
  let seq : IndexSequence = IndexSequence::new()
  seq.remove(2)
  seq.remove(3)
  seq.remove(4)
  seq.keep(5)
  seq.keep(6)
  seq.remove(7)
  seq.keep(8)
  let vec = RowDVector::from_array([
    0.0F, 1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F, 7.0F, 8.0F, 9.0F,
  ])
  seq.rearrange_columns(vec, true)
  let expected = RowDVector::from_array([
    0.0F, 1.0F, 5.0F, 6.0F, 8.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
  ])
  inspect(vec.equals(expected), content="true")
}

///|
/// Ported from rapier/src/dynamics/joint/multibody_joint/multibody.rs (index_sequence_rearrange_rows).
test "index sequence rearrange rows" {
  let seq : IndexSequence = IndexSequence::new()
  seq.remove(2)
  seq.remove(3)
  seq.remove(4)
  seq.keep(5)
  seq.keep(6)
  seq.remove(7)
  seq.keep(8)
  let vec = DVector::from_array([
    0.0F, 1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F, 7.0F, 8.0F, 9.0F,
  ])
  seq.rearrange_rows(vec, true)
  let expected = DVector::from_array([
    0.0F, 1.0F, 5.0F, 6.0F, 8.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
  ])
  inspect(vec.equals(expected), content="true")
  seq.inv_rearrange_rows(vec)
  let expected2 = DVector::from_array([
    0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 5.0F, 6.0F, 0.0F, 8.0F, 0.0F,
  ])
  inspect(vec.equals(expected2), content="true")
}

///|
/// Ported from rapier/src/dynamics/joint/multibody_joint/multibody.rs (index_sequence_with_rearranged_rows_mut).
test "index sequence with rearranged rows mut" {
  let seq : IndexSequence = IndexSequence::new()
  seq.remove(2)
  seq.remove(3)
  seq.remove(4)
  seq.keep(5)
  seq.keep(6)
  seq.remove(7)
  seq.keep(8)
  let vec = DVector::from_array([
    0.0F, 1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F, 7.0F, 8.0F, 9.0F,
  ])
  let mut rearranged_len = 0
  let mut rearranged_equals = false
  seq.with_rearranged_rows_mut(vec, fn(v) {
    rearranged_len = v.len()
    let expected = DVector::from_array([0.0F, 1.0F, 5.0F, 6.0F, 8.0F])
    rearranged_equals = v.equals(expected)
    v.scale_in_place(10.0F)
  })
  inspect(rearranged_len == 5, content="true")
  inspect(rearranged_equals, content="true")
  let expected = DVector::from_array([
    0.0F, 10.0F, 0.0F, 0.0F, 0.0F, 50.0F, 60.0F, 0.0F, 80.0F, 0.0F,
  ])
  inspect(vec.equals(expected), content="true")
}
