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
pub struct ModifiedObjects[Handle, Object] {
  handles : Array[Handle]
  object_marker : Object?
}

///|
pub fn[Handle, Object] ModifiedObjects::new() -> ModifiedObjects[Handle, Object] {
  { handles: [], object_marker: None }
}

///|
pub fn[Handle, Object] ModifiedObjects::with_capacity(
  capacity : Int,
) -> ModifiedObjects[Handle, Object] {
  let result : ModifiedObjects[Handle, Object] = {
    handles: [],
    object_marker: None,
  }
  result.handles.reserve_capacity(capacity)
  result
}

///|
pub fn[Handle, Object] ModifiedObjects::clear(
  self : ModifiedObjects[Handle, Object],
) -> Unit {
  self.handles.clear()
}

///|
pub fn[Handle, Object : HasModifiedFlag] ModifiedObjects::push_once(
  self : ModifiedObjects[Handle, Object],
  handle : Handle,
  object : Object,
) -> Unit {
  if !object.has_modified_flag() {
    self.push_unchecked(handle, object)
  }
}

///|
pub fn[Handle, Object : HasModifiedFlag] ModifiedObjects::push_unchecked(
  self : ModifiedObjects[Handle, Object],
  handle : Handle,
  object : Object,
) -> Unit {
  object.set_modified_flag()
  self.handles.push(handle)
}

///|
pub fn[Handle, Object] ModifiedObjects::as_internal(
  self : ModifiedObjects[Handle, Object],
) -> Array[Handle] {
  self.handles.copy()
}

///|
pub fn[Handle, Object] ModifiedObjects::as_mut_internal(
  self : ModifiedObjects[Handle, Object],
) -> Array[Handle] {
  self.handles
}
