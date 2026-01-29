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
/// Rapier-aligned deterministic data structures (Arena, Coarena, etc.).

///|
#declaration_only
pub type Index

///|
#declaration_only
pub fn Index::default() -> Index {
  ...
}

///|
#declaration_only
pub fn Index::from_raw_parts(index : Int, generation : Int) -> Index {
  ...
}

///|
#declaration_only
pub fn Index::into_raw_parts(self : Index) -> (Int, Int) {
  ...
}

///|
#declaration_only
pub type Arena[T]

///|
#declaration_only
pub fn[T] Arena::new() -> Arena[T] {
  ...
}

///|
#declaration_only
pub fn[T] Arena::with_capacity(capacity : Int) -> Arena[T] {
  ...
}

///|
#declaration_only
pub fn[T] Arena::clear(self : Arena[T]) -> Unit {
  ...
}

///|
#declaration_only
pub fn[T] Arena::len(self : Arena[T]) -> Int {
  ...
}

///|
#declaration_only
pub fn[T] Arena::is_empty(self : Arena[T]) -> Bool {
  ...
}

///|
#declaration_only
pub fn[T] Arena::capacity(self : Arena[T]) -> Int {
  ...
}

///|
#declaration_only
pub fn[T] Arena::reserve(self : Arena[T], additional : Int) -> Unit {
  ...
}

///|
#declaration_only
pub fn[T] Arena::insert(self : Arena[T], value : T) -> Index {
  ...
}

///|
#declaration_only
pub fn[T] Arena::try_insert(self : Arena[T], value : T) -> Result[Index, T] {
  ...
}

///|
#declaration_only
pub fn[T] Arena::try_insert_with(
  self : Arena[T],
  create : (Index) -> T,
) -> Result[Index, (Index) -> T] {
  ...
}

///|
#declaration_only
pub fn[T] Arena::insert_with(self : Arena[T], create : (Index) -> T) -> Index {
  ...
}

///|
#declaration_only
pub fn[T] Arena::retain(
  self : Arena[T],
  predicate : (Index, T) -> Bool,
) -> Unit {
  ...
}

///|
#declaration_only
pub fn[T] Arena::remove(self : Arena[T], index : Index) -> T? {
  ...
}

///|
#declaration_only
pub fn[T] Arena::contains(self : Arena[T], index : Index) -> Bool {
  ...
}

///|
#declaration_only
pub fn[T] Arena::get(self : Arena[T], index : Index) -> T? {
  ...
}

///|
#declaration_only
pub fn[T] Arena::get_mut(self : Arena[T], index : Index) -> T? {
  ...
}

///|
#declaration_only
pub fn[T] Arena::get2_mut(
  self : Arena[T],
  index1 : Index,
  index2 : Index,
) -> (T?, T?) {
  ...
}

///|
#declaration_only
pub fn[T] Arena::iter(self : Arena[T]) -> Array[(Index, T)] {
  ...
}

///|
#declaration_only
pub fn[T] Arena::iter_mut(self : Arena[T]) -> Array[(Index, T)] {
  ...
}

///|
#declaration_only
pub type Coarena[T]

///|
#declaration_only
pub fn[T] Coarena::new() -> Coarena[T] {
  ...
}

///|
#declaration_only
pub fn[T] Coarena::len(self : Coarena[T]) -> Int {
  ...
}

///|
#declaration_only
pub fn[T] Coarena::reserve(self : Coarena[T], additional : Int) -> Unit {
  ...
}

///|
#declaration_only
pub fn[T] Coarena::iter(self : Coarena[T]) -> Array[(Index, T)] {
  ...
}

///|
#declaration_only
pub fn[T] Coarena::get_unknown_gen(self : Coarena[T], index : Int) -> T? {
  ...
}

///|
#declaration_only
pub fn[T] Coarena::get_mut_unknown_gen(self : Coarena[T], index : Int) -> T? {
  ...
}

///|
#declaration_only
pub fn[T] Coarena::remove(
  self : Coarena[T],
  index : Index,
  removed_value : T,
) -> T? {
  ...
}

///|
#declaration_only
pub fn[T] Coarena::get(self : Coarena[T], index : Index) -> T? {
  ...
}

///|
#declaration_only
pub fn[T] Coarena::get_mut(self : Coarena[T], index : Index) -> T? {
  ...
}

///|
#declaration_only
pub fn[T : Default] Coarena::insert(
  self : Coarena[T],
  index : Index,
  value : T,
) -> Unit {
  ...
}

///|
#declaration_only
pub fn[T] Coarena::ensure_element_exist(
  self : Coarena[T],
  index : Index,
  default : T,
) -> T {
  ...
}

///|
#declaration_only
pub fn[T] Coarena::ensure_pair_exists(
  self : Coarena[T],
  index1 : Index,
  index2 : Index,
  default : T,
) -> (T, T) {
  ...
}

///|
#declaration_only
pub type ModifiedObjects[Handle, Object]

///|
#declaration_only
pub fn[Handle, Object] ModifiedObjects::new() -> ModifiedObjects[Handle, Object] {
  ...
}

///|
#declaration_only
pub fn[Handle, Object] ModifiedObjects::with_capacity(
  capacity : Int,
) -> ModifiedObjects[Handle, Object] {
  ...
}

///|
#declaration_only
pub fn[Handle, Object] ModifiedObjects::clear(
  self : ModifiedObjects[Handle, Object],
) -> Unit {
  ...
}

///|
#declaration_only
pub fn[Handle, Object : HasModifiedFlag] ModifiedObjects::push_once(
  self : ModifiedObjects[Handle, Object],
  handle : Handle,
  object : Object,
) -> Unit {
  ...
}

///|
#declaration_only
pub fn[Handle, Object : HasModifiedFlag] ModifiedObjects::push_unchecked(
  self : ModifiedObjects[Handle, Object],
  handle : Handle,
  object : Object,
) -> Unit {
  ...
}

///|
#declaration_only
pub fn[Handle, Object] ModifiedObjects::as_internal(
  self : ModifiedObjects[Handle, Object],
) -> Array[Handle] {
  ...
}

///|
#declaration_only
pub fn[Handle, Object] ModifiedObjects::as_mut_internal(
  self : ModifiedObjects[Handle, Object],
) -> Array[Handle] {
  ...
}

///|
pub(open) trait HasModifiedFlag {
  has_modified_flag(self : Self) -> Bool
  set_modified_flag(self : Self) -> Unit
}

///|
#declaration_only
pub type Subscription[T]

///|
#declaration_only
pub type PubSub[T]

///|
#declaration_only
pub fn[T] PubSub::new() -> PubSub[T] {
  ...
}

///|
#declaration_only
pub fn[T] PubSub::publish(self : PubSub[T], message : T) -> Unit {
  ...
}

///|
#declaration_only
pub fn[T] PubSub::subscribe(self : PubSub[T]) -> Subscription[T] {
  ...
}

///|
#declaration_only
pub fn[T] PubSub::read_ith(
  self : PubSub[T],
  sub : Subscription[T],
  i : Int,
) -> T? {
  ...
}

///|
#declaration_only
pub fn[T] PubSub::read(self : PubSub[T], sub : Subscription[T]) -> Array[T] {
  ...
}

///|
#declaration_only
pub fn[T] PubSub::ack(self : PubSub[T], sub : Subscription[T]) -> Unit {
  ...
}
