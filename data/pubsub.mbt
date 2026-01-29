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
pub struct Subscription[T] {
  id : Int
  marker : T?
}

///|
struct PubSubCursor {
  // Index into the `offsets` deque, in the same absolute space as `deleted_offsets`.
  mut id : Int
  // Index into the `messages` deque, in the same absolute space as `deleted_messages`.
  mut next : Int
}

///|
pub struct PubSub[T] {
  mut deleted_messages : Int
  mut deleted_offsets : Int
  messages : Array[T]
  offsets : Array[Int]
  cursors : Array[PubSubCursor]
}

///|
const INVALID_OFFSET : Int = -1

///|
pub fn[T] PubSub::new() -> PubSub[T] {
  {
    deleted_messages: 0,
    deleted_offsets: 0,
    messages: [],
    offsets: [],
    cursors: [],
  }
}

///|
fn[T] PubSub::offsets_len(self : PubSub[T]) -> Int {
  self.offsets.length() - self.deleted_offsets
}

///|
fn[T] PubSub::reset_shifts(self : PubSub[T]) -> Unit {
  // Make absolute indices relative again, then drain removed prefixes.
  for i in self.deleted_offsets..<self.offsets.length() {
    let offset = self.offsets[i]
    if offset != INVALID_OFFSET {
      self.offsets[i] = offset - self.deleted_messages
    }
  }
  for i in 0..<self.cursors.length() {
    self.cursors[i].id = self.cursors[i].id - self.deleted_offsets
    self.cursors[i].next = self.cursors[i].next - self.deleted_messages
  }
  if self.deleted_offsets > 0 {
    self.offsets.drain(0, self.deleted_offsets) |> ignore
  }
  if self.deleted_messages > 0 {
    self.messages.drain(0, self.deleted_messages) |> ignore
  }
  self.deleted_offsets = 0
  self.deleted_messages = 0
}

///|
fn[T] PubSub::maybe_reset_shifts(self : PubSub[T]) -> Unit {
  // The rapier implementation resets to avoid u32 overflow. Here we also use it as a compaction
  // mechanism so the arrays don't grow without bounds.
  if self.deleted_messages > 100_000 || self.deleted_offsets > 100_000 {
    self.reset_shifts()
  }
}

///|
pub fn[T] PubSub::publish(self : PubSub[T], message : T) -> Unit {
  if self.offsets_len() == 0 {
    // No subscribers, drop the message.
    return
  }
  self.messages.push(message)
}

///|
pub fn[T] PubSub::subscribe(self : PubSub[T]) -> Subscription[T] {
  let cursor : PubSubCursor = {
    next: self.messages.length(),
    id: self.offsets.length(),
  }
  let sub : Subscription[T] = { id: self.cursors.length(), marker: None }
  self.offsets.push(cursor.next)
  self.cursors.push(cursor)
  sub
}

///|
pub fn[T] PubSub::read_ith(
  self : PubSub[T],
  sub : Subscription[T],
  i : Int,
) -> T? {
  if i < 0 {
    return None
  }
  match self.cursors.get(sub.id) {
    None => None
    Some(cursor) => self.messages.get(cursor.next + i)
  }
}

///|
pub fn[T] PubSub::read(self : PubSub[T], sub : Subscription[T]) -> Array[T] {
  match self.cursors.get(sub.id) {
    None => []
    Some(cursor) => {
      let result : Array[T] = []
      for i in cursor.next..<self.messages.length() {
        result.push(self.messages[i])
      }
      result
    }
  }
}

///|
pub fn[T] PubSub::ack(self : PubSub[T], sub : Subscription[T]) -> Unit {
  if sub.id < 0 || sub.id >= self.cursors.length() {
    return
  }

  // Mark the old offset as obsolete.
  let old_id = self.cursors[sub.id].id
  self.offsets[old_id] = INVALID_OFFSET

  // Move cursor to the newly appended offset at the end.
  let new_id = self.offsets.length()
  let new_next = self.messages.length()
  self.cursors[sub.id].id = new_id
  self.cursors[sub.id].next = new_next
  self.offsets.push(new_next)

  // Drop obsolete offsets at the front.
  while self.deleted_offsets < self.offsets.length() &&
        self.offsets[self.deleted_offsets] == INVALID_OFFSET {
    self.deleted_offsets = self.deleted_offsets + 1
  }

  // There must be at least one offset at this point (otherwise there are no subscribers).
  if self.offsets_len() == 0 {
    return
  }
  let next = self.offsets[self.deleted_offsets]
  let num_to_delete = next - self.deleted_messages
  if num_to_delete > 0 {
    self.deleted_messages = self.deleted_messages + num_to_delete
  }
  self.maybe_reset_shifts()
}
