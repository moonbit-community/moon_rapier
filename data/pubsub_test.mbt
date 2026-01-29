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
test "pubsub subscribe/publish/read/ack" {
  let ps : PubSub[Int] = PubSub::new()

  // Dropped because there are no subscribers.
  ps.publish(1)
  let sub = ps.subscribe()
  ps.publish(2)
  ps.publish(3)
  let unread1 = ps.read(sub)
  inspect(unread1 == [2, 3], content="true")
  inspect(ps.read_ith(sub, 0) == Some(2), content="true")
  inspect(ps.read_ith(sub, 1) == Some(3), content="true")
  inspect(ps.read_ith(sub, 2) is None, content="true")
  ps.ack(sub)
  inspect(ps.read(sub).is_empty(), content="true")
  ps.publish(4)
  inspect(ps.read(sub) == [4], content="true")
}
