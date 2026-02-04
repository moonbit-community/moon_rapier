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
test "graph add/iterate/find/remove" {
  let g : Graph[String, Int] = Graph::with_capacity(4, 4)
  let a = g.add_node("a")
  let b = g.add_node("b")
  let e = g.add_edge(a, b, 42)
  inspect(g.node_weight(a) == Some("a"), content="true")
  inspect(g.node_weight(b) == Some("b"), content="true")
  inspect(g.edge_weight(e) == Some(42), content="true")
  match g.edge_endpoints(e) {
    None => inspect(false, content="true")
    Some((ea, eb)) => {
      inspect(ea.index() == a.index(), content="true")
      inspect(eb.index() == b.index(), content="true")
    }
  }

  // Undirected lookup.
  match g.find_edge(a, b) {
    None => inspect(false, content="true")
    Some(ix) => inspect(ix.index() == e.index(), content="true")
  }
  match g.find_edge(b, a) {
    None => inspect(false, content="true")
    Some(ix) => inspect(ix.index() == e.index(), content="true")
  }

  // Iterator yields the edge at least once.
  let it = g.edges(a)
  match it.next() {
    None => inspect(false, content="true")
    Some(er) => {
      inspect(er.id().index() == e.index(), content="true")
      inspect(er.weight() == 42, content="true")
    }
  }
  inspect(g.set_node_weight(a, "a2"), content="true")
  inspect(g.node_weight(a) == Some("a2"), content="true")

  // Mutable edge weight via wrapper.
  let em = g.edge_weight_mut(e)
  match em {
    None => inspect(false, content="true")
    Some(w) => {
      w.set(7)
      inspect(g.edge_weight(e) == Some(7), content="true")
    }
  }

  // Removing a node removes connected edges and may relocate the last node.
  inspect(g.remove_node(a) == Some("a2"), content="true")
  inspect(g.node_weight(a) == Some("b"), content="true")
  inspect(g.find_edge(a, b) is None, content="true")
}
