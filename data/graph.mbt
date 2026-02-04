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
/// A stripped-down undirected graph adapted from Rapier's internal graph
/// (itself based on petgraph's UnGraph).
///
/// Note: this is a capability-equivalent MoonBit representation; iteration
/// yields copies instead of references.

///|
pub struct NodeIndex {
  index : Int
}

///|
pub fn NodeIndex::new(x : Int) -> NodeIndex {
  { index: x }
}

///|
pub fn NodeIndex::index(self : NodeIndex) -> Int {
  self.index
}

///|
pub fn NodeIndex::end() -> NodeIndex {
  NodeIndex::new(-1)
}

///|
pub struct EdgeIndex {
  index : Int
}

///|
pub fn EdgeIndex::new(x : Int) -> EdgeIndex {
  { index: x }
}

///|
pub fn EdgeIndex::index(self : EdgeIndex) -> Int {
  self.index
}

///|
pub fn EdgeIndex::end() -> EdgeIndex {
  EdgeIndex::new(-1)
}

///|
pub(all) enum Direction {
  Outgoing
  Incoming
}

///|
pub fn Direction::opposite(self : Direction) -> Direction {
  match self {
    Direction::Outgoing => Direction::Incoming
    Direction::Incoming => Direction::Outgoing
  }
}

///|
pub struct Node[N] {
  mut weight : N
  mut next_out : EdgeIndex
  mut next_in : EdgeIndex
}

///|
pub struct Edge[E] {
  mut weight : E
  mut next_out : EdgeIndex
  mut next_in : EdgeIndex
  mut node0 : NodeIndex
  mut node1 : NodeIndex
}

///|
pub fn[E] Edge::source(self : Edge[E]) -> NodeIndex {
  self.node0
}

///|
pub fn[E] Edge::target(self : Edge[E]) -> NodeIndex {
  self.node1
}

///|
pub struct Graph[N, E] {
  nodes : Array[Node[N]]
  edges : Array[Edge[E]]
}

///|
pub fn[N, E] Graph::with_capacity(nodes : Int, edges : Int) -> Graph[N, E] {
  let ns : Array[Node[N]] = []
  let es : Array[Edge[E]] = []
  ns.reserve_capacity(nodes)
  es.reserve_capacity(edges)
  { nodes: ns, edges: es }
}

///|
pub fn[N, E] Graph::add_node(self : Graph[N, E], weight : N) -> NodeIndex {
  let idx = NodeIndex::new(self.nodes.length())
  self.nodes.push({
    weight,
    next_out: EdgeIndex::end(),
    next_in: EdgeIndex::end(),
  })
  idx
}

///|
pub fn[N, E] Graph::node_weight(self : Graph[N, E], a : NodeIndex) -> N? {
  let i = a.index()
  if i < 0 || i >= self.nodes.length() {
    return None
  }
  Some(self.nodes[i].weight)
}

///|
pub fn[N, E] Graph::set_node_weight(
  self : Graph[N, E],
  a : NodeIndex,
  weight : N,
) -> Bool {
  let i = a.index()
  if i < 0 || i >= self.nodes.length() {
    return false
  }
  let n = self.nodes[i]
  n.weight = weight
  self.nodes[i] = n
  true
}

///|
pub fn[N, E] Graph::edge_weight(self : Graph[N, E], a : EdgeIndex) -> E? {
  let i = a.index()
  if i < 0 || i >= self.edges.length() {
    return None
  }
  Some(self.edges[i].weight)
}

///|
pub struct EdgeWeightMut[N, E] {
  graph : Graph[N, E]
  edge : EdgeIndex
}

///|
pub fn[N, E] EdgeWeightMut::get(self : EdgeWeightMut[N, E]) -> E {
  self.graph.edges[self.edge.index()].weight
}

///|
pub fn[N, E] EdgeWeightMut::set(self : EdgeWeightMut[N, E], value : E) -> Unit {
  let idx = self.edge.index()
  let e = self.graph.edges[idx]
  e.weight = value
  self.graph.edges[idx] = e
}

///|
pub fn[N, E] Graph::edge_weight_mut(
  self : Graph[N, E],
  a : EdgeIndex,
) -> EdgeWeightMut[N, E]? {
  let i = a.index()
  if i < 0 || i >= self.edges.length() {
    return None
  }
  Some({ graph: self, edge: a })
}

///|
pub fn[N, E] Graph::add_edge(
  self : Graph[N, E],
  a : NodeIndex,
  b : NodeIndex,
  weight : E,
) -> EdgeIndex {
  let ai = a.index()
  let bi = b.index()
  if ai < 0 || ai >= self.nodes.length() || bi < 0 || bi >= self.nodes.length() {
    panic()
  }
  let edge_idx = EdgeIndex::new(self.edges.length())
  let edge : Edge[E] = {
    weight,
    next_out: EdgeIndex::end(),
    next_in: EdgeIndex::end(),
    node0: a,
    node1: b,
  }
  if ai == bi {
    let an = self.nodes[ai]
    edge.next_out = an.next_out
    edge.next_in = an.next_in
    an.next_out = edge_idx
    an.next_in = edge_idx
    self.nodes[ai] = an
  } else {
    let an = self.nodes[ai]
    let bn = self.nodes[bi]
    edge.next_out = an.next_out
    edge.next_in = bn.next_in
    an.next_out = edge_idx
    bn.next_in = edge_idx
    self.nodes[ai] = an
    self.nodes[bi] = bn
  }
  self.edges.push(edge)
  edge_idx
}

///|
pub fn[N, E] Graph::edge_endpoints(
  self : Graph[N, E],
  e : EdgeIndex,
) -> (NodeIndex, NodeIndex)? {
  let i = e.index()
  if i < 0 || i >= self.edges.length() {
    return None
  }
  let ed = self.edges[i]
  Some((ed.source(), ed.target()))
}

///|
fn[N, E] Graph::change_edge_links(
  self : Graph[N, E],
  edge_node0 : NodeIndex,
  edge_node1 : NodeIndex,
  e : EdgeIndex,
  edge_next_out : EdgeIndex,
  edge_next_in : EdgeIndex,
) -> Unit {
  // Outgoing list on node0.
  let n0i = edge_node0.index()
  if n0i >= 0 && n0i < self.nodes.length() {
    let n0 = self.nodes[n0i]
    let fst = n0.next_out
    if fst.index() == e.index() {
      n0.next_out = edge_next_out
      self.nodes[n0i] = n0
    } else {
      let mut cur = fst
      while cur.index() >= 0 {
        let ci = cur.index()
        if ci < 0 || ci >= self.edges.length() {
          break
        }
        let ce = self.edges[ci]
        if ce.next_out.index() == e.index() {
          ce.next_out = edge_next_out
          self.edges[ci] = ce
          break
        }
        cur = ce.next_out
      }
    }
  }

  // Incoming list on node1.
  let n1i = edge_node1.index()
  if n1i >= 0 && n1i < self.nodes.length() {
    let n1 = self.nodes[n1i]
    let fst = n1.next_in
    if fst.index() == e.index() {
      n1.next_in = edge_next_in
      self.nodes[n1i] = n1
    } else {
      let mut cur = fst
      while cur.index() >= 0 {
        let ci = cur.index()
        if ci < 0 || ci >= self.edges.length() {
          break
        }
        let ce = self.edges[ci]
        if ce.next_in.index() == e.index() {
          ce.next_in = edge_next_in
          self.edges[ci] = ce
          break
        }
        cur = ce.next_in
      }
    }
  }
}

///|
fn[N, E] Graph::swap_remove_edge(self : Graph[N, E], idx : Int) -> Edge[E]? {
  let last = self.edges.length() - 1
  if idx < 0 || idx > last {
    return None
  }
  let removed = self.edges[idx]
  if idx != last {
    self.edges[idx] = self.edges[last]
  }
  self.edges.pop() |> ignore
  Some(removed)
}

///|
fn[N, E] Graph::remove_edge_adjust_indices(
  self : Graph[N, E],
  e : EdgeIndex,
) -> E? {
  let removed = self.swap_remove_edge(e.index())
  match removed {
    None => None
    Some(edge) => {
      let swapped = if e.index() >= 0 && e.index() < self.edges.length() {
        Some((self.edges[e.index()].node0, self.edges[e.index()].node1))
      } else {
        None
      }
      match swapped {
        None => Some(edge.weight)
        Some((s0, s1)) => {
          let swapped_e = EdgeIndex::new(self.edges.length())
          self.change_edge_links(s0, s1, swapped_e, e, e)
          Some(edge.weight)
        }
      }
    }
  }
}

///|
pub fn[N, E] Graph::remove_edge(self : Graph[N, E], e : EdgeIndex) -> E? {
  let i = e.index()
  if i < 0 || i >= self.edges.length() {
    return None
  }
  let ed = self.edges[i]
  self.change_edge_links(ed.node0, ed.node1, e, ed.next_out, ed.next_in)
  self.remove_edge_adjust_indices(e)
}

///|
fn[N, E] Graph::swap_remove_node(self : Graph[N, E], idx : Int) -> Node[N]? {
  let last = self.nodes.length() - 1
  if idx < 0 || idx > last {
    return None
  }
  let removed = self.nodes[idx]
  if idx != last {
    self.nodes[idx] = self.nodes[last]
  }
  self.nodes.pop() |> ignore
  Some(removed)
}

///|
pub fn[N, E] Graph::remove_node(self : Graph[N, E], a : NodeIndex) -> N? {
  let ai = a.index()
  if ai < 0 || ai >= self.nodes.length() {
    return None
  }

  // Remove all edges from and to this node.
  while true {
    let next = self.nodes[ai].next_out
    if next.index() < 0 {
      break
    }
    self.remove_edge(next) |> ignore
  }
  while true {
    let next = self.nodes[ai].next_in
    if next.index() < 0 {
      break
    }
    self.remove_edge(next) |> ignore
  }
  let removed = self.swap_remove_node(ai)
  match removed {
    None => None
    Some(node) => {
      // If no node had to relocate, we are done.
      if ai >= self.nodes.length() {
        return Some(node.weight)
      }

      // The swapped element's old index (last).
      let old_index = NodeIndex::new(self.nodes.length())
      let new_index = a
      let swap_edges_out = self.nodes[ai].next_out
      let swap_edges_in = self.nodes[ai].next_in

      // Adjust the starts of the out edges.
      let mut ecur = swap_edges_out
      while ecur.index() >= 0 {
        let ei = ecur.index()
        if ei < 0 || ei >= self.edges.length() {
          break
        }
        let ed = self.edges[ei]
        ed.node0 = new_index
        self.edges[ei] = ed
        ecur = ed.next_out
      }

      // Adjust the ends of the in edges.
      ecur = swap_edges_in
      while ecur.index() >= 0 {
        let ei = ecur.index()
        if ei < 0 || ei >= self.edges.length() {
          break
        }
        let ed = self.edges[ei]
        ed.node1 = new_index
        self.edges[ei] = ed
        ecur = ed.next_in
      }

      // Sanity: old_index currently unused; kept for parity with Rapier's algorithm.
      let _ = old_index
      Some(node.weight)
    }
  }
}

///|
pub struct Edges[N, E] {
  graph : Graph[N, E]
  skip_start : NodeIndex
  direction : Direction
  mut next_out : EdgeIndex
  mut next_in : EdgeIndex
}

///|
pub fn[N, E] Graph::edges(self : Graph[N, E], a : NodeIndex) -> Edges[N, E] {
  self.edges_directed(a, Direction::Outgoing)
}

///|
pub fn[N, E] Graph::edges_directed(
  self : Graph[N, E],
  a : NodeIndex,
  dir : Direction,
) -> Edges[N, E] {
  let ai = a.index()
  if ai < 0 || ai >= self.nodes.length() {
    return {
      graph: self,
      skip_start: a,
      direction: dir,
      next_out: EdgeIndex::end(),
      next_in: EdgeIndex::end(),
    }
  }
  let n = self.nodes[ai]
  {
    graph: self,
    skip_start: a,
    direction: dir,
    next_out: n.next_out,
    next_in: n.next_in,
  }
}

///|
pub struct EdgeReference[E] {
  index : EdgeIndex
  weight : E
}

///|
pub fn[E] EdgeReference::id(self : EdgeReference[E]) -> EdgeIndex {
  self.index
}

///|
pub fn[E] EdgeReference::weight(self : EdgeReference[E]) -> E {
  self.weight
}

///|
pub fn[N, E] Edges::next(self : Edges[N, E]) -> EdgeReference[E]? {
  // Mirror Rapier's Edges iterator behavior for an undirected graph: first walk
  // the outgoing list, then the incoming list while skipping self loops.
  let _ = self.direction
  if self.next_out.index() >= 0 {
    let i = self.next_out.index()
    if i >= 0 && i < self.graph.edges.length() {
      let ed = self.graph.edges[i]
      self.next_out = ed.next_out
      return Some({ index: EdgeIndex::new(i), weight: ed.weight })
    }
    self.next_out = EdgeIndex::end()
  }
  while self.next_in.index() >= 0 {
    let edge_index = self.next_in
    let i = edge_index.index()
    if i < 0 || i >= self.graph.edges.length() {
      self.next_in = EdgeIndex::end()
      break
    }
    let ed = self.graph.edges[i]
    self.next_in = ed.next_in
    if ed.node0.index() == self.skip_start.index() {
      continue
    }
    return Some({ index: edge_index, weight: ed.weight })
  }
  None
}

///|
pub fn[N, E] Graph::find_edge(
  self : Graph[N, E],
  a : NodeIndex,
  b : NodeIndex,
) -> EdgeIndex? {
  match self.find_edge_undirected(a, b) {
    None => None
    Some((ix, _)) => Some(ix)
  }
}

///|
pub fn[N, E] Graph::find_edge_undirected(
  self : Graph[N, E],
  a : NodeIndex,
  b : NodeIndex,
) -> (EdgeIndex, Direction)? {
  let ai = a.index()
  if ai < 0 || ai >= self.nodes.length() {
    return None
  }
  let node = self.nodes[ai]

  // Outgoing
  let mut ecur = node.next_out
  while ecur.index() >= 0 {
    let i = ecur.index()
    if i < 0 || i >= self.edges.length() {
      break
    }
    let ed = self.edges[i]
    if ed.node1.index() == b.index() {
      return Some((ecur, Direction::Outgoing))
    }
    ecur = ed.next_out
  }

  // Incoming
  ecur = node.next_in
  while ecur.index() >= 0 {
    let i = ecur.index()
    if i < 0 || i >= self.edges.length() {
      break
    }
    let ed = self.edges[i]
    if ed.node0.index() == b.index() {
      return Some((ecur, Direction::Incoming))
    }
    ecur = ed.next_in
  }
  None
}

///|
pub fn[N, E] Graph::raw_nodes(self : Graph[N, E]) -> Array[Node[N]] {
  self.nodes
}

///|
pub fn[N, E] Graph::raw_edges(self : Graph[N, E]) -> Array[Edge[E]] {
  self.edges
}

///|
pub fn[N, E] Graph::first_edge(
  self : Graph[N, E],
  a : NodeIndex,
  dir : Direction,
) -> EdgeIndex? {
  let ai = a.index()
  if ai < 0 || ai >= self.nodes.length() {
    return None
  }
  let edix = match dir {
    Direction::Outgoing => self.nodes[ai].next_out
    Direction::Incoming => self.nodes[ai].next_in
  }
  if edix.index() < 0 {
    None
  } else {
    Some(edix)
  }
}

///|
pub fn[N, E] Graph::next_edge(
  self : Graph[N, E],
  e : EdgeIndex,
  dir : Direction,
) -> EdgeIndex? {
  let ei = e.index()
  if ei < 0 || ei >= self.edges.length() {
    return None
  }
  let edix = match dir {
    Direction::Outgoing => self.edges[ei].next_out
    Direction::Incoming => self.edges[ei].next_in
  }
  if edix.index() < 0 {
    None
  } else {
    Some(edix)
  }
}
