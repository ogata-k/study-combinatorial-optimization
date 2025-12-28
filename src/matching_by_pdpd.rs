//! 非負整数辺重み二部グラフの重み最小辺数最大マッチングを求める。
//! Potential付きDijkstraによって最小費用流を求めることでマッチングを求めるPrimal-Dual法を採用したアルゴリズムで最大マッチングを求める。

// 実装の参考先
// https://tjkendev.github.io/procon-library/python/min_cost_flow/primal-dual.html
// https://yatani.jp/teaching/lib/exe/fetch.php?media=2025algorithms:13-graph4-pnp.pdf
// https://drken1215.hatenablog.com/entry/2023/10/31/232108
// https://ei1333.github.io/luzhiled/snippets/graph/primal-dual.html
// https://pione.hatenablog.com/entry/2021/02/28/075034

use std::cmp::Reverse;
use std::collections::BinaryHeap;

type Capacity = u16;
// このu8::MAX=256程度のコストならDial法（キューをコストごとのバケットで代用する方法）を用いてDijkstra法をより高速にできる。
// 具体的には、ノード数が1000*1000を超えると今の実装の４～５倍程度に高速になる。（特に密グラフで効果は大きい）
// ただ、Dial法を採用したDijkstra法は読みにくくメンテナンスもしにくくコスト上限の変更に弱くなるので、今回は不採用にしている。
/// マッチコスト。小さいほど優先。
/// - 非負整数
/// - 実運用では 0 < cost <= 200 程度を想定
pub type Cost = u8;
/// 最終的に手に入るマッチングである最大マッチングのコスト
pub type TotalCost = u32;
type ExtendTotalCost = i64;
/// コストを辺に指定するときに負のコストとして指定することがあるので拡張
type ExtendCost = i64;

/// ノードとして利用できる型が満たしていてほしい情報
pub type NodeId = usize;

/// マッチングを求めるのに利用するグラフのための有向辺
#[derive(Debug, Clone)]
struct Edge {
    to_node_index: usize,
    capacity: Capacity,
    cost: ExtendCost,
    /// to_nodeから出ている辺の何本目か
    reverse_edge_index: usize,
    /// 同じノードから出ている次に辿るべき辺。
    next_edge_index: Option<usize>,
}

/// マッチングを求めるためのグラフ
#[derive(Debug, Clone)]
struct BipartiteResidualGraph {
    all_node_count: usize,
    left_node_count: usize,
    right_node_count: usize,
    /// 各ノードがどの辺からたどればいいかを表す。Noneなら辺が出ていない。
    /// edgesと併せてChain Forward Star（CFS）と呼ばれる方法で、隣接リストを配列で管理する方法となる。
    /// CFSは、メモリ効率がよく、キャッシュヒット率もよい。
    heads: Vec<Option<usize>>,
    edges: Vec<Edge>,
}

impl BipartiteResidualGraph {
    /// source, sinkを考慮してグラフを初期化する
    fn new(left_node_count: usize, right_node_count: usize) -> Self {
        let all_node_count = left_node_count + right_node_count + 2;
        BipartiteResidualGraph {
            all_node_count,
            left_node_count,
            right_node_count,
            heads: vec![None; all_node_count],
            // 辺は最大で、source-> Left -> Right -> sinkの順方向と逆方向の２本這うことができるので、あらかじめ容量を確保
            edges: Vec::with_capacity(
                2 * (left_node_count * right_node_count + left_node_count + right_node_count),
            ),
        }
    }

    // 同じ(from_index, to_index)の辺が指定されることと、from_index > to_indexの辺が指定されることは想定していない
    fn add_edge(&mut self, from_index: usize, to_index: usize, capacity: Capacity, cost: Cost) {
        // これから挿入するXX_edge_indexのXXから出ている辺のedgesのインデックスを用意
        let edge_index = self.edges.len();
        let reverse_edge_index = edge_index + 1;

        // 通常の順方向に辺を張る
        self.edges.push(Edge {
            to_node_index: to_index,
            capacity,
            cost: cost as ExtendCost,
            reverse_edge_index,
            // 以前最初に辿っていた辺を次に辿るべき次の辺として指定
            next_edge_index: self.heads[from_index],
        });
        // 最初に辿るべき辺を更新
        self.heads[from_index] = Some(edge_index);

        // アルゴリズムの残余グラフのために逆方向にも辺を張る
        self.edges.push(Edge {
            to_node_index: from_index,
            capacity: 0,
            cost: -(cost as ExtendCost),
            reverse_edge_index: edge_index,
            // 以前最初に辿っていた辺を次に辿るべき次の辺として指定
            next_edge_index: self.heads[to_index],
        });
        // 最初に辿るべき辺を更新
        self.heads[to_index] = Some(reverse_edge_index);
    }
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
struct DijkstraState {
    node_index: NodeId,
    tmp_current_total_cost: ExtendTotalCost,
}

impl Ord for DijkstraState {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        // 現在計算したコストだけを使って比較する
        self.tmp_current_total_cost
            .cmp(&other.tmp_current_total_cost)
            // indexでも比較することで全順序にする
            .then_with(|| self.node_index.cmp(&other.node_index))
    }
}

impl PartialOrd for DijkstraState {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

/// 二部マッチングを解くためのソルバー
#[derive(Debug, Clone)]
pub struct MatchingSolver {
    source_node_index: usize,
    sink_node_index: usize,
    graph: BipartiteResidualGraph,
}

const FLOW_CAPACITY: Capacity = 1;
const DUMMY_COST: Cost = 0;
impl MatchingSolver {
    /// MatchingSolverの初期化。
    /// left_count + right_count + 2 <= usize::MAXを仮定している。
    pub fn new(left_count: usize, right_count: usize) -> Self {
        let graph = BipartiteResidualGraph::new(left_count, right_count);
        let source_node_index = left_count + right_count;
        let sink_node_index = source_node_index + 1;

        Self {
            source_node_index,
            sink_node_index,
            graph,
        }
    }

    /// マッチしてほしい組を登録
    /// left_index < left_node_count, right_index < right_node_countを仮定している。
    /// また、同じ(left_index, right_index)を持つ辺を張ることは想定していない
    pub fn add_candidate(
        &mut self,
        left_index: NodeId,
        right_index: NodeId,
        cost: Cost,
    ) -> Result<(), String> {
        let left_node_count = self.graph.left_node_count;
        let right_node_count = self.graph.right_node_count;

        if left_index >= left_node_count || right_index >= right_node_count {
            return Err("left_index or right_index is out of range".to_string());
        }

        if cost == 0 {
            return Err("0 cost is invalid".to_string());
        }

        self.graph.add_edge(
            left_index,
            left_node_count + right_index,
            FLOW_CAPACITY,
            cost,
        );

        Ok(())
    }

    fn setup_source_sink(&mut self) {
        let left_node_count = self.graph.left_node_count;
        let right_node_count = self.graph.right_node_count;
        let source_node_index = self.source_node_index;
        let sink_node_index = self.sink_node_index;

        // source -> leftに最大１流せる辺を張る
        // 今回は二部マッチング目的なので、もとからある二部グラフの特定のノードをsourceとするわけではない。
        // sourceはいわゆるダミーのノードなので、固定でコストは0にしておく。
        for i in 0..left_node_count {
            self.graph
                .add_edge(source_node_index, i, FLOW_CAPACITY, DUMMY_COST);
        }

        // right -> sinkに最大１流せる辺を張る
        // 今回は二部マッチング目的なので、もとからある二部グラフの特定のノードをsinkとするわけではない。
        // sinkはいわゆるダミーのノードなので、固定でコストは0にしておく。
        for i in 0..right_node_count {
            self.graph.add_edge(
                left_node_count + i,
                sink_node_index,
                FLOW_CAPACITY,
                DUMMY_COST,
            )
        }
    }

    /// マッチしたleftとrightの組の一覧を返す
    ///
    /// L: leftのノード数, R: rightのノード数, V = L+R, E = 候補の辺の数, F = マッチング数
    /// 最悪計算量: O(F · E · log V), F ≤ min(L, R)
    /// 平均計算量: O(F · E · log V)
    /// 空間計算量: O(V + E)
    pub fn solve(mut self) -> Result<(TotalCost, Vec<(NodeId, Option<(NodeId, Cost)>)>), String> {
        self.setup_source_sink();

        let all_node_count = self.graph.all_node_count;
        let left_node_count = self.graph.left_node_count;

        let source_node_index = self.source_node_index;
        let sink_node_index = self.sink_node_index;

        const INF: ExtendTotalCost = ExtendTotalCost::MAX / 4;
        let mut total_cost: ExtendTotalCost = 0;
        // 辺の重みを非負にしてダイクストラ法を使うことができるようにポテンシャルを導入
        // 今回は二部グラフのマッチングなので、ポテンシャルは0の決め打ちで初期化してしまって問題ない。
        // というのも、ダイクストラ法が使えない理由である負の辺が初回は存在しないことが保証されているため。
        let mut potential: Vec<i64> = vec![0; all_node_count];

        // 各ノードに来る前に度のノード、辺を辿ってきたかを表す
        let mut prev_node: Vec<Option<usize>> = vec![None; all_node_count];
        let mut prev_edge: Vec<Option<usize>> = vec![None; all_node_count];

        // 今回は流せるだけ流してみて流せなくなったら終了する方式
        // この方式により辺数の最大を保証する
        // 一時コストを初期化
        let mut current_total_cost = vec![INF; all_node_count];
        // 最小重みを求めて辿りたいので、最小ヒープを使う。そのため、逆順にソートして最大ヒープを最小ヒープとして扱う。
        let mut priority_queue: BinaryHeap<Reverse<DijkstraState>> = BinaryHeap::new();

        loop {
            // ダイクストラ法で最短経路を求めるのに利用するひとつ前の辺のデータを初期化
            // その回のダイクストラで到達しなかったノードは参照されないので、理論上では初期化は不要。
            prev_node.fill(None);
            prev_edge.fill(None);

            // 一時コストを初期化
            current_total_cost.fill(INF);

            // ヒープキューの初期化はダイクストラ法を実施した後は常に空なのでしてもしなくても同じなのでしていない
            // priority_queue.clear();

            // フローの開始点は最初から来ているのでコスト=0で辿ることができる。
            current_total_cost[source_node_index] = 0;
            // 辿ることができているので、次に探索する予定のキューに挿入
            priority_queue.push(Reverse(DijkstraState {
                node_index: source_node_index,
                tmp_current_total_cost: 0,
            }));
            while let Some(state) = priority_queue.pop() {
                let DijkstraState {
                    node_index,
                    tmp_current_total_cost,
                } = state.0;

                // 今わかっている最小コスト（= current_total_cost[node_id]）が、
                // 過去に探索したときのコスト（= tmp_current_total_cost）より悪化しているので探索しても改善しない。
                // なので、探索しても無駄なので不要な計算をさせないためにスキップする。
                // 補足：Dijkstra法 特有の最適化
                if current_total_cost[node_index] < tmp_current_total_cost {
                    continue;
                }

                let mut _edge_index: Option<&usize> = self.graph.heads[node_index].as_ref();
                while let Some(edge_index) = _edge_index {
                    let edge = self
                        .graph
                        .edges
                        .get(*edge_index)
                        .ok_or_else(|| "invalid edge_index".to_string())?;

                    // コストを非負化
                    let reduced_cost =
                        edge.cost + potential[node_index] - potential[edge.to_node_index];
                    let tmp_current_node_total_cost = current_total_cost[node_index] + reduced_cost;
                    // まだフローに流せて、調整済みコストを辿ったときに今わかっている最小コストより小さいなら更新できるので更新する。
                    if edge.capacity > 0
                        && tmp_current_node_total_cost < current_total_cost[edge.to_node_index]
                    {
                        current_total_cost[edge.to_node_index] = tmp_current_node_total_cost;
                        // 更新できたのでto_node_idに流れてくるときのノードと流す辺を更新
                        prev_node[edge.to_node_index] = Some(node_index);
                        prev_edge[edge.to_node_index] = Some(*edge_index);
                        priority_queue.push(Reverse(DijkstraState {
                            node_index: edge.to_node_index,
                            tmp_current_total_cost: tmp_current_node_total_cost,
                        }));
                    }
                    _edge_index = edge.next_edge_index.as_ref();
                }
            }

            if current_total_cost[sink_node_index] == INF {
                // ポテンシャル調整後のグラフで sink へ到達できないため、これ以上フローを増やせず、最大マッチングが確定
                // つまり、最大マッチングが確定。
                break;
            }

            // ポテンシャルの更新
            for i in 0..all_node_count {
                //　到達ノード（=コストが更新されているノード）のみを対象にして更新する
                if current_total_cost[i] != INF {
                    // ダイクストラ法でのコスト更新時にtmp_current_node_total_cost < current_total_cost[edge.to_node_index]となっているため、
                    // 現在のコストを足すだけで、すべての残余辺のreduced costが非負になることが数学的に保証できる。
                    potential[i] += current_total_cost[i];
                }
            }

            // 流せる量を決定し、まだ流せるか確認（マッチングなら通常１）
            let partial_flow = 1;

            // 今回流した partial_flow（今回は1なので、1をかける必要はなし。）分の最小追加コスト（= potential[sink]）
            // sink のポテンシャルは、今回流した 1 単位の最小追加コストに等しい
            total_cost += potential[sink_node_index];

            // 残余グラフの更新
            let mut current_residual_node_index = sink_node_index;
            while current_residual_node_index != source_node_index {
                let prev_edge_index = prev_edge[current_residual_node_index]
                    .ok_or_else(|| "invalid node_index for residual flow graph".to_string())?;
                let rev_prev_edge_index = self.graph.edges[prev_edge_index].reverse_edge_index;

                // 順方向は追加で流した分減らし、逆方向はその分増やす。
                // 増やす理由は、後からやっぱり流すのをやめたとできるようにするため。
                // 詳しくは、残余グラフを用いたマッチング用アルゴリズム一般の話を参照のこと
                self.graph.edges[prev_edge_index].capacity -= partial_flow;
                self.graph.edges[rev_prev_edge_index].capacity += partial_flow;

                // 次は現在のノードのひとつ前のノード
                current_residual_node_index =
                    prev_node[current_residual_node_index].ok_or_else(|| {
                        "invalid previous node for residual flow graph's flow path".to_string()
                    })?;
            }
        }

        let mut matching: Vec<(NodeId, Option<(NodeId, Cost)>)> = Vec::new();
        for left_node_index in 0..left_node_count {
            let mut _edge_index = self.graph.heads[left_node_index];
            loop {
                match _edge_index {
                    None => {
                        // 該当する辺なしということでマッチしなかった
                        matching.push((left_node_index, None));
                        break;
                    }
                    Some(edge_index) => {
                        let edge = self.graph.edges.get(edge_index).ok_or_else(|| {
                            "invalid edge index for construct matching result".to_string()
                        })?;

                        // Leftノードのみを走査している。そのため、left_node_indexはsource/sinkを指すことはない。
                        // また、ノード番号の割り当ての都合上、Left->Right辺のto_node_indexは[L, L+R)の範囲に収まるため、
                        // source/sinkのインデックスはこの範囲外にあるためto_node_indexとしても考慮不要。
                        // よって、capacity==0なLeft->Right辺はその条件式だけでそのままマッチングとして復元できる。
                        // 今回は（不要ではあるが）念のためとして指定して安全よりの実装にしてある。
                        if edge.capacity == 0
                            && left_node_count <= edge.to_node_index
                            && edge.to_node_index < left_node_count + self.graph.right_node_count
                            && edge.cost != DUMMY_COST as ExtendCost
                        {
                            // マッチング
                            matching.push((
                                left_node_index,
                                // LeftからRightへ出ている辺だけなので、要求された割当候補のコストにedge.costは制限されている。そのため、Costへキャストして問題ない、
                                Some((edge.to_node_index - left_node_count, edge.cost as Cost)),
                            ));
                            break;
                        } else {
                            _edge_index = edge.next_edge_index;
                        }
                    }
                }
            }
        }

        Ok((total_cost as TotalCost, matching))
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use std::collections::HashSet;

    #[test]
    fn get_matching_when_nothing_node() {
        // ノードなしでもマッチング結果の取得ができること
        let solver = MatchingSolver::new(0, 0);
        let result = solver.solve();
        assert!(result.is_ok());
        assert_eq!(result.unwrap(), (0, vec![]));
    }

    #[test]
    fn get_matching_when_only_left_node() {
        // Leftノードだけでもマッチング結果の取得ができること
        let solver = MatchingSolver::new(10, 0);
        let result = solver.solve();
        assert!(result.is_ok());
        assert_eq!(
            result.unwrap(),
            (
                0,
                vec![
                    (0, None),
                    (1, None),
                    (2, None),
                    (3, None),
                    (4, None),
                    (5, None),
                    (6, None),
                    (7, None),
                    (8, None),
                    (9, None),
                ]
            )
        );
    }

    #[test]
    fn get_matching_when_only_right_node() {
        // Rightノードだけでもマッチング結果の取得ができること
        let solver = MatchingSolver::new(0, 15);
        let result = solver.solve();
        assert!(result.is_ok());
        assert_eq!(result.unwrap(), (0, vec![]));
    }

    #[test]
    fn get_matching_when_only_node_nothing_edge() {
        // LeftノードとRightノードだけでもマッチング結果の取得ができること
        let solver = MatchingSolver::new(15, 10);
        let result = solver.solve();
        assert!(result.is_ok());
        assert_eq!(
            result.unwrap(),
            (
                0,
                vec![
                    (0, None),
                    (1, None),
                    (2, None),
                    (3, None),
                    (4, None),
                    (5, None),
                    (6, None),
                    (7, None),
                    (8, None),
                    (9, None),
                    (10, None),
                    (11, None),
                    (12, None),
                    (13, None),
                    (14, None),
                ]
            )
        );
    }

    #[test]
    fn only_left_node_candidate_one() {
        // Leftノードがいくつかあるが、一つのLeftノードが一つだけリクエストを出しているとき、ちゃんと選ばれていること
        let mut solver = MatchingSolver::new(3, 5);
        assert_eq!(solver.add_candidate(1, 3, 7), Ok(()));
        let result = solver.solve();
        assert!(result.is_ok());
        assert_eq!(
            result.unwrap(),
            (7, vec![(0, None), (1, Some((3, 7))), (2, None)])
        );
    }

    #[test]
    fn only_left_node_candidate_many() {
        // Leftノードがいくつかあるが、一つのLeftノードいくつかリクエストを出しているとき、ちゃんと最小のコストのリクエストが選ばれていること
        let mut solver = MatchingSolver::new(3, 5);
        assert_eq!(solver.add_candidate(1, 1, 2), Ok(()));
        assert_eq!(solver.add_candidate(1, 3, 7), Ok(()));
        assert_eq!(solver.add_candidate(1, 4, 6), Ok(()));
        let result = solver.solve();
        assert!(result.is_ok());
        assert_eq!(
            result.unwrap(),
            (2, vec![(0, None), (1, Some((1, 2))), (2, None)])
        );
    }

    #[test]
    fn only_right_node_candidate_many() {
        // Leftノードがいくつかあるが、一つのLeftノードいくつかリクエストを出しているとき、ちゃんと最小のコストのリクエストが選ばれていること
        let mut solver = MatchingSolver::new(4, 5);
        assert_eq!(solver.add_candidate(1, 2, 3), Ok(()));
        assert_eq!(solver.add_candidate(2, 2, 7), Ok(()));
        assert_eq!(solver.add_candidate(3, 2, 6), Ok(()));
        let result = solver.solve();
        assert!(result.is_ok());
        assert_eq!(
            result.unwrap(),
            (3, vec![(0, None), (1, Some((2, 3))), (2, None), (3, None)])
        );
    }

    #[test]
    fn many_candidate_get_matching_1() {
        // 複数のリクエストが指定されたときにちゃんと最小のコストのリクエストが選ばれていること
        let candidates: Vec<(NodeId, NodeId, Cost)> =
            vec![(0, 0, 10), (0, 2, 2), (1, 0, 5), (1, 2, 8)];
        let mut solver = MatchingSolver::new(4, 3);
        for candidate in candidates {
            assert_eq!(
                solver.add_candidate(candidate.0, candidate.1, candidate.2),
                Ok(())
            );
        }

        let result = solver.solve();
        assert!(result.is_ok());
        assert_eq!(
            result.unwrap(),
            (
                7,
                vec![(0, Some((2, 2))), (1, Some((0, 5))), (2, None), (3, None)]
            )
        );
    }

    #[test]
    fn many_candidate_get_matching_2() {
        // 複数のリクエストが指定されたときにちゃんと最小のコストのリクエストが選ばれていること
        let candidates: Vec<(NodeId, NodeId, Cost)> =
            vec![(0, 0, 10), (0, 1, 9), (1, 0, 9), (1, 1, 1)];
        let mut solver = MatchingSolver::new(2, 2);
        for candidate in candidates {
            assert_eq!(
                solver.add_candidate(candidate.0, candidate.1, candidate.2),
                Ok(())
            );
        }

        let result = solver.solve();
        assert!(result.is_ok());
        assert_eq!(
            result.unwrap(),
            (11, vec![(0, Some((0, 10))), (1, Some((1, 1)))])
        );
    }

    #[test]
    fn many_candidate_get_matching_3() {
        // 複数のリクエストが指定されたときにちゃんと最小のコストのリクエストが選ばれていること
        let candidates: Vec<(NodeId, NodeId, Cost)> = vec![
            (0, 0, 5),
            (1, 0, 10),
            (2, 0, 5),
            (0, 1, 1),
            (1, 1, 1),
            (2, 1, 10),
        ];
        let mut solver = MatchingSolver::new(3, 2);
        for candidate in candidates {
            assert_eq!(
                solver.add_candidate(candidate.0, candidate.1, candidate.2),
                Ok(())
            );
        }

        let result = solver.solve();
        assert!(result.is_ok());
        assert_eq!(
            result.unwrap(),
            (6, vec![(0, Some((1, 1))), (1, None), (2, Some((0, 5)))])
        );
    }

    #[test]
    fn many_candidate_get_matching_4() {
        // 複数のリクエストが指定されたときにちゃんと最小のコストのリクエストが選ばれていること
        // 擬似乱数生成 (Xorshift)
        let mut seed = 123456789u64;
        let mut rand = || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            seed
        };

        let left_count = 200;
        let right_count = 150;
        let edge_count = 400;
        let mut edges = Vec::with_capacity(edge_count);

        let mut solver = MatchingSolver::new(left_count, right_count);

        // 重複を避けるためのセット
        let mut seen = HashSet::new();

        while edges.len() < edge_count {
            let u = (rand() % left_count as u64) as usize;
            let v = (rand() % right_count as u64) as usize;
            if !seen.contains(&(u, v)) {
                seen.insert((u, v));
                // 重みは 1 ~ 255
                let w = (rand() % 255 + 1) as u8;
                edges.push((u, v, w));
                assert_eq!(solver.add_candidate(u, v, w), Ok(()));
            }
        }

        let result = solver.solve();
        assert!(result.is_ok());
        assert_eq!(
            result.as_ref().unwrap().0,
            result
                .as_ref()
                .ok()
                .map(|res| {
                    res.1
                        .iter()
                        .filter_map(|i| i.1.map(|j| j.1 as TotalCost))
                        .sum::<TotalCost>() as TotalCost
                })
                .unwrap()
        );
        assert_eq!(
            result
                .as_ref()
                .unwrap()
                .1
                .iter()
                .filter(|t| t.1.is_some())
                .count(),
            134
        );
    }

    #[test]
    fn many_candidate_get_matching_5() {
        // 複数のリクエストが指定されたときにちゃんと最小のコストのリクエストが選ばれていること
        // 完全二部グラフバージョン
        let n: usize = 5;
        let mut solver = MatchingSolver::new(n, n);
        for i in 0..n {
            for j in 0..n {
                assert_eq!(solver.add_candidate(i, j, 1 + (i + j) as u8), Ok(()));
            }
        }

        let result = solver.solve();
        assert!(result.is_ok());
        assert_eq!(
            result.unwrap(),
            (
                25,
                vec![
                    (0, Some((0, 1))),
                    (1, Some((1, 3))),
                    (2, Some((2, 5))),
                    (3, Some((3, 7))),
                    (4, Some((4, 9)))
                ]
            )
        );
    }

    #[test]
    fn all_same_cost() {
        // コストが等しいときでも最大マッチングになっていること by AI
        let l = 10;
        let r = 10;
        let mut solver = MatchingSolver::new(l, r);

        for i in 0..l {
            for j in 0..r {
                solver.add_candidate(i, j, 5).unwrap();
            }
        }

        let result = solver.solve().unwrap();
        assert_eq!(result.1.iter().filter(|x| x.1.is_some()).count(), 10);
        assert_eq!(result.0, 50);
    }

    #[test]
    fn left_much_larger_than_right() {
        // Left / Right のサイズ差が極端なケースでmin(L, R) 制約が守られているか、片側が余るときの復元安全性が確保されているかの確認 by AI
        let mut solver = MatchingSolver::new(100, 3);
        for i in 0..100 {
            solver.add_candidate(i, i % 3, 1).unwrap();
        }

        let result = solver.solve().unwrap();
        assert_eq!(result.1.iter().filter(|x| x.1.is_some()).count(), 3);
    }

    #[test]
    fn greedy_trap_case() {
        // 局所最適を選ぶと失敗するケースに陥るようなGreedyアルゴリズムになっていないこと by AI
        // L0-R0 を取ると L1 が詰む
        let mut solver = MatchingSolver::new(2, 2);
        solver.add_candidate(0, 0, 1).unwrap();
        solver.add_candidate(0, 1, 100).unwrap();
        solver.add_candidate(1, 0, 2).unwrap();

        let result = solver.solve().unwrap();
        assert_eq!(result.0, 102);
        assert_eq!(result.1, vec![(0, Some((1, 100))), (1, Some((0, 2)))]);
    }

    #[test]
    fn invalid_candidate_rejected() {
        // add_candidateの境界条件でエラーとなること by AI
        let mut solver = MatchingSolver::new(2, 2);
        assert!(solver.add_candidate(2, 0, 1).is_err());
        assert!(solver.add_candidate(0, 2, 1).is_err());
        assert!(solver.add_candidate(0, 0, 0).is_err());
    }

    #[test]
    fn rematching_via_residual_graph() {
        // 残余グラフ + 逆辺が正しく機能していることで、一度マッチしたRightを別のLeftが奪うケースでも問題ないこと by AI
        let mut solver = MatchingSolver::new(3, 2);
        solver.add_candidate(0, 0, 10).unwrap();
        solver.add_candidate(1, 0, 1).unwrap();
        solver.add_candidate(2, 1, 1).unwrap();

        let result = solver.solve().unwrap();
        assert_eq!(result.0, 2);
        assert_eq!(
            result.1,
            vec![(0, None), (1, Some((0, 1))), (2, Some((1, 1)))]
        );
    }

    #[test]
    fn sparse_large_graph() {
        // ある程度の大規模でもメモリエラーにならず動くこと by AI
        let mut solver = MatchingSolver::new(300, 300);
        for i in 0..300 {
            solver.add_candidate(i, i, 1).unwrap();
        }

        let result = solver.solve().unwrap();
        assert_eq!(result.0, 300);
    }
}
