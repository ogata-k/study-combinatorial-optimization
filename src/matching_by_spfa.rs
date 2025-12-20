use std::cmp::{Ordering, Reverse};
use std::collections::BinaryHeap;

// ==========================================================
// 1. 型の定義と基本設定
// ==========================================================

/// ノードの識別子
type NodeId = usize;
/// グラフの容量（マッチングなら通常は 1）
type Capacity = i32;
/// 辺の重みやコスト
type Weight = i64;
/// エッジのインデックス（メモリ節約のため i32）
type EdgeId = i32;

/// 不要Weight初期化用の十分大きい値
const INF_WEIGHT: Weight = Weight::MAX / 3;

/// ダイクストラ法で「最小」の距離を優先するための構造体
#[derive(Copy, Clone, Eq, PartialEq)]
struct State {
    dist: Weight,
    v: NodeId,
}

impl Ord for State {
    fn cmp(&self, other: &Self) -> Ordering {
        self.dist.cmp(&other.dist)
    }
}

impl PartialOrd for State {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

// ==========================================================
// 2. グラフ構造とアルゴリズム
// ==========================================================

/// 前方星（Forward Star）形式によるエッジ管理
struct Edge {
    to: NodeId,
    capacity: Capacity, // 辺の容量
    weight: Weight,     // 辺の重要度
    next: EdgeId,       // 同じ始点から出る次のエッジへのインデックス
    rev: usize,         // 逆辺の edges 内での位置
}

// 実際の運用では必要ならleftノードやrightノードに渡すIdからそれぞれの実態を参照できるような対応付けを別のところで持っておく必要がある。
pub struct SuccessiveShortestPath {
    /// 費用流を作った時の開始点
    source: NodeId,
    /// 費用流を作った時の流れていく先
    sink: NodeId,
    /// 割当元のノード数
    left_count: usize,
    /// 割当先のノード数
    right_count: usize,
    /// 役割: 各頂点から出ているエッジの「最初の1本」が、edges 配列のどこにあるかを記録します。
    ///
    /// 詳細: head\[v\] を見れば、頂点 v に接続されたエッジのインデックスが分かります。もし v からエッジが出ていなければ -1 が格納されます。
    ///
    /// 空間最適化: Vec<Vec<...>> のように各頂点ごとに可変長配列を持たないため、メモリアロケーションのオーバーヘッドを大幅に削減しています。
    head: Vec<EdgeId>,
    /// 役割: グラフ上のすべてのエッジ（順方向・逆方向の両方）を1つの連続したメモリ領域に一括管理します。
    ///
    /// 詳細: エッジは Edge 構造体として格納され、各エッジは next プロパティを通じて「同じ頂点から出る次のエッジ」のインデックスを指し、擬似的な連結リストを形成します。
    ///
    /// 空間最適化: 連続したメモリ配置（データ局所性）により、CPUキャッシュに乗りやすく、探索が高速になります。
    edges: Vec<Edge>,
    /// 役割: 各頂点が持つ「ポテンシャル（電位のようなもの）」の値を保持します。
    ///
    /// 詳細: ダイクストラ法で負の辺を扱うための最も重要なデータです。エッジの重み w を、w′=w+potential(u)−potential(v) という変換によって一時的にすべて「非負（0以上）」に見せかけるために使用します。
    ///
    /// 空間最適化: 計算ごとに破棄せず、構造体内に保持して再利用します。
    potential: Vec<Weight>,
    /// 役割: ダイクストラ法の実行中に、始点 S から各頂点への「暫定的な最短距離」を記録します。
    ///
    /// 詳細: ヒープから取り出した距離と比較し、より短い経路が見つかればこの値を更新します。
    ///
    /// 空間最適化: 型を Weight（通常 i64）に固定し、頂点数分だけ確保します。
    dist: Vec<Weight>,
    /// 役割: 最短経路が見つかった後、その「道筋」を逆順に辿って復元するために使用します。
    ///
    ///     prev_v[i]: 頂点 i に到達する直前にいた頂点の番号。
    ///
    /// 詳細: ゴール（シンク）からこれらを辿ることで、どの辺の容量を減らし、どの辺にフローを流すべきかを特定します。
    prev_v: Vec<NodeId>,
    /// 役割: 最短経路が見つかった後、その「道筋」を逆順に辿って復元するために使用します。
    ///
    ///     prev_e[i]: 頂点 i に到達する際に通ったエッジのインデックス。
    ///
    /// 詳細: ゴール（シンク）からこれらを辿ることで、どの辺の容量を減らし、どの辺にフローを流すべきかを特定します。
    prev_e: Vec<usize>,
}

impl SuccessiveShortestPath {
    /// left_count: 割当元の数, right_count: 割当先の数, edges: \[(割当元のId,割当先のId,重要度)\]
    /// ただし、割当元のIdは0以上left_count未満、割当先のIdは0以上right_count未満とする。
    pub fn new(left_count: usize, right_count: usize, edges: &[(NodeId, NodeId, Weight)]) -> Self {
        let source = 0;
        let sink = left_count + right_count + 1;
        // ノードの数 = sourceノード + leftノードの数 + righノードの数 + sinkノード
        let node_count = left_count + right_count + 2;
        // 辺の数 = source->leftノードの数 + edgesの数 + rightノード->sinkの数
        let edge_count = left_count + right_count + edges.len();

        let mut ssp = Self {
            source,
            sink,
            left_count,
            right_count,
            head: vec![-1; node_count],
            edges: Vec::with_capacity(edge_count * 2),
            potential: vec![0; node_count],
            dist: vec![0; node_count],
            prev_v: vec![0; node_count],
            prev_e: vec![0; node_count],
        };

        // source->leftノード
        for i in 0..left_count {
            // 流せるけど最終結果の重みの合計値には含めたくないのでweight=0
            ssp.add_edge(source, i + 1, 1, 0);
        }

        // rightノード->sink
        for i in 0..right_count {
            // 流せるけど最終結果の重みの合計値には含めたくないのでweight=0
            ssp.add_edge(left_count + i + 1, sink, 1, 0);
        }

        // 指定された辺の追加
        for &(u, v, w) in edges {
            // 最大化のために重みを負にして最小費用流を解く。なので-w。
            ssp.add_edge(u + 1, left_count + v + 1, 1, -w);
        }

        ssp
    }

    /// 辺の追加
    pub fn add_edge(&mut self, from: NodeId, to: NodeId, cap: Capacity, weight: Weight) {
        let from_edge_id = self.edges.len();
        // 逆方向の辺は順方向の次に挿入するので+1した位置
        let to_edge_id = from_edge_id + 1;

        // 順方向
        self.edges.push(Edge {
            to,
            capacity: cap,
            weight,
            // 以前のfromから出ていた辺のEdgeIdを紐づける
            next: self.head[from],
            rev: to_edge_id,
        });
        // fromから出ている辺のEdgeIdを更新
        self.head[from] = from_edge_id as EdgeId;

        // 残余グラフの辺の追加（逆方向でコストは反転）
        self.edges.push(Edge {
            to: from,
            // まだ順方向に流していないので、流せる容量はない。
            capacity: 0,
            // 順方向の流れを打ち消すことのできるスペックを持っているため反転した値を指定
            weight: -weight,
            // 以前のtoから出ていた辺のEdgeIdを紐づける
            next: self.head[to],
            rev: from_edge_id,
        });
        // toから出ている辺のEdgeIdを更新
        self.head[to] = to_edge_id as EdgeId;
    }

    /// 最小費用流（最大マッチングのコアロジック）
    fn min_cost_flow(&mut self) -> Option<Weight> {
        // sourceからsinkに流す
        let s = self.source;
        let t = self.sink;
        // 初期値として1だけ流しておくが、合計コストは0
        let mut flow: Capacity = 1;
        let mut total_cost: Weight = 0;
        let node_count = self.head.len();

        while flow > 0 {
            // ダイクストラ法の実行中に始点のsourceから各頂点への「暫定的な最短距離」の記録を初期化
            self.dist.fill(INF_WEIGHT);
            // 開始時点ではまだ流していないので0
            self.dist[s] = 0;

            // ダイクストラ法では「まだ訪問していない頂点の中で、最短距離が最小のもの」を常に選ぶ必要がある。
            // 値が大きい値をヒープから取得できるBinaryHeapを使うと効率的な最小値の取得。
            // Reverseをはさんでいるため、比較条件が反転
            let mut heap = BinaryHeap::<Reverse<State>>::new();
            heap.push(Reverse(State { dist: 0, v: s }));

            while let Some(Reverse(State { dist, v })) = heap.pop() {
                if self.dist[v] < dist {
                    // 暫定的な値として記録している値より距離が遠いなら確認する必要はない
                    continue;
                }

                let mut edge_id = self.head[v];
                // 有効な辺が取れている限り探索
                while edge_id != -1 {
                    let e = &self.edges[edge_id as usize];
                    // ポテンシャルを用いて負の辺を非負化する
                    let reduced_weight = e.weight + self.potential[v] - self.potential[e.to];

                    // 流すことのできる辺（capacity>0）で、更新しようとしている値が更新前の値と比べて小さいなら最短路と計算の価値があるということで状態を更新
                    if e.capacity > 0 && self.dist[e.to] > self.dist[v] + reduced_weight {
                        self.dist[e.to] = self.dist[v] + reduced_weight;
                        self.prev_v[e.to] = v;
                        self.prev_e[e.to] = edge_id as usize;
                        heap.push(Reverse(State {
                            dist: self.dist[e.to],
                            v: e.to,
                        }));
                    }
                    edge_id = e.next;
                }
            }

            if self.dist[t] == INF_WEIGHT {
                return None;
            }

            // ポテンシャルの更新
            for i in 0..node_count {
                if self.dist[i] != INF_WEIGHT {
                    self.potential[i] += self.dist[i];
                }
            }

            // 流せる量を決定（マッチングなら通常1）
            let mut d = flow;
            let mut v = t;
            while v != s {
                d = d.min(self.edges[self.prev_e[v]].capacity);
                v = self.prev_v[v];
            }

            flow -= d;
            total_cost += (d as Weight) * self.potential[t];

            // 残余グラフの更新
            // 残余グラフなので、終点からたどって更新する。
            let mut v = t;
            while v != s {
                let e_idx = self.prev_e[v];
                let rev_idx = self.edges[e_idx].rev;
                self.edges[e_idx].capacity -= d;
                self.edges[rev_idx].capacity += d;
                v = self.prev_v[v];
            }
        }
        Some(total_cost)
    }

    /// マッチングされたペアのリスト (left_id, right_id) を取得する
    /// left_n: 左側ノードの数
    pub fn get_matched_pairs(&self) -> Vec<(usize, usize)> {
        let left_count = self.left_count;
        let mut pairs = Vec::new();

        // 左側ノード（ID: 1..=left_count）から出ているエッジをスキャン
        for u in 1..=left_count {
            let mut e_idx = self.head[u];
            while e_idx != -1 {
                let e = &self.edges[e_idx as usize];

                // 行き先が右側ノードの範囲内であり、かつ容量が0（＝流れた）であるものを探す
                // ※逆辺（重みが正になっているもの）は除外する
                if e.to > left_count
                    && e.to <= left_count + self.head.len() - 2
                    && e.capacity == 0
                    && e.weight < 0
                {
                    // 全体IDから元のインデックスに変換
                    // u: 1..left_count -> 0..left_count-1
                    // e.to: left_count+1.. -> 0..
                    let left_idx = u - 1;
                    let right_idx = e.to - left_count - 1;
                    pairs.push((left_idx, right_idx));
                }
                e_idx = e.next;
            }
        }
        pairs
    }
}

// ==========================================================
// 3. ラッパー関数（2部グラフ用）
// ==========================================================

pub fn solve_max_weight_matching(
    left_count: usize,
    right_count: usize,
    edges: &[(NodeId, NodeId, Weight)],
) -> (Weight, Vec<(usize, usize)>) {
    let mut ssp = SuccessiveShortestPath::new(left_count, right_count, edges);

    let mut ans = 0;
    while let Some(cost) = ssp.min_cost_flow() {
        if cost >= 0 {
            break;
        }
        // コストが反転して記録されているので判定して回答データを更新する
        ans -= cost;
    }
    let matched = ssp.get_matched_pairs();
    (ans, matched)
}
