//! 非負整数辺重み二部グラフの重み最小辺数最大マッチングを数理最適化問題として求める。
//! cp/matching_by_pdpd.rsと同じ問題を解くことを想定している。

use crate::cp::matching_by_pdpd::{Cost, NodeId, TotalCost};
use good_lp::{
    Expression, Solution, SolverModel, Variable, constraint, microlp, variable, variables,
};
use std::collections::BTreeMap;

pub struct BipartiteMatchingLpSolver {
    candidates: BTreeMap<(NodeId, NodeId), Cost>,
}

impl BipartiteMatchingLpSolver {
    pub fn new() -> Self {
        BipartiteMatchingLpSolver {
            candidates: BTreeMap::new(),
        }
    }

    /// マッチしてほしい組を登録
    /// 同じ(left_index, right_index)を持つ辺を張ることは想定しておらず、後勝ちで登録する。
    pub fn add_candidate(
        &mut self,
        left_index: NodeId,
        right_index: NodeId,
        cost: Cost,
    ) -> Result<(), String> {
        if cost == 0 {
            return Err("0 cost is invalid".to_string());
        }

        self.candidates.insert((left_index, right_index), cost);

        Ok(())
    }

    /// マッチしたleftとrightの組の一覧を返す
    pub fn solve(self) -> (TotalCost, Vec<(NodeId, Option<(NodeId, Cost)>)>) {
        // 変数定義
        let mut vars = variables!();
        // キーは(LeftのノードID(0-origin), RightのノードID(0-origin))
        let mut candidate_edges: BTreeMap<(usize, usize), (Variable, Cost)> = BTreeMap::new();
        for ((left_index, right_index), candidate) in self.candidates.iter() {
            let v = vars.add(variable().binary());
            candidate_edges.insert((*left_index, *right_index), (v, *candidate));
        }

        //
        // 目的関数
        //
        let mut total_candidate_cost: TotalCost = 0;
        let mut unweighted_candidate_objective: Expression = 0.0.into();
        let mut weighted_candidate_objective: Expression = 0.0.into();
        for ((_left_index, _right_index), (v, cost)) in candidate_edges.iter() {
            total_candidate_cost = total_candidate_cost + (*cost as TotalCost);
            unweighted_candidate_objective = unweighted_candidate_objective + *v;
            weighted_candidate_objective = weighted_candidate_objective + (*cost as f64) * *v;
        }

        // 目的関数は、辺を区別なく選ぶこと（total_candidate_costを使う項）として最大化することで最大マッチングを考えつつ、
        // コストで重み付けした項を最小化することでコスト最小化を行う。
        // ここでtotal_candidate_cost + 1.0となっているのは、希望が一つだけだった時に選ばないことが最大になってしまうため。
        let objective: Expression = (total_candidate_cost as f64 + 1.0)
            * unweighted_candidate_objective
            - weighted_candidate_objective;
        let mut problem = vars.maximise(&objective).using(microlp);

        //
        // 制約条件
        //

        // 各左の集合の要素は最大で一つの右の集合の要素に割り当てられる
        let mut left_subjects: BTreeMap<usize, Vec<&Variable>> = BTreeMap::new();
        let mut right_subjects: BTreeMap<usize, Vec<&Variable>> = BTreeMap::new();
        for ((left_index, right_index), (v, _cost)) in candidate_edges.iter() {
            left_subjects
                .entry(*left_index)
                .or_insert_with(|| Vec::new())
                .push(v);
            right_subjects
                .entry(*right_index)
                .or_insert_with(|| Vec::new())
                .push(v);
        }
        // 各右の集合の要素は最大で一つの左の集合の要素に割り当てられる
        for (_k, expr_vec) in left_subjects.into_iter() {
            problem = problem.with(constraint!(
                expr_vec
                    .into_iter()
                    .fold(0.0.into(), |acc: Expression, b| acc + *b)
                    <= 1
            ));
        }
        // 各右の集合の要素は最大で一つの左の集合の要素に割り当てられる
        for (_k, expr_vec) in right_subjects.into_iter() {
            problem = problem.with(constraint!(
                expr_vec
                    .into_iter()
                    .fold(0.0.into(), |acc: Expression, b| acc + *b)
                    <= 1
            ));
        }

        // 解く
        let solution = problem.solve().expect("solve failed");

        let mut total_cost: TotalCost = 0;
        let mut matching: BTreeMap<NodeId, Option<(NodeId, Cost)>> = BTreeMap::new();

        for ((left_index, right_index), (v, cost)) in candidate_edges.iter() {
            if solution.value(*v) == 1.0 {
                // マッチした場合
                total_cost += *cost as TotalCost;
                matching.insert(*left_index, Some((*right_index, *cost)));
            } else {
                // マッチしなかった場合
                // マッチしているときの値を上書きしないように注意して保持させておく
                if !matching.contains_key(left_index) {
                    matching.insert(*left_index, None);
                }
            }
        }

        (total_cost, matching.into_iter().collect())
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use std::collections::HashSet;

    #[test]
    fn get_matching_when_nothing_node() {
        // ノードなしでもマッチング結果の取得ができること
        let solver = BipartiteMatchingLpSolver::new();
        let result = solver.solve();
        assert_eq!(result, (0, vec![]));
    }

    #[test]
    fn get_matching_when_only_left_node() {
        // Leftノードだけでもマッチング結果の取得ができること
        let solver = BipartiteMatchingLpSolver::new();
        let result = solver.solve();
        assert_eq!(
            result,
            (
                0,
                // グラフの方式と違い、あらかじめノードを確保する方式ではないので割当先がないなら希望もなく、割り当てるものがない
                vec![]
            )
        );
    }

    #[test]
    fn get_matching_when_only_right_node() {
        // Rightノードだけでもマッチング結果の取得ができること
        let solver = BipartiteMatchingLpSolver::new();
        let result = solver.solve();
        assert_eq!(result, (0, vec![]));
    }

    #[test]
    fn get_matching_when_only_node_nothing_edge() {
        // LeftノードとRightノードだけでもマッチング結果の取得ができること
        let solver = BipartiteMatchingLpSolver::new();
        let result = solver.solve();
        assert_eq!(
            result,
            (
                0,
                // グラフの方式と違い、あらかじめノードを確保する方式ではないので割当の希望がないなら割り当てるものはない
                vec![]
            )
        );
    }

    #[test]
    fn only_left_node_candidate_one() {
        // Leftノードがいくつかあるが、一つのLeftノードが一つだけリクエストを出しているとき、ちゃんと選ばれていること
        let mut solver = BipartiteMatchingLpSolver::new();
        assert_eq!(solver.add_candidate(1, 3, 7), Ok(()));
        let result = solver.solve();
        // グラフの方式と違い、あらかじめノードを確保する方式ではないので希望をもとに求められるノードしか取得できない
        assert_eq!(result, (7, vec![(1, Some((3, 7)))]));
    }

    #[test]
    fn only_left_node_candidate_many() {
        // Leftノードがいくつかあるが、一つのLeftノードいくつかリクエストを出しているとき、ちゃんと最小のコストのリクエストが選ばれていること
        let mut solver = BipartiteMatchingLpSolver::new();
        assert_eq!(solver.add_candidate(1, 1, 2), Ok(()));
        assert_eq!(solver.add_candidate(1, 3, 7), Ok(()));
        assert_eq!(solver.add_candidate(1, 4, 6), Ok(()));
        let result = solver.solve();
        // グラフの方式と違い、あらかじめノードを確保する方式ではないので希望をもとに求められるノードしか取得できない
        assert_eq!(result, (2, vec![(1, Some((1, 2)))]));
    }

    #[test]
    fn only_right_node_candidate_many() {
        // Leftノードがいくつかあるが、一つのLeftノードいくつかリクエストを出しているとき、ちゃんと最小のコストのリクエストが選ばれていること
        let mut solver = BipartiteMatchingLpSolver::new();
        assert_eq!(solver.add_candidate(1, 2, 3), Ok(()));
        assert_eq!(solver.add_candidate(2, 2, 7), Ok(()));
        assert_eq!(solver.add_candidate(3, 2, 6), Ok(()));
        let result = solver.solve();
        // グラフの方式と違い、あらかじめノードを確保する方式ではないので希望をもとに求められるノードしか取得できない
        assert_eq!(result, (3, vec![(1, Some((2, 3))), (2, None), (3, None)]));
    }

    #[test]
    fn many_candidate_get_matching_1() {
        // 複数のリクエストが指定されたときにちゃんと最小のコストのリクエストが選ばれていること
        let candidates: Vec<(NodeId, NodeId, Cost)> =
            vec![(0, 0, 10), (0, 2, 2), (1, 0, 5), (1, 2, 8)];
        let mut solver = BipartiteMatchingLpSolver::new();
        for candidate in candidates {
            assert_eq!(
                solver.add_candidate(candidate.0, candidate.1, candidate.2),
                Ok(())
            );
        }

        let result = solver.solve();
        // グラフの方式と違い、あらかじめノードを確保する方式ではないので希望をもとに求められるノードしか取得できない
        assert_eq!(result, (7, vec![(0, Some((2, 2))), (1, Some((0, 5)))]));
    }

    #[test]
    fn many_candidate_get_matching_2() {
        // 複数のリクエストが指定されたときにちゃんと最小のコストのリクエストが選ばれていること
        let candidates: Vec<(NodeId, NodeId, Cost)> =
            vec![(0, 0, 10), (0, 1, 9), (1, 0, 9), (1, 1, 1)];
        let mut solver = BipartiteMatchingLpSolver::new();
        for candidate in candidates {
            assert_eq!(
                solver.add_candidate(candidate.0, candidate.1, candidate.2),
                Ok(())
            );
        }

        let result = solver.solve();
        assert_eq!(result, (11, vec![(0, Some((0, 10))), (1, Some((1, 1)))]));
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
        let mut solver = BipartiteMatchingLpSolver::new();
        for candidate in candidates {
            assert_eq!(
                solver.add_candidate(candidate.0, candidate.1, candidate.2),
                Ok(())
            );
        }

        let result = solver.solve();
        // グラフの方式と違い、あらかじめノードを確保する方式ではないので希望をもとに求められるノードしか取得できない。
        // また、グラフの方式で求めたときとも一致するとは限らない。実際、今の実装では一致していない。
        assert_eq!(
            result,
            (6, vec![(0, Some((0, 5))), (1, Some((1, 1))), (2, None)])
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

        let mut solver = BipartiteMatchingLpSolver::new();

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
        assert_eq!(
            result.0,
            result
                .1
                .iter()
                .filter_map(|i| i.1.map(|j| j.1 as TotalCost))
                .sum::<TotalCost>() as TotalCost
        );
        assert_eq!(result.1.iter().filter(|t| t.1.is_some()).count(), 134);
    }

    #[test]
    fn many_candidate_get_matching_5() {
        // 複数のリクエストが指定されたときにちゃんと最小のコストのリクエストが選ばれていること
        // 完全二部グラフバージョン
        let n: usize = 5;
        let mut solver = BipartiteMatchingLpSolver::new();
        for i in 0..n {
            for j in 0..n {
                assert_eq!(solver.add_candidate(i, j, 1 + (i + j) as u8), Ok(()));
            }
        }

        let result = solver.solve();
        // また、グラフの方式で求めたときとも一致するとは限らない。実際、今の実装では一致していない。
        assert_eq!(
            result,
            (
                25,
                vec![
                    (0, Some((2, 3))),
                    (1, Some((4, 6))),
                    (2, Some((0, 3))),
                    (3, Some((1, 5))),
                    (4, Some((3, 8)))
                ]
            )
        );
    }

    #[test]
    fn all_same_cost() {
        // コストが等しいときでも最大マッチングになっていること by AI
        let l = 10;
        let r = 10;
        let mut solver = BipartiteMatchingLpSolver::new();

        for i in 0..l {
            for j in 0..r {
                assert_eq!(solver.add_candidate(i, j, 5), Ok(()));
            }
        }

        let result = solver.solve();
        assert_eq!(result.1.iter().filter(|x| x.1.is_some()).count(), 10);
        assert_eq!(result.0, 50);
    }

    #[test]
    fn left_much_larger_than_right() {
        // Left / Right のサイズ差が極端なケースでmin(L, R) 制約が守られているか、片側が余るときの復元安全性が確保されているかの確認 by AI
        let mut solver = BipartiteMatchingLpSolver::new();
        for i in 0..100 {
            assert_eq!(solver.add_candidate(i, i % 3, 1), Ok(()));
        }

        let result = solver.solve();
        assert_eq!(result.1.iter().filter(|x| x.1.is_some()).count(), 3);
    }

    #[test]
    fn greedy_trap_case() {
        // 局所最適を選ぶと失敗するケースに陥るようなGreedyアルゴリズムになっていないこと by AI
        // L0-R0 を取ると L1 が詰む
        let mut solver = BipartiteMatchingLpSolver::new();
        assert_eq!(solver.add_candidate(0, 0, 1), Ok(()));
        assert_eq!(solver.add_candidate(0, 1, 100), Ok(()));
        assert_eq!(solver.add_candidate(1, 0, 2), Ok(()));

        let result = solver.solve();
        assert_eq!(result.0, 102);
        assert_eq!(result.1, vec![(0, Some((1, 100))), (1, Some((0, 2)))]);
    }

    #[test]
    fn invalid_candidate_rejected() {
        // CPのほうのadd_candidateの境界条件でエラーとなるときはLPのほうではエラーにならないこと
        let mut solver = BipartiteMatchingLpSolver::new();
        // グラフの方式の時のようにあらかじめ割り当てたいノードを用意するわけではないので、ノードに関するエラーにはならない。
        // out of cost value range (0<cost<=Cost::MAX)
        assert!(solver.add_candidate(0, 0, 0).is_err());
    }

    #[test]
    fn rematching_via_residual_graph() {
        // 残余グラフ + 逆辺が正しく機能していることで、一度マッチしたRightを別のLeftが奪うケースでも問題ないこと by AI
        let mut solver = BipartiteMatchingLpSolver::new();
        assert_eq!(solver.add_candidate(0, 0, 10), Ok(()));
        assert_eq!(solver.add_candidate(1, 0, 1), Ok(()));
        assert_eq!(solver.add_candidate(2, 1, 1), Ok(()));

        let result = solver.solve();
        assert_eq!(result.0, 2);
        assert_eq!(
            result.1,
            vec![(0, None), (1, Some((0, 1))), (2, Some((1, 1)))]
        );
    }

    #[test]
    fn sparse_large_graph() {
        // ある程度の大規模でもメモリエラーにならず動くこと by AI
        let mut solver = BipartiteMatchingLpSolver::new();
        for i in 0..300 {
            assert_eq!(solver.add_candidate(i, i, 1), Ok(()));
        }

        let result = solver.solve();
        assert_eq!(result.0, 300);
    }
}
