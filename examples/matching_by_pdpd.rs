use std::fmt::Display;
use std::hash::Hash;
use study_combinatorial_optimization::matching_by_pdpd::{
    Cost, MinCostMaxBipartiteMatching, TotalCost,
};
use study_combinatorial_optimization::util::indexer::{BTreeIndexer, Indexer};

fn main() {
    example_1();
}

fn example<Left: Display + Eq + Hash + Ord + Clone, Right: Display + Eq + Hash + Ord + Clone>(
    description: &str,
    candidates: Vec<(Left, Right, Cost)>,
    expect_total_cost: TotalCost,
) {
    println!("\n--- Example : {description} ---");

    let left_all_data: Vec<Left> = candidates.iter().map(|e| e.0.clone()).collect();
    let right_all_data: Vec<Right> = candidates.iter().map(|e| e.1.clone()).collect();

    let left_indexer = BTreeIndexer::new(left_all_data);
    let right_indexer = BTreeIndexer::new(right_all_data);

    let mut solver = MinCostMaxBipartiteMatching::new(left_indexer.len(), right_indexer.len());
    for candidate in candidates {
        let left_index = left_indexer.to_index(&candidate.0).unwrap();
        let right_index = right_indexer.to_index(&candidate.1).unwrap();
        solver
            .add_candidate(left_index, right_index, candidate.2)
            .expect(
                "Invalid candidate. Expect left and right node index is in range, 0 < cost <= 256",
            );
    }

    let result = solver.solve();

    println!("\n--- Solved with following result ---");
    println!("Matching Cost : {}", result.0);
    println!("Match:");
    for match_pair in result.1.iter() {
        let left = match_pair.0;
        let left_label = left_indexer.to_value(left).unwrap();
        match match_pair.1 {
            None => {
                println!("{} -- None", left_label);
            }
            Some((right, cost)) => {
                let right_label = right_indexer.to_value(right).unwrap();
                println!(" {} --- {}  with cost {}", left_label, right_label, cost);
            }
        }
    }
    println!();

    assert_eq!(expect_total_cost, result.0);
    assert_eq!(
        expect_total_cost,
        result
            .1
            .iter()
            .filter_map(|i| i.1.map(|j| j.1 as TotalCost))
            .sum::<TotalCost>() as TotalCost
    );
}

fn example_1() {
    // https://yukashun.com/complete_bipartite-minimum_weight-python/ のサンプル
    let description = "Basic Matching";
    let candidates: Vec<(u8, u8, Cost)> = vec![
        (0, 0, 3),
        (0, 1, 1),
        (0, 2, 4),
        (1, 0, 3),
        (1, 1, 2),
        (1, 2, 5),
        (2, 0, 4),
        (2, 1, 2),
        (2, 2, 3),
    ];

    let expect_total_cost = 7;
    // 実行結果は別解の最大マッチング
    example(description, candidates, expect_total_cost);
}
