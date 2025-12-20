use std::collections::HashSet;
use study_combinatorial_optimization::matching_by_spfa::solve_max_weight_matching;

pub fn main() {
    example_1();
    example_2();
    example_3();
    example_4();
    example_5();
    example_6();
    example_7();
    example_8();
    example_9();
    example_10();
}

fn example_1() {
    println!("--- Example 1: Basic Matching ---");
    let edges = vec![(0, 0, 10), (0, 1, 2), (1, 0, 5), (1, 1, 8)];
    let res = solve_max_weight_matching(2, 2, &edges);
    println!(
        "Total Cost: {} (Expected: 18) with matching: {:?}",
        res.0, res.1
    );
    // (0,0) weight 10 + (1,1) weight 8 = 18
}
fn example_2() {
    println!("\n--- Example 2: Better than Greedy ---");
    let edges = vec![
        (0, 0, 10), // これを選ぶと (1,1) しか選べない (10+1=11)
        (0, 1, 9),  // こちらを選ぶと (1,0) も選べる (9+9=18)
        (1, 0, 9),
        (1, 1, 1),
    ];
    let res = solve_max_weight_matching(2, 2, &edges);
    println!(
        "Total Cost: {} (Expected: 18) with matching: {:?}",
        res.0, res.1
    );
}
fn example_3() {
    println!("\n--- Example 3: Unbalanced Nodes (3 vs 2) ---");
    let edges = vec![
        (0, 0, 5),
        (1, 0, 10),
        (2, 0, 5),
        (0, 1, 1),
        (1, 1, 1),
        (2, 1, 10),
    ];
    let res = solve_max_weight_matching(3, 2, &edges);
    println!(
        "Total Cost: {} (Expected: 20) with matching: {:?}",
        res.0, res.1
    );
    // (1,0) weight 10 + (2,1) weight 10 = 20
}
fn example_4() {
    println!("\n--- Example 4: Including Negative Weights ---");
    let edges = vec![
        (0, 0, 10),
        (1, 1, -5), // このペアを作ると合計が減るので無視すべき
    ];
    let res = solve_max_weight_matching(2, 2, &edges);
    println!(
        "Total Cost: {} (Expected: 10) with matching: {:?}",
        res.0, res.1
    );
    // (0,0) のみ採用。 (1,1) は選ばない。
}
fn example_5() {
    println!("\n--- Example 5: Complex Conflict ---");
    let edges = vec![
        (0, 0, 100),
        (0, 1, 100),
        (0, 2, 100),
        (1, 1, 20),
        (1, 2, 10),
        (2, 0, 80),
        (2, 1, 70),
        (2, 2, 10),
    ];
    // 最適な組み合わせ:
    // (0,2) [100], (1,1) [20], (2,0) [80] => 200
    let res = solve_max_weight_matching(3, 3, &edges);
    println!(
        "Total Cost: {} (Expected: 200) with matching: {:?}",
        res.0, res.1
    );
}

fn example_6() {
    println!("\n--- Example 6: Random Large Case (L=50, R=100, E=600) ---");
    // 擬似乱数生成 (Xorshift)
    let mut seed = 123456789u64;
    let mut rand = || {
        seed ^= seed << 13;
        seed ^= seed >> 7;
        seed ^= seed << 17;
        seed
    };

    let left_count = 50;
    let right_count = 100;
    let edge_count = 600;
    let mut edges = Vec::with_capacity(edge_count);

    // 重複を避けるためのセット
    let mut seen = HashSet::new();

    while edges.len() < edge_count {
        let u = (rand() % left_count as u64) as usize;
        let v = (rand() % right_count as u64) as usize;
        if !seen.contains(&(u, v)) {
            seen.insert((u, v));
            // 重みは 1 ~ 1000
            let w = (rand() % 1000 + 1) as i64;
            edges.push((u, v, w));
        }
    }

    let (total_cost, pairs) = solve_max_weight_matching(left_count, right_count, &edges);

    println!(
        "Result: Total Cost = {}, Pairs count = {}\nPairs = {:?}",
        total_cost,
        pairs.len(),
        pairs
    );

    // 検証:
    // 1. マッチングの整合性 (各ノードが最大1回しか使われていないか)
    let mut used_left = vec![false; left_count];
    let mut used_right = vec![false; right_count];
    let mut calculated_weight = 0;

    for &(u, v) in &pairs {
        assert!(!used_left[u], "Left node {} used twice!", u);
        assert!(!used_right[v], "Right node {} used twice!", v);
        used_left[u] = true;
        used_right[v] = true;

        // 重みの加算 (edgesから検索)
        let w = edges
            .iter()
            .find(|&&(eu, ev, _)| eu == u && ev == v)
            .unwrap()
            .2;
        calculated_weight += w;
    }

    assert_eq!(total_cost, calculated_weight, "Weight mismatch!");
    println!("Verification Passed: Matching is valid and weight sum matches.");
}

fn example_7() {
    println!("\n--- Example 7: Zero Weights ---");
    let edges = vec![(0, 0, 10), (1, 1, 0)];
    let res = solve_max_weight_matching(2, 2, &edges);
    // Current implementation breaks on cost >= 0, so cost 0 is not included.
    // So expected weight 10, matching size 1.
    println!(
        "Total Cost: {} (Expected: 10) with matching: {:?}",
        res.0, res.1
    );
}

fn example_8() {
    println!("\n--- Example 8: Long Augmenting Path (Chain Reaction) ---");
    // L0-R0 (10), L1-R1 (10), L2-R2 (10)
    // L0-R1 (20), L1-R2 (20), L2-R3 (20) ...
    // This forces a chain of re-matches.
    let edges = vec![
        (0, 0, 10),
        (1, 1, 10),
        (2, 2, 10),
        (0, 1, 12),
        (1, 2, 12),
        (2, 3, 12),
    ];
    // Greedy might pick (0,0), (1,1), (2,2) -> Total 30.
    // Optimal: (0,1), (1,2), (2,3) -> Total 36.
    let res = solve_max_weight_matching(3, 4, &edges);
    println!(
        "Total Cost: {} (Expected: 36) with matching: {:?}",
        res.0, res.1
    );
}

fn example_9() {
    println!("\n--- Example 9: Multiple Edges between same nodes ---");
    let edges = vec![(0, 0, 10), (0, 0, 20), (0, 0, 5)];
    let res = solve_max_weight_matching(1, 1, &edges);
    println!(
        "Total Cost: {} (Expected: 20) with matching: {:?}",
        res.0, res.1
    );
}

fn example_10() {
    println!("\n--- Example 10: Dense Graph (Complete Bipartite) ---");
    let n = 5;
    let mut edges = Vec::new();
    for i in 0..n {
        for j in 0..n {
            edges.push((i, j, (i + j) as i64));
        }
    }
    // Total Cost matching in this specific matrix (w_ij = i+j).
    // Total weight = Sum(0..4) + Sum(0..4) = 10 + 10 = 20.
    let res = solve_max_weight_matching(n, n, &edges);
    println!(
        "Total Cost: {} (Expected: 20) with matching: {:?}",
        res.0, res.1
    );
}
