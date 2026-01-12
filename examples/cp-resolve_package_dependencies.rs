//! パッケージの依存関係を解決するように、指定したバージョンの制約を満たしつつ適当なインストール順に並べたパッケージの一覧を求める

use study_combinatorial_optimization::cp::resolve_package_dependencies::{
    Constraint, PackageInfo, PackageName, PackageRegistry, Version, resolve_deps_by_pub_grub,
    resolve_deps_by_recursive_backtracking,
};

fn main() {
    resolve_by_recursive_backtracking();
    println!("\n-------------------------\n");
    resolve_by_pub_grub();
}

fn resolve_by_recursive_backtracking() {
    println!("resolve dependencies by Recursive Backtracking");
    // A -> B (^1.0.0), C (^1.0.0)
    // B -> D (^1.0.0)
    // C -> D (^1.1.0)
    // D (1.0.0, 1.1.0, 1.2.0)
    // Expected: A, B, C, D(1.2.0 or 1.1.0)
    // D 1.0.0 is invalid because of C's constraint.

    let mut registry = PackageRegistry::new();
    let v100 = Version::new(1, 0, 0);
    let v110 = Version::new(1, 1, 0);
    let v120 = Version::new(1, 2, 0);

    // D
    registry.add_package("D", v100, PackageInfo::new(vec![]));
    registry.add_package("D", v110, PackageInfo::new(vec![]));
    registry.add_package("D", v120, PackageInfo::new(vec![]));

    // B -> D ^1.0.0
    registry.add_package(
        "B",
        v100,
        PackageInfo::new(vec![("D".to_string(), "^1.0.0".parse().unwrap())]),
    );

    // C -> D ^1.1.0
    registry.add_package(
        "C",
        v100,
        PackageInfo::new(vec![("D".to_string(), "^1.1.0".parse().unwrap())]),
    );

    // A -> B, C
    registry.add_package(
        "A",
        v100,
        PackageInfo::new(vec![
            ("B".to_string(), "^1.0.0".parse().unwrap()),
            ("C".to_string(), "^1.0.0".parse().unwrap()),
        ]),
    );

    let root_version: Version = "0.1.0".parse().unwrap();
    let root_deps: Vec<(PackageName, Constraint)> =
        vec![("A".to_string(), "^1.0.0".parse().unwrap())];
    let result =
        resolve_deps_by_recursive_backtracking(&registry, "ROOT", root_version, &root_deps);

    let graph = result.unwrap();

    // TODO グラフ描画
    println!("{:#?}", graph);
}

fn resolve_by_pub_grub() {
    println!("resolve dependencies by PubGrub");
    // A -> B (^1.0.0), C (^1.0.0)
    // B -> D (^1.0.0)
    // C -> D (^1.1.0)
    // D (1.0.0, 1.1.0, 1.2.0)
    // Expected: A, B, C, D(1.2.0 or 1.1.0)
    // D 1.0.0 is invalid because of C's constraint.

    let mut registry = PackageRegistry::new();
    let v100 = Version::new(1, 0, 0);
    let v110 = Version::new(1, 1, 0);
    let v120 = Version::new(1, 2, 0);

    // D
    registry.add_package("D", v100, PackageInfo::new(vec![]));
    registry.add_package("D", v110, PackageInfo::new(vec![]));
    registry.add_package("D", v120, PackageInfo::new(vec![]));

    // B -> D ^1.0.0
    registry.add_package(
        "B",
        v100,
        PackageInfo::new(vec![("D".to_string(), "^1.0.0".parse().unwrap())]),
    );

    // C -> D ^1.1.0
    registry.add_package(
        "C",
        v100,
        PackageInfo::new(vec![("D".to_string(), "^1.1.0".parse().unwrap())]),
    );

    // A -> B, C
    registry.add_package(
        "A",
        v100,
        PackageInfo::new(vec![
            ("B".to_string(), "^1.0.0".parse().unwrap()),
            ("C".to_string(), "^1.0.0".parse().unwrap()),
        ]),
    );

    let root_version: Version = "0.1.0".parse().unwrap();
    let root_deps: Vec<(PackageName, Constraint)> =
        vec![("A".to_string(), "^1.0.0".parse().unwrap())];
    let result = resolve_deps_by_pub_grub(&registry, "ROOT", root_version, &root_deps);

    let graph = result.unwrap();

    // TODO グラフ描画
    println!("{:#?}", graph);
}
