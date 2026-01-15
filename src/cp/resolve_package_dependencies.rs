//! パッケージの依存関係を解決するように、指定したバージョンの制約を満たしつつ適当なインストール順に並べたパッケージの一覧を求めるためのモジュール

use std::cmp::Ordering;
use std::collections::{BTreeMap, HashMap, HashSet, btree_map};
use std::fmt;
use std::fmt::Formatter;
use std::ops::RangeBounds;
use std::str::FromStr;

/// パッケージのバージョン。
/// セマンティックバージョンを想定。
#[derive(Debug, Clone, PartialEq, Eq, Hash, Copy)]
pub struct Version {
    major: u32,
    minor: u32,
    patch: u32,
}

impl Version {
    pub fn new(major: u32, minor: u32, patch: u32) -> Self {
        Version {
            major,
            minor,
            patch,
        }
    }

    /// そのバージョンにおいて指定された戦略で許される
    fn upper_bound(&self, strategy: Strategy) -> Version {
        match strategy {
            Strategy::Caret => {
                if self.major > 0 {
                    // メジャーが1以上なら、次のメジャーまで互換
                    // つまり、^1.2.3 -> 1.2.3以上2.0.0未満
                    Version {
                        major: self.major + 1,
                        minor: 0,
                        patch: 0,
                    }
                } else if self.minor > 0 {
                    // 0.x.x なら、次のマイナーまで互換
                    // つまり、^0.2.3 -> 0.2.3以上0.3.0未満
                    Version {
                        major: 0,
                        minor: self.minor + 1,
                        patch: 0,
                    }
                } else {
                    // 0.0.x なら、次のパッチまで互換（実質固定）
                    // つまり、^0.0.3 -> 0.0.3以上0.0.4未満
                    Version {
                        major: 0,
                        minor: 0,
                        patch: self.patch + 1,
                    }
                }
            }
            Strategy::Tilde => {
                // メジャーを固定して、次のマイナーまで互換
                // つまり、~1.2.3 -> 1.2.3以上1.3.0未満
                Version {
                    major: self.major,
                    minor: self.minor + 1,
                    patch: 0,
                }
            }
            Strategy::Exact => {
                Version {
                    major: self.major,
                    minor: self.minor,
                    // 自分以上、次のパッチ未満。つまり、一致。
                    patch: self.patch + 1,
                }
            }
        }
    }
}

impl FromStr for Version {
    type Err = String;
    /// 文字列 "1.2.3"や"1.2", "1" から Version を生成。
    /// 未指定部分は0で埋める。
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        let parts: Vec<&str> = s.split('.').collect();
        let parts_len = parts.len();
        if parts_len > 3 || parts_len == 0 {
            return Err(format!("Invalid Version format: {}", s));
        }
        let major = parts[0].parse().map_err(|_| "Invalid major")?;
        let minor = if parts_len < 2 {
            0
        } else {
            parts[1].parse().map_err(|_| "Invalid minor")?
        };
        let patch = if parts_len < 3 {
            0
        } else {
            parts[2].parse().map_err(|_| "Invalid patch")?
        };

        Ok(Version::new(major, minor, patch))
    }
}

impl Ord for Version {
    fn cmp(&self, other: &Self) -> Ordering {
        self.major
            .cmp(&other.major)
            .then(self.minor.cmp(&other.minor))
            .then(self.patch.cmp(&other.patch))
    }
}

impl PartialOrd for Version {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl fmt::Display for Version {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{}.{}.{}", self.major, self.minor, self.patch)
    }
}

/// バージョン制約の戦略。
/// 仕様に関しては、[Constraint::is_satisfied]を参照。
#[derive(Debug, Clone, PartialEq, Eq, Hash, Copy)]
pub enum Strategy {
    /// ^表記。互換性があるバージョンを指定するのに利用する。
    Caret,
    /// ~表記。メジャーバージョンとマイナーバージョンまで固定して、パッチだけの更新を許す場合に利用する。
    Tilde,
    /// =表記。指定バージョンと完全一致しているバージョンを指定するのに利用する。
    Exact,
}

impl Strategy {
    fn to_symbol(&self) -> &'static str {
        match self {
            Strategy::Caret => "^",
            Strategy::Tilde => "~",
            Strategy::Exact => "=",
        }
    }
}

impl fmt::Display for Strategy {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        write!(f, "{}", &self.to_symbol())
    }
}

/// バージョン制約
#[derive(Debug, Clone, PartialEq, Eq, Hash, Copy)]
pub struct Constraint {
    strategy: Strategy,
    version: Version,
}

impl Constraint {
    pub fn new(strategy: Strategy, version: Version) -> Self {
        Constraint { strategy, version }
    }

    /// 指定されたバージョンが制約を満たすならtrueを返す。
    ///
    /// [Strategy::Caret]なら、次の条件のように制約バージョン以上かつ「破壊的変更」が発生するバージョン未満。
    /// - ^1.2.3 -> 1.2.3以上2.0.0未満
    /// - ^0.2.3 -> 0.2.3以上0.3.0未満
    /// - ^0.0.3 -> 0.0.3以上0.0.4未満
    ///
    /// [Strategy::Tilde]なら、次の条件のように制約バージョン以上かつ次のマイナーバージョン未満。
    /// - ~1.2.3 -> 1.2.3以上1.3.0未満
    /// - ~0.2.3 -> 0.2.3以上0.3.0未満
    /// - ~0.0.3 -> 0.0.3以上0.1.0未満
    ///
    /// [Strategy::Exact]なら、一致したバージョンのみ。
    pub fn is_satisfied(&self, version: Version) -> bool {
        match self.strategy {
            // Exact戦略のときもversion >= self.version && version < self.version.upper_bound(self.strategy)とかけるが、
            // version == self.versionと同じ意味なので簡略化
            Strategy::Exact => version == self.version,
            _ => version >= self.version && version < self.version.upper_bound(self.strategy),
        }
    }
}

impl FromStr for Constraint {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        if s.is_empty() {
            return Err("Constraint and Version is empty".to_string());
        }
        let (strategy, version_part) = if s.starts_with('^') {
            (Strategy::Caret, &s[1..])
        } else if s.starts_with('~') {
            (Strategy::Tilde, &s[1..])
        } else if s.starts_with('=') {
            (Strategy::Exact, &s[1..])
        } else {
            // デフォルトはキャレット戦略
            (Strategy::Caret, s)
        };

        // バージョン文字列のパース
        let version = Version::from_str(version_part).map_err(|e| {
            format!(
                "Invalid Constraint for strategy {} with parse version error : {}",
                &strategy, e
            )
        })?;
        Ok(Constraint { strategy, version })
    }
}

impl fmt::Display for Constraint {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        write!(f, "{}{}", self.strategy.to_symbol(), self.version)
    }
}

/// パッケージ名
pub type PackageName = String;

/// パッケージの依存先
pub type PackageDependence = (PackageName, Constraint);

/// 特定バージョンのパッケージの情報
#[derive(Debug, Clone)]
pub struct PackageInfo {
    deps: Vec<PackageDependence>,
}

impl PackageInfo {
    pub fn new(deps: Vec<PackageDependence>) -> Self {
        Self { deps }
    }

    /// 依存先の情報を取得
    pub fn get_deps(&self) -> &Vec<PackageDependence> {
        &self.deps
    }
}

/// パッケージのバージョンごとの情報を保持するエントリーポイント
#[derive(Debug, Clone)]
pub struct PackageEntry {
    // 自分のバージョンごとに、依存先のパッケージ名と制約条件を持つ
    versions: BTreeMap<Version, PackageInfo>,
}

impl PackageEntry {
    pub fn new() -> Self {
        Self {
            versions: BTreeMap::new(),
        }
    }

    /// 指定された依存関係（deps）を持つものとして、versionというバージョンを記録する
    pub fn add_version(&mut self, version: Version, info: PackageInfo) {
        self.versions.insert(version, info);
    }

    /// 指定された範囲のバージョン情報を依存先と一緒に取得する
    pub fn get_versions_with_deps(
        &'_ self,
        range: impl RangeBounds<Version>,
    ) -> btree_map::Range<'_, Version, PackageInfo> {
        self.versions.range(range)
    }

    /// 指定された範囲のバージョンを満たすこのパッケージが保持しているバージョン一覧を取得する
    pub fn get_versions(
        &self,
        range: impl RangeBounds<Version>,
    ) -> impl DoubleEndedIterator<Item = &Version> {
        self.versions.range(range).map(|(k, _)| k)
    }

    /// 指定されたバージョンのパッケージに関する情報を取得
    pub fn get_package_info(&self, version: &Version) -> Option<&PackageInfo> {
        self.versions.get(version)
    }
}

/// パッケージのレジストリ。
/// 利用可能なパッケージとそのバージョンがすべて含まれている。
#[derive(Debug, Clone)]
pub struct PackageRegistry {
    packages: HashMap<PackageName, PackageEntry>,
}

impl PackageRegistry {
    pub fn new() -> Self {
        Self {
            packages: HashMap::new(),
        }
    }

    /// パッケージ情報を登録
    /// すでに存在していた場合は上書き
    pub fn add_package(&mut self, name: &str, version: Version, info: PackageInfo) {
        match self.packages.get_mut(name) {
            None => {
                let mut package_entry = PackageEntry::new();
                package_entry.add_version(version, info);
                self.packages.insert(name.to_string(), package_entry);
            }
            Some(package_entry) => {
                package_entry.add_version(version, info);
            }
        }
    }

    /// 指定されたパッケージの指定されたバージョンの依存情報を取得
    fn get_deps(&self, name: &str, version: &Version) -> Option<&Vec<PackageDependence>> {
        self.packages
            .get(name)
            .map(|entry| entry.get_package_info(version).map(|info| info.get_deps()))
            .flatten()
    }
}

// 依存関係解決後の各パッケージの情報
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ResolvedPackageInfo {
    /// 確定したバージョン
    version: Version,
    /// このバージョンが依存するパッケージ名のリスト
    /// (解決済みなので制約情報は不要で、名前だけでリンクを表現できる)
    dep_packages: Vec<PackageName>,
}

impl ResolvedPackageInfo {
    fn new(version: Version, deps: Vec<PackageName>) -> Self {
        Self {
            version,
            dep_packages: deps,
        }
    }

    /// 確定したバージョンを取得
    pub fn get_version(&self) -> &Version {
        &self.version
    }

    /// このバージョンが依存するパッケージ名のリストを取得。
    /// 解決済みを表す[ResolvedGraph]からパッケージ名から辿ることができるので、解決済みなので制約情報はない。
    pub fn get_dependencies(&self) -> &Vec<PackageName> {
        &self.dep_packages
    }
}

/// 依存関係解決の結果（グラフ構造）。
/// パッケージ名をキーとし、解決された情報を値とする。
pub type ResolvedGraph = HashMap<PackageName, ResolvedPackageInfo>;

/// 再帰的バックトラッキング (Recursive Backtracking)という方法でバージョン依存を解決する。
///
/// 実装してみた感想としては、エラー文言の制御が難しいと感じた。
/// 再帰的な処理であることと依存関係の解決状況を調整しながら他の可能性も探索する必要があることで適当な文言を選ぶことが難しくなっているみたい。
pub fn resolve_deps_by_recursive_backtracking(
    registry: &PackageRegistry,
    root_package_name: &str,
    root_package_version: Version,
    root_deps: &Vec<(PackageName, Constraint)>,
) -> Result<ResolvedGraph, String> {
    let mut assigned: HashMap<PackageName, Version> = HashMap::new();
    for (name, _) in root_deps {
        if name == root_package_name {
            return Err("Invalid self dependency.".to_string());
        }
    }

    // 循環参照チェック
    fn has_transitive_dependency(
        registry: &PackageRegistry,
        assigned: &HashMap<PackageName, Version>,
        start: &PackageName,
        target: &PackageName,
        root_package_name: &str,
        root_deps: &Vec<(PackageName, Constraint)>,
    ) -> bool {
        let mut visited = HashSet::new();
        let mut stack = vec![start.clone()];
        visited.insert(start.clone());

        while let Some(current) = stack.pop() {
            let deps_opt = if current == root_package_name {
                Some(root_deps)
            } else {
                assigned
                    .get(&current)
                    .and_then(|v| registry.get_deps(&current, v))
            };

            if let Some(deps) = deps_opt {
                for (dep_name, _) in deps {
                    if dep_name == target {
                        return true;
                    }
                    if assigned.contains_key(dep_name) && !visited.contains(dep_name) {
                        visited.insert(dep_name.clone());
                        stack.push(dep_name.clone());
                    }
                }
            }
        }
        false
    }

    // 深さ優先的にパッケージの依存関係を解決できるか再帰的に確認し、最終的に解決できそうならそのペッケージ情報を返す。
    fn recursive(
        registry: &PackageRegistry,
        root_package_name: &str,
        root_deps: &Vec<(PackageName, Constraint)>,
        assigned: &mut HashMap<PackageName, Version>,
    ) -> Result<ResolvedGraph, String> {
        // 1. 未解決のパッケージ（必要とされているがassignedに含まれていないパッケージ）を探す
        let mut next_package_name: Option<PackageName> = None;
        for (name, _) in root_deps {
            if !assigned.contains_key(name) {
                next_package_name = Some(name.clone());
                break;
            }
        }

        // 確定済みのパッケージからの依存を確認
        if next_package_name.is_none() {
            for (package_name, version) in assigned.iter() {
                if let Some(deps) = registry.get_deps(package_name, version) {
                    for (dep_name, _) in deps {
                        if !assigned.contains_key(dep_name) {
                            next_package_name = Some(dep_name.clone());
                            break;
                        }
                    }
                    if next_package_name.is_some() {
                        break;
                    }
                } else {
                    return Err(format!(
                        "Package info missing for {} {}",
                        package_name, version
                    ));
                }
            }
        }

        // ベースケース：全ての依存関係が解決済み
        if next_package_name.is_none() {
            // 結果グラフの構築
            let mut graph = HashMap::new();
            for (name, version) in assigned.iter() {
                let deps = registry
                    .get_deps(name, version)
                    .ok_or_else(|| format!("Package info missing for {} {}", name, version))?;

                let dep_names = deps.iter().map(|(n, _)| n.clone()).collect();

                graph.insert(name.clone(), ResolvedPackageInfo::new(*version, dep_names));
            }
            return Ok(graph);
        }

        // ベースケースで解決できなかった場合。
        // つまり、まだ解決しなければならない依存関係が存在する場合。
        let package_name = next_package_name.unwrap();

        // 2. 探そうとしているパッケージの制約として、ルートからの制約と現時点で確定している制約を収集する
        let mut constraints: Vec<(PackageName, &Constraint)> = Vec::new();
        // ルートからの制約
        for (name, constraint) in root_deps {
            if name == &package_name {
                constraints.push((root_package_name.to_string(), constraint));
            }
        }
        // 確定済みパッケージからの制約
        for (assigned_package_name, assigned_package_version) in assigned.iter() {
            if let Some(assigned_package_deps) =
                registry.get_deps(assigned_package_name, assigned_package_version)
            {
                for (assigned_package_dep_name, assigned_package_dep_constraint) in
                    assigned_package_deps
                {
                    if assigned_package_dep_name == &package_name {
                        constraints.push((
                            assigned_package_name.to_string(),
                            assigned_package_dep_constraint,
                        ));
                    }
                }
            }
        }

        // 3. レジストリから候補となるバージョンを取得し、降順（新しい順）に試す
        match registry.packages.get(&package_name) {
            None => {
                if package_name == root_package_name {
                    // ルートパッケージは通常記録されていないので、依存先として検出されたここでは分けてエラーを返す。
                    return Err(format!("Cycle detected with {}.", root_package_name));
                }

                Err(format!("Package entry of {} not found.", package_name))
            }
            Some(entry) => {
                let mut last_error = None;
                let mut rejection_reasons = Vec::new();

                // 全バージョンを取得して降順にソートして、最新から順に確認する
                for (candidate_version, candidate_package_info) in
                    entry.get_versions_with_deps(..).rev()
                {
                    // すべての制約を満たすかチェック
                    let unsatisfied = constraints
                        .iter()
                        .find(|(_, c)| !c.is_satisfied(*candidate_version));

                    if unsatisfied.is_none() {
                        // 依存先の整合性チェック（既に解決済みのパッケージに対して、この候補が持つ制約が矛盾しないか）
                        let mut consistent = true;
                        for (dep_name, dep_constraint) in candidate_package_info.get_deps() {
                            // 自分自身への依存の場合はそもそもの依存先の指定がおかしいということでエラーにする
                            if dep_name == &package_name {
                                return Err(format!(
                                    "Invalid self dependency for {} package.",
                                    package_name
                                ));
                            }
                            // 既に解決済みのパッケージに対して、この候補が持つ制約が矛盾しないか
                            else if let Some(assigned_dev_version) = assigned.get(dep_name) {
                                // 循環参照チェック
                                if has_transitive_dependency(
                                    registry,
                                    assigned,
                                    dep_name,
                                    &package_name,
                                    root_package_name,
                                    root_deps,
                                ) {
                                    consistent = false;
                                    rejection_reasons
                                        .push(format!("Cycle detected with {}", dep_name));
                                    break;
                                }
                                if !dep_constraint.is_satisfied(*assigned_dev_version) {
                                    consistent = false;
                                    break;
                                }
                            }
                        }

                        if consistent {
                            // 依存先が既存の解決済みパッケージのバージョンと制約を満たすので、仮決定して深さ優先的に再帰
                            assigned.insert(package_name.clone(), *candidate_version);
                            match recursive(registry, root_package_name, root_deps, assigned) {
                                Ok(graph) => return Ok(graph),
                                Err(e) => {
                                    // 失敗した場合はエラーを記録してバックトラック
                                    rejection_reasons.push(format!(
                                        "Version {} failed to resolve dependencies: {}",
                                        candidate_version, e
                                    ));
                                    // 新しいバージョンから順に試しているため、最初のエラー（最新バージョンでの失敗）を
                                    // 優先して残す方が、ユーザーにとって「なぜ最新が入らないのか」を知る上で有益な場合が多い。
                                    if last_error.is_none() {
                                        last_error = Some(e);
                                    }
                                    assigned.remove(&package_name);
                                }
                            }
                        }
                    } else {
                        let (unsatisfied_package, unsatisfied_constraint) = unsatisfied.unwrap();
                        rejection_reasons.push(format!(
                            "Version {} rejected by constraint {} from {}",
                            candidate_version, unsatisfied_constraint, unsatisfied_package
                        ));
                    }
                }

                // すべての候補を試しても解決できなかった場合、
                // 最後に発生したエラー（最も深い場所でのエラーの可能性が高い）があればそれを返す
                if let Some(e) = last_error {
                    return Err(e);
                }

                Err(format!(
                    "Could not resolve dependencies for {}.\nConstraints: {:?}\nAvailable versions: {:?}\nRejection reasons:\n{}",
                    package_name,
                    constraints,
                    entry.get_versions(..).collect::<Vec<_>>(),
                    rejection_reasons.join("\n")
                ))
            }
        }
    }

    recursive(registry, root_package_name, root_deps, &mut assigned).map(|mut graph| {
        graph.insert(
            root_package_name.to_string(),
            ResolvedPackageInfo::new(
                root_package_version,
                root_deps.iter().map(|(name, _)| name.clone()).collect(),
            ),
        );
        graph
    })
}

/// DPLL(Davis-Putnam-Logemann-Loveland)という方法でバージョン依存を解決する
pub fn resolve_deps_by_dpll(
    registry: &PackageRegistry,
    root_package_name: &str,
    root_package_version: Version,
    root_deps: &Vec<(PackageName, Constraint)>,
) -> Result<ResolvedGraph, String> {
    // cf. https://qiita.com/tmk0308/items/34c0bbf09ae0633b3c5e
    // cf. https://ja.wikipedia.org/wiki/DPLL%E3%82%A2%E3%83%AB%E3%82%B4%E3%83%AA%E3%82%BA%E3%83%A0
    // cf. https://zenn.dev/semiexp/books/e559bbd5ab00a2/viewer/6f1878
    // cf. https://github.com/motok822/pubgrub-cpp/blob/main/src/dpll_solver.cpp

    todo!()
}

/// PubGrub (Next-Generation Version Solving)という方法でバージョン依存を解決する
pub fn resolve_deps_by_pub_grub(
    registry: &PackageRegistry,
    root_package_name: &str,
    root_package_version: Version,
    root_deps: &Vec<(PackageName, Constraint)>,
) -> Result<ResolvedGraph, String> {
    // cf. https://qiita.com/tmk0308/items/34c0bbf09ae0633b3c5e
    // cf. https://crates.io/crates/pubgrub
    // cf. https://docs.rs/pubgrub/0.3.0/src/pubgrub/solver.rs.html#108-248

    todo!()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn version_parse() {
        assert_eq!("1.2.3".parse::<Version>(), Ok(Version::new(1, 2, 3)));
        assert_eq!("1.2".parse::<Version>(), Ok(Version::new(1, 2, 0)));
        assert_eq!("1".parse::<Version>(), Ok(Version::new(1, 0, 0)));
        assert!("invalid".parse::<Version>().is_err());
    }

    #[test]
    fn version_ord() {
        let v1 = Version::new(1, 0, 0);
        let v2 = Version::new(1, 1, 0);
        assert!(v1 < v2);
        assert!(v2 > v1);
    }

    #[test]
    fn constraint_parse() {
        let c: Constraint = "^1.2.3".parse().unwrap();
        assert_eq!(c.strategy, Strategy::Caret);
        assert_eq!(c.version, Version::new(1, 2, 3));

        let c: Constraint = "~1.2.3".parse().unwrap();
        assert_eq!(c.strategy, Strategy::Tilde);
        assert_eq!(c.version, Version::new(1, 2, 3));

        let c: Constraint = "1.2.3".parse().unwrap();
        assert_eq!(c.strategy, Strategy::Caret);
        assert_eq!(c.version, Version::new(1, 2, 3));
    }

    #[test]
    fn is_satisfied_caret() {
        // ^1.2.3 -> >=1.2.3 <2.0.0
        let c: Constraint = "^1.2.3".parse().unwrap();
        assert!(c.is_satisfied(Version::new(1, 2, 3)));
        assert!(c.is_satisfied(Version::new(1, 99, 99)));
        assert!(!c.is_satisfied(Version::new(2, 0, 0)));
        assert!(!c.is_satisfied(Version::new(1, 2, 2)));

        // ^0.2.3 -> >=0.2.3 <0.3.0
        let c: Constraint = "^0.2.3".parse().unwrap();
        assert!(c.is_satisfied(Version::new(0, 2, 3)));
        assert!(c.is_satisfied(Version::new(0, 2, 99)));
        assert!(!c.is_satisfied(Version::new(0, 3, 0)));

        // ^0.0.3 -> >=0.0.3 <0.0.4
        let c: Constraint = "^0.0.3".parse().unwrap();
        assert!(c.is_satisfied(Version::new(0, 0, 3)));
        assert!(!c.is_satisfied(Version::new(0, 0, 4)));
    }

    #[test]
    fn is_satisfied_tilde() {
        // ~1.2.3 -> >=1.2.3 <1.3.0
        let c: Constraint = "~1.2.3".parse().unwrap();
        assert!(c.is_satisfied(Version::new(1, 2, 3)));
        assert!(c.is_satisfied(Version::new(1, 2, 99)));
        assert!(!c.is_satisfied(Version::new(1, 3, 0)));
    }

    #[test]
    fn is_satisfied_exact() {
        let c: Constraint = "=1.2.3".parse().unwrap();
        assert!(c.is_satisfied(Version::new(1, 2, 3)));
        assert!(!c.is_satisfied(Version::new(1, 2, 4)));
        assert!(!c.is_satisfied(Version::new(1, 2, 2)));
    }

    #[test]
    fn constraint_parse_error() {
        assert!("".parse::<Constraint>().is_err());
        assert!("^1.a.2".parse::<Constraint>().is_err());
    }

    #[test]
    fn package_entry_range_search() {
        let mut entry = PackageEntry::new();
        let v100 = Version::new(1, 0, 0);
        let v110 = Version::new(1, 1, 0);
        let v120 = Version::new(1, 2, 0);
        let v200 = Version::new(2, 0, 0);

        entry.add_version(v100, PackageInfo::new(vec![]));
        entry.add_version(v110, PackageInfo::new(vec![]));
        entry.add_version(v120, PackageInfo::new(vec![]));
        entry.add_version(v200, PackageInfo::new(vec![]));

        // 全範囲
        assert_eq!(entry.get_versions(..).count(), 4);

        // 指定範囲 (1.1.0 <= v < 2.0.0)
        let range: Vec<&Version> = entry.get_versions(v110..v200).collect();
        assert_eq!(range, vec![&v110, &v120]);

        // 指定範囲 (v >= 1.2.0)
        let from: Vec<&Version> = entry.get_versions(v120..).collect();
        assert_eq!(from, vec![&v120, &v200]);
    }

    #[test]
    fn package_registry_lookup() {
        let mut registry = PackageRegistry::new();
        let v1 = Version::new(1, 0, 0);
        let dep = ("OtherPkg".to_string(), "^1.0.0".parse().unwrap());

        registry.add_package("MyPkg", v1, PackageInfo::new(vec![dep.clone()]));

        // 存在するパッケージとバージョン
        let deps = registry.get_deps("MyPkg", &v1);
        assert!(deps.is_some());
        assert_eq!(deps.unwrap()[0], dep);

        // 存在しないパッケージ
        assert!(registry.get_deps("Unknown", &v1).is_none());

        // 存在するパッケージだがバージョンがない
        assert!(registry.get_deps("MyPkg", &Version::new(2, 0, 0)).is_none());
    }

    #[test]
    fn resolve_diamond_dependency() {
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

        assert!(result.is_ok());
        let graph = result.unwrap();
        println!("{:#?}", graph);

        // Dは1.1.0以上2.0.0未満である必要がある (Cの制約)
        let d_ver = graph.get("D").unwrap().get_version();
        assert!(*d_ver >= v110);
        assert!(*d_ver < Version::new(2, 0, 0));
    }

    #[test]
    fn resolve_fail_missing_package() {
        let mut registry = PackageRegistry::new();
        let v1 = Version::new(1, 0, 0);
        // A -> B (B is missing)
        registry.add_package(
            "A",
            v1,
            PackageInfo::new(vec![("B".to_string(), "^1.0.0".parse().unwrap())]),
        );

        let root_version: Version = "0.1.0".parse().unwrap();
        let root_deps = vec![("A".to_string(), "^1.0.0".parse().unwrap())];
        let result =
            resolve_deps_by_recursive_backtracking(&registry, "ROOT", root_version, &root_deps);

        assert!(result.is_err());
        let err = result.err().unwrap();
        assert!(err.contains("Package entry of B not found"));
    }

    #[test]
    fn resolve_fail_no_matching_version() {
        let mut registry = PackageRegistry::new();
        let v1 = Version::new(1, 0, 0);
        // A -> B ^2.0.0
        registry.add_package(
            "A",
            v1,
            PackageInfo::new(vec![("B".to_string(), "^2.0.0".parse().unwrap())]),
        );
        // B 1.0.0 only
        registry.add_package("B", v1, PackageInfo::new(vec![]));

        let root_version: Version = "0.1.0".parse().unwrap();
        let root_deps = vec![("A".to_string(), "^1.0.0".parse().unwrap())];
        let result =
            resolve_deps_by_recursive_backtracking(&registry, "ROOT", root_version, &root_deps);

        assert!(result.is_err());
        // エラーメッセージにはBの解決失敗が含まれるはず
        let err = result.err().unwrap();
        assert!(err.contains("Could not resolve dependencies for B"));
    }

    #[test]
    fn resolve_fail_conflict() {
        let mut registry = PackageRegistry::new();
        let v1 = Version::new(1, 0, 0);

        // Root -> A, B
        // A -> C ^1.0.0
        // B -> C ^2.0.0
        // C has 1.0.0 and 2.0.0

        registry.add_package("C", Version::new(1, 0, 0), PackageInfo::new(vec![]));
        registry.add_package("C", Version::new(2, 0, 0), PackageInfo::new(vec![]));

        registry.add_package(
            "A",
            v1,
            PackageInfo::new(vec![("C".to_string(), "^1.0.0".parse().unwrap())]),
        );
        registry.add_package(
            "B",
            v1,
            PackageInfo::new(vec![("C".to_string(), "^2.0.0".parse().unwrap())]),
        );

        let root_version: Version = "0.1.0".parse().unwrap();
        let root_deps = vec![
            ("A".to_string(), "^1.0.0".parse().unwrap()),
            ("B".to_string(), "^1.0.0".parse().unwrap()),
        ];
        let result =
            resolve_deps_by_recursive_backtracking(&registry, "ROOT", root_version, &root_deps);

        assert!(result.is_err());
    }

    #[test]
    fn resolve_fail_circular_dependency() {
        let mut registry = PackageRegistry::new();
        let v1 = Version::new(1, 0, 0);

        // A -> B ^1.0.0
        registry.add_package(
            "A",
            v1,
            PackageInfo::new(vec![("B".to_string(), "^1.0.0".parse().unwrap())]),
        );
        // B -> A ^1.0.0
        registry.add_package(
            "B",
            v1,
            PackageInfo::new(vec![("A".to_string(), "^1.0.0".parse().unwrap())]),
        );

        let root_version: Version = "0.1.0".parse().unwrap();
        let root_deps = vec![("A".to_string(), "^1.0.0".parse().unwrap())];
        let result =
            resolve_deps_by_recursive_backtracking(&registry, "ROOT", root_version, &root_deps);

        assert!(result.is_err());
    }

    #[test]
    fn resolve_fail_self_dependency() {
        let mut registry = PackageRegistry::new();
        let v1 = Version::new(1, 0, 0);

        // A -> A ^1.0.0
        registry.add_package(
            "A",
            v1,
            PackageInfo::new(vec![("A".to_string(), "^1.0.0".parse().unwrap())]),
        );

        let root_version: Version = "0.1.0".parse().unwrap();
        let root_deps = vec![("A".to_string(), "^1.0.0".parse().unwrap())];
        let result =
            resolve_deps_by_recursive_backtracking(&registry, "A", root_version, &root_deps);

        assert!(result.is_err());
    }

    #[test]
    fn resolve_fail_direct_self_dependency_in_package() {
        let mut registry = PackageRegistry::new();
        let v1 = Version::new(1, 0, 0);

        // A -> A ^1.0.0
        registry.add_package(
            "A",
            v1,
            PackageInfo::new(vec![("A".to_string(), "^1.0.0".parse().unwrap())]),
        );

        let root_version: Version = "0.1.0".parse().unwrap();
        let root_deps = vec![("A".to_string(), "^1.0.0".parse().unwrap())];
        let result =
            resolve_deps_by_recursive_backtracking(&registry, "ROOT", root_version, &root_deps);

        assert!(result.is_err());
    }

    #[test]
    fn resolve_fail_indirect_self_dependency_in_package() {
        let mut registry = PackageRegistry::new();
        let v1 = Version::new(1, 0, 0);

        // A -> B ^1.0.0
        registry.add_package(
            "A",
            v1,
            PackageInfo::new(vec![("B".to_string(), "^1.0.0".parse().unwrap())]),
        );

        // B -> B ^1.0.0
        registry.add_package(
            "B",
            v1,
            PackageInfo::new(vec![("B".to_string(), "^1.0.0".parse().unwrap())]),
        );

        let root_version: Version = "0.1.0".parse().unwrap();
        let root_deps = vec![("A".to_string(), "^1.0.0".parse().unwrap())];
        let result =
            resolve_deps_by_recursive_backtracking(&registry, "ROOT", root_version, &root_deps);

        assert!(result.is_err());
    }

    #[test]
    fn resolve_backtracking_complex() {
        // Root -> A, B
        // A -> C ^1.0.0
        // B -> D ^1.0.0
        // C 1.1.0 -> D ^2.0.0
        // C 1.0.0 -> D ^1.0.0
        // D 2.0.0, 1.0.0
        //
        // 単純な貪欲法でCの最新(1.1.0)を選ぶと、Dは2.0.0になる必要があるが、
        // BがD ^1.0.0を要求しているため競合する。
        // バックトラックしてC 1.0.0を選ぶことで、D 1.0.0となり、Bの要求も満たせる。

        let mut registry = PackageRegistry::new();
        let v100 = Version::new(1, 0, 0);
        let v110 = Version::new(1, 1, 0);
        let v200 = Version::new(2, 0, 0);

        registry.add_package("D", v100, PackageInfo::new(vec![]));
        registry.add_package("D", v200, PackageInfo::new(vec![]));

        registry.add_package(
            "C",
            v110,
            PackageInfo::new(vec![("D".to_string(), "^2.0.0".parse().unwrap())]),
        );
        registry.add_package(
            "C",
            v100,
            PackageInfo::new(vec![("D".to_string(), "^1.0.0".parse().unwrap())]),
        );

        registry.add_package(
            "A",
            v100,
            PackageInfo::new(vec![("C".to_string(), "^1.0.0".parse().unwrap())]),
        );
        registry.add_package(
            "B",
            v100,
            PackageInfo::new(vec![("D".to_string(), "^1.0.0".parse().unwrap())]),
        );

        let root_version: Version = "0.1.0".parse().unwrap();
        let root_deps = vec![
            ("A".to_string(), "^1.0.0".parse().unwrap()),
            ("B".to_string(), "^1.0.0".parse().unwrap()),
        ];
        let result =
            resolve_deps_by_recursive_backtracking(&registry, "ROOT", root_version, &root_deps);

        assert!(result.is_ok());
        let graph = result.unwrap();

        // Cは1.0.0が選ばれているはず
        assert_eq!(graph.get("C").unwrap().version, v100);
        // Dは1.0.0が選ばれているはず
        assert_eq!(graph.get("D").unwrap().version, v100);
    }

    #[test]
    fn resolve_shared_dependency_narrowing() {
        // Root -> A, B
        // A -> Shared ^1.0.0
        // B -> Shared ^1.5.0
        // Shared 1.0.0, 1.4.0, 1.6.0
        // 両方の制約を満たす 1.6.0 が選ばれるべき

        let mut registry = PackageRegistry::new();
        let v100 = Version::new(1, 0, 0);

        registry.add_package("Shared", Version::new(1, 0, 0), PackageInfo::new(vec![]));
        registry.add_package("Shared", Version::new(1, 4, 0), PackageInfo::new(vec![]));
        registry.add_package("Shared", Version::new(1, 6, 0), PackageInfo::new(vec![]));

        registry.add_package(
            "A",
            v100,
            PackageInfo::new(vec![("Shared".to_string(), "^1.0.0".parse().unwrap())]),
        );
        registry.add_package(
            "B",
            v100,
            PackageInfo::new(vec![("Shared".to_string(), "^1.5.0".parse().unwrap())]),
        );

        let root_version: Version = "0.1.0".parse().unwrap();
        let root_deps = vec![
            ("A".to_string(), "^1.0.0".parse().unwrap()),
            ("B".to_string(), "^1.0.0".parse().unwrap()),
        ];
        let result =
            resolve_deps_by_recursive_backtracking(&registry, "ROOT", root_version, &root_deps);

        assert!(result.is_ok());
        let graph = result.unwrap();
        assert_eq!(graph.get("Shared").unwrap().version, Version::new(1, 6, 0));
    }

    #[test]
    fn resolve_fail_indirect_circular_dependency() {
        // A -> B -> C -> A
        let mut registry = PackageRegistry::new();
        let v1 = Version::new(1, 0, 0);

        registry.add_package(
            "A",
            v1,
            PackageInfo::new(vec![("B".to_string(), "^1.0.0".parse().unwrap())]),
        );
        registry.add_package(
            "B",
            v1,
            PackageInfo::new(vec![("C".to_string(), "^1.0.0".parse().unwrap())]),
        );
        registry.add_package(
            "C",
            v1,
            PackageInfo::new(vec![("A".to_string(), "^1.0.0".parse().unwrap())]),
        );

        let root_version: Version = "0.1.0".parse().unwrap();
        let root_deps = vec![("A".to_string(), "^1.0.0".parse().unwrap())];
        let result =
            resolve_deps_by_recursive_backtracking(&registry, "ROOT", root_version, &root_deps);

        assert!(result.is_err());
        let err = result.err().unwrap();
        assert!(err.contains("Cycle detected"));
    }

    #[test]
    fn resolve_deep_dependency_chain() {
        // A->B->C->D->E
        let mut registry = PackageRegistry::new();
        let v1 = Version::new(1, 0, 0);

        registry.add_package("E", v1, PackageInfo::new(vec![]));
        registry.add_package(
            "D",
            v1,
            PackageInfo::new(vec![("E".to_string(), "^1.0.0".parse().unwrap())]),
        );
        registry.add_package(
            "C",
            v1,
            PackageInfo::new(vec![("D".to_string(), "^1.0.0".parse().unwrap())]),
        );
        registry.add_package(
            "B",
            v1,
            PackageInfo::new(vec![("C".to_string(), "^1.0.0".parse().unwrap())]),
        );
        registry.add_package(
            "A",
            v1,
            PackageInfo::new(vec![("B".to_string(), "^1.0.0".parse().unwrap())]),
        );

        let root_version: Version = "0.1.0".parse().unwrap();
        let root_deps = vec![("A".to_string(), "^1.0.0".parse().unwrap())];
        let result =
            resolve_deps_by_recursive_backtracking(&registry, "ROOT", root_version, &root_deps);

        assert!(result.is_ok());
        let graph = result.unwrap();
        assert_eq!(graph.len(), 6); // Root + A,B,C,D,E
    }

    #[test]
    fn resolve_exact_strategy_chain() {
        // A -> B =1.0.0
        // B 1.0.0 -> C =1.0.0
        // B 1.1.0 -> C =1.1.0
        // C 1.0.0
        // C 1.1.0

        let mut registry = PackageRegistry::new();
        let v100 = Version::new(1, 0, 0);
        let v110 = Version::new(1, 1, 0);

        registry.add_package("C", v100, PackageInfo::new(vec![]));
        registry.add_package("C", v110, PackageInfo::new(vec![]));

        registry.add_package(
            "B",
            v100,
            PackageInfo::new(vec![("C".to_string(), "=1.0.0".parse().unwrap())]),
        );
        registry.add_package(
            "B",
            v110,
            PackageInfo::new(vec![("C".to_string(), "=1.1.0".parse().unwrap())]),
        );

        registry.add_package(
            "A",
            v100,
            PackageInfo::new(vec![("B".to_string(), "=1.0.0".parse().unwrap())]),
        );

        let root_version: Version = "0.1.0".parse().unwrap();
        let root_deps = vec![("A".to_string(), "^1.0.0".parse().unwrap())];

        let result =
            resolve_deps_by_recursive_backtracking(&registry, "ROOT", root_version, &root_deps);

        assert!(result.is_ok());
        let graph = result.unwrap();
        assert_eq!(graph.get("B").unwrap().version, v100);
        assert_eq!(graph.get("C").unwrap().version, v100);
    }

    #[test]
    fn resolve_fail_dependency_on_root() {
        let mut registry = PackageRegistry::new();
        let v1 = Version::new(1, 0, 0);

        // A -> ROOT ^0.1.0 (Root package)
        registry.add_package(
            "A",
            v1,
            PackageInfo::new(vec![("ROOT".to_string(), "^0.1.0".parse().unwrap())]),
        );

        let root_version: Version = "0.1.0".parse().unwrap();
        let root_deps = vec![("A".to_string(), "^1.0.0".parse().unwrap())];

        let result =
            resolve_deps_by_recursive_backtracking(&registry, "ROOT", root_version, &root_deps);

        // 循環参照として検出され、候補が却下されるためエラーになる
        assert!(result.is_err());
        let err = result.err().unwrap();
        assert!(err.contains("Cycle detected"));
    }

    #[test]
    fn resolve_fail_transitive_dependency_on_root() {
        let mut registry = PackageRegistry::new();
        let v1 = Version::new(1, 0, 0);

        // A -> B ^1.0.0
        registry.add_package(
            "A",
            v1,
            PackageInfo::new(vec![("B".to_string(), "^1.0.0".parse().unwrap())]),
        );

        // B -> ROOT ^0.1.0 (Root package)
        registry.add_package(
            "B",
            v1,
            PackageInfo::new(vec![("ROOT".to_string(), "^0.1.0".parse().unwrap())]),
        );

        let root_version: Version = "0.1.0".parse().unwrap();
        let root_deps = vec![("A".to_string(), "^1.0.0".parse().unwrap())];

        let result =
            resolve_deps_by_recursive_backtracking(&registry, "ROOT", root_version, &root_deps);

        // 循環参照として検出され、候補が却下されるためエラーになる
        assert!(result.is_err());
        let err = result.err().unwrap();
        assert!(err.contains("Cycle detected"));
    }
}
