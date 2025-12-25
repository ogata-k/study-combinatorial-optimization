//! 入力や出力用に圧縮と展開を実行するときに使う便利なヘルパー群

use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet};
use std::hash::Hash;
use std::sync::Arc;

/// 特定の値に圧縮したり展開したりする機能を提供するトレイト
pub trait ZipUnzip<F, T> {
    type FromStorage;

    /// 利用するデータを圧縮しつつ初期化
    fn with_zip(all_data: Self::FromStorage) -> Self;

    /// 圧縮したアイテム数を取得
    fn count(&self) -> usize;

    /// 圧縮結果を取得
    fn zip(&self, from: &F) -> Option<&T>;

    /// 展開結果を取得
    fn unzip(&self, zipped: &T) -> Option<&F>;
}

/// Hashを使った圧縮展開用
#[derive(Debug, Clone)]
pub struct UsizeHashZipUnzipper<From: Hash + Eq> {
    count: usize,
    map: HashMap<Arc<From>, usize>,
    reverse_map: Vec<Arc<From>>,
}

impl<From: Hash + Eq> ZipUnzip<From, usize> for UsizeHashZipUnzipper<From> {
    type FromStorage = HashSet<From>;

    fn with_zip(all_data: Self::FromStorage) -> Self {
        let count = all_data.len();
        let mut map: HashMap<Arc<From>, usize> = HashMap::with_capacity(count);
        let mut reverse_map: Vec<Arc<From>> = Vec::with_capacity(count);

        for (i, f) in all_data.into_iter().enumerate() {
            let f = Arc::new(f);
            reverse_map.push(f.clone());
            map.insert(f, i);
        }

        Self {
            count,
            map,
            reverse_map,
        }
    }

    fn count(&self) -> usize {
        self.count
    }

    fn zip(&self, from: &From) -> Option<&usize> {
        self.map.get(from)
    }

    fn unzip(&self, zipped: &usize) -> Option<&From> {
        self.reverse_map.get(*zipped).map(|rc| &**rc)
    }
}

/// BTreeを使った圧縮展開用
#[derive(Debug, Clone)]
pub struct UsizeBTreeZipUnzipper<From: Ord + Eq> {
    count: usize,
    map: BTreeMap<Arc<From>, usize>,
    reverse_map: Vec<Arc<From>>,
}

impl<From: Ord + Eq> ZipUnzip<From, usize> for UsizeBTreeZipUnzipper<From> {
    type FromStorage = BTreeSet<From>;

    fn with_zip(all_data: Self::FromStorage) -> Self {
        let count = all_data.len();
        let mut map: BTreeMap<Arc<From>, usize> = BTreeMap::new();
        let mut reverse_map: Vec<Arc<From>> = Vec::with_capacity(count);

        for (i, f) in all_data.into_iter().enumerate() {
            let f = Arc::new(f);
            reverse_map.push(f.clone());
            map.insert(f, i);
        }

        Self {
            count,
            map,
            reverse_map,
        }
    }

    fn count(&self) -> usize {
        self.count
    }

    fn zip(&self, from: &From) -> Option<&usize> {
        self.map.get(from)
    }

    fn unzip(&self, zipped: &usize) -> Option<&From> {
        self.reverse_map.get(*zipped).map(|rc| &**rc)
    }
}
