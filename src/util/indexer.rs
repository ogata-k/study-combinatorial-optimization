//! 座標圧縮 (Coordinate Compression)もしくはインターニング (Interning)と呼ばれる機能を提供するモジュール

use std::collections::{BTreeMap, HashMap};
use std::hash::Hash;

/// インデックスの値に圧縮したり展開したりする機能を提供するトレイト
pub trait Indexer<T> {
    /// 利用するデータを圧縮しつつ初期化
    fn new<I>(all_data: I) -> Self
    where
        I: IntoIterator<Item = T>;

    /// 圧縮したアイテムが空ならtrue
    fn is_empty(&self) -> bool;

    /// 圧縮したアイテム数を取得
    fn len(&self) -> usize;

    /// 圧縮結果を取得
    fn to_index(&self, value: &T) -> Option<usize>;

    /// 圧縮前の値を取得
    fn to_value(&self, index: usize) -> Option<&T>;
}
/// 【Interning】出現順不同でIDを振る（高速）
/// 順序関係を気にせず、単にユニークなIDが欲しい場合に利用
#[derive(Debug, Clone)]
pub struct HashIndexer<T: Hash + Eq + Clone> {
    value_to_index: HashMap<T, usize>,
    index_to_value: Vec<T>,
}

impl<T: Hash + Eq + Clone> Indexer<T> for HashIndexer<T> {
    fn new<I>(all_data: I) -> Self
    where
        I: IntoIterator<Item = T>,
    {
        let mut value_to_index = HashMap::new();
        let mut index_to_value = Vec::new();
        let mut index = 0;
        for data in all_data.into_iter() {
            if value_to_index.contains_key(&data) {
                continue;
            }

            // すでに保存ができていなくてユニークが保証されているので挿入
            index_to_value.push(data.clone());
            value_to_index.insert(data, index);
            index += 1;
        }

        Self {
            value_to_index,
            index_to_value,
        }
    }

    fn is_empty(&self) -> bool {
        self.index_to_value.is_empty()
    }

    fn len(&self) -> usize {
        self.index_to_value.len()
    }

    fn to_index(&self, value: &T) -> Option<usize> {
        self.value_to_index.get(value).copied()
    }

    fn to_value(&self, index: usize) -> Option<&T> {
        self.index_to_value.get(index)
    }
}

/// 【Interning】出現順不同でIDを振る（高速）
/// 順序関係を気にせず、単にユニークなIDが欲しい場合に利用
#[derive(Debug, Clone)]
pub struct BTreeIndexer<T: Ord + Eq + Clone> {
    value_to_index: BTreeMap<T, usize>,
    index_to_value: Vec<T>,
}

impl<T: Ord + Eq + Clone> Indexer<T> for BTreeIndexer<T> {
    fn new<I>(all_data: I) -> Self
    where
        I: IntoIterator<Item = T>,
    {
        let mut value_to_index = BTreeMap::new();
        let mut index_to_value = Vec::new();
        let mut index = 0;
        for data in all_data.into_iter() {
            if value_to_index.contains_key(&data) {
                continue;
            }

            // すでに保存ができていなくてユニークが保証されているので挿入
            index_to_value.push(data.clone());
            value_to_index.insert(data, index);
            index += 1;
        }

        Self {
            value_to_index,
            index_to_value,
        }
    }

    fn is_empty(&self) -> bool {
        self.index_to_value.is_empty()
    }

    fn len(&self) -> usize {
        self.index_to_value.len()
    }

    fn to_index(&self, value: &T) -> Option<usize> {
        self.value_to_index.get(value).copied()
    }

    fn to_value(&self, index: usize) -> Option<&T> {
        self.index_to_value.get(index)
    }
}
/// 【Coordinate Compression】値の昇順にIDを振る（座標圧縮）
/// 大小関係を維持したい場合に利用 (A < B なら index(A) < index(B))
#[derive(Debug, Clone)]
pub struct OrderedIndexer<T: Ord> {
    // 値自体がインデックスに対応する（index 0 の値は values[0]）
    values: Vec<T>,
}

impl<T: Ord> Indexer<T> for OrderedIndexer<T> {
    fn new<I>(all_data: I) -> Self
    where
        I: IntoIterator<Item = T>,
    {
        let mut values: Vec<T> = all_data.into_iter().collect();
        // ソートして重複排除
        values.sort();
        values.dedup();

        Self { values }
    }

    fn is_empty(&self) -> bool {
        self.values.is_empty()
    }

    fn len(&self) -> usize {
        self.values.len()
    }

    fn to_index(&self, value: &T) -> Option<usize> {
        // ソート済みなので二分探索が使える (O(log N))
        self.values.binary_search(value).ok()
    }

    fn to_value(&self, index: usize) -> Option<&T> {
        self.values.get(index)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_hash_indexer() {
        // 入力順にIDが振られることを確認
        let data = vec!["banana", "apple", "cherry", "apple"];
        let indexer = HashIndexer::new(data);

        assert_eq!(indexer.len(), 3);
        assert!(!indexer.is_empty());

        // 重複した "apple" は最初のインデックス 1 になるはず
        assert_eq!(indexer.to_index(&"banana"), Some(0));
        assert_eq!(indexer.to_index(&"apple"), Some(1));
        assert_eq!(indexer.to_index(&"cherry"), Some(2));
        assert_eq!(indexer.to_index(&"durian"), None);

        // IDから値の復元
        assert_eq!(indexer.to_value(1), Some(&"apple"));
        assert_eq!(indexer.to_value(99), None);
    }

    #[test]
    fn test_empty_hash_indexer() {
        let data: Vec<i32> = vec![];
        let indexer = HashIndexer::new(data);
        assert!(indexer.is_empty());
        assert_eq!(indexer.len(), 0);
        assert_eq!(indexer.to_index(&10), None);
        assert_eq!(indexer.to_value(0), None);
    }

    #[test]
    fn test_btree_indexer() {
        // Hashが使えない型（想定）でも同様に動作することを確認
        let data = vec![100, 10, 50, 10];
        let indexer = BTreeIndexer::new(data);

        assert_eq!(indexer.len(), 3);
        // BTreeIndexerも挿入順（重複除外）でIDが振られる実装
        assert_eq!(indexer.to_index(&100), Some(0));
        assert_eq!(indexer.to_index(&10), Some(1));
        assert_eq!(indexer.to_index(&50), Some(2));
    }

    #[test]
    fn test_empty_btree_indexer() {
        let data: Vec<i32> = vec![];
        let indexer = BTreeIndexer::new(data);
        assert!(indexer.is_empty());
        assert_eq!(indexer.len(), 0);
        assert_eq!(indexer.to_index(&10), None);
        assert_eq!(indexer.to_value(0), None);
    }

    #[test]
    fn test_ordered_indexer() {
        // 値の昇順にIDが振られることを確認（座標圧縮）
        let data = vec![100, 20, 50, 20, 100];
        let indexer = OrderedIndexer::new(data);

        // ソート後のユニーク値: [20, 50, 100]
        assert_eq!(indexer.len(), 3);

        // 大小関係が維持されているか
        assert_eq!(indexer.to_index(&20), Some(0));
        assert_eq!(indexer.to_index(&50), Some(1));
        assert_eq!(indexer.to_index(&100), Some(2));

        // 逆引き
        assert_eq!(indexer.to_value(0), Some(&20));
        assert_eq!(indexer.to_value(1), Some(&50));
        assert_eq!(indexer.to_value(2), Some(&100));
    }

    #[test]
    fn test_empty_ordered_indexer() {
        let data: Vec<i32> = vec![];
        let indexer = OrderedIndexer::new(data);
        assert!(indexer.is_empty());
        assert_eq!(indexer.len(), 0);
        assert_eq!(indexer.to_index(&10), None);
        assert_eq!(indexer.to_value(0), None);
    }
}
