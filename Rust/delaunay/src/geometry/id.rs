use std::{ops::{Index, IndexMut}, slice::SliceIndex};

#[derive(Debug, Default, Clone, Copy)]
pub struct LocalId<const N: usize> {
   pub i: i8 
}

impl<const N: usize> LocalId<N> {
    fn ni8() -> i8 { i8::try_from(N).unwrap() }

    fn get_id(id: i8, offset: i8) -> i8 {
        let k = offset.abs();
        let n = Self::ni8();

        if offset < 0 {
            return (id + n - (k % n)) % n
        }

        (id + k) % n
    }

    pub fn new(i: i8) -> Self { 
        assert!(i >= 0 && i < Self::ni8()); 
        Self{ i }
    }

    pub fn range() -> [Self; N] {
        std::array::from_fn(|i| {
            Self::from(i)
        })
    }

    pub fn make_invalid() -> Self { Self{i:-1}}

    pub fn is_valid(self) -> bool {
        self.i >= 0
    }

    pub fn id(self) -> usize { self.into() }

    pub fn next(self) -> Self {
        assert!(self.is_valid());
        Self::new(Self::get_id(self.i, 1))
    }

    pub fn prev(self) -> Self {
        assert!(self.is_valid());
        Self::new(Self::get_id(self.i, -1))
    }

    pub fn step(self, sign: bool) -> Self {
        if sign { self.next() }
        else { self.prev() }
    }
}

impl<const N: usize> From<LocalId<N>> for usize {
    fn from(val: LocalId<N>) -> Self {
        usize::try_from(val.i).expect("LocalId::id() failed because contained id was invalid")
    }
} 

impl<const N: usize> From<usize> for LocalId<N> {
    fn from(value: usize) -> Self {
        assert!(value < N);
        let i = i8::try_from(value).expect("usize value is greater than i8::MAX");
        Self { i } 
    }
}

impl<T, const N: usize> Index<LocalId<N>> for [T] {
    type Output = T;
    fn index(&self, idx: LocalId<N>) -> &Self::Output {
        &self[idx.id()]
    }
}

impl<T, const N: usize> IndexMut<LocalId<N>> for [T] {
    fn index_mut(&mut self, index: LocalId<N>) -> &mut Self::Output {
        &mut self[index.id()]
    }
}
