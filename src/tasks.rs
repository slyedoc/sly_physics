use bevy::tasks::TaskPool;

pub trait ParallelSliceEnumerateMut<T: Send>: AsMut<[T]> {
    fn par_chunk_map_enumerate_mut<F, R>(
        &mut self,
        task_pool: &TaskPool,
        chunk_size: usize,
        f: F,
    ) -> Vec<R>
    where
        F: Fn(usize, &mut [T]) -> R + Send + Sync,
        R: Send + 'static,
    {
        let slice = self.as_mut();
        let f = &f;
        task_pool.scope(|scope| {
            for (i, chunk) in slice.chunks_mut(chunk_size).enumerate() {
                scope.spawn(async move { f(i, chunk) });
            }
        })
    }
}

impl<S, T: Send> ParallelSliceEnumerateMut<T> for S where S: AsMut<[T]> {}