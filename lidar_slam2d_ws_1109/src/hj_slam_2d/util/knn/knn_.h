#ifndef KNN_H
#define KNN_H

#include <cstdlib>
#include "KDTreeTableAdaptor.h"

void cpp_knn(const float* points, const size_t npts, const size_t dim, 
			const float* queries, const size_t nqueries,
			const size_t K, long* indices);

void cpp_knn_omp(const float* points, const long long npts, const long long dim,
			const float* queries, const long long nqueries,
			const long long K, long* indices);

void cpp_knn_omp_kdtree(const float* points, const long long npts, const long long dim,
	const float* queries, const long long nqueries,
	const long long K, long* indices, KDTreeTableAdaptor< float, float> &mat_index, std::vector<float> &dists);

void cpp_knn_batch(const float* batch_data, const size_t batch_size, const size_t npts, const size_t dim,
			const float* queries, const size_t nqueries,
			const size_t K, long* batch_indices);

void cpp_knn_batch_omp(const float* batch_data, const long long batch_size, const long long npts, const long long dim,
				const float* queries, const long long nqueries,
				const long long K, long* batch_indices);

void cpp_knn_batch_distance_pick(const float* batch_data, const size_t batch_size, const size_t npts, const size_t dim, 
				float* queries, const size_t nqueries,
				const size_t K, long* batch_indices);

void cpp_knn_batch_distance_pick_omp(const float* batch_data, const long long batch_size, const long long npts, const long long dim,
				float* batch_queries, const long long nqueries,
				const long long K, long* batch_indices);

#endif