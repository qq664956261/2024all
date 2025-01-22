
#include "knn_.h"
#include "nanoflann.hpp"
using namespace nanoflann;

#include <ctime>
#include <cstdlib>
#include <iostream>
#include <omp.h>

#include <vector>
#include <random>
#include <algorithm>
#include <iterator>

using namespace std;

void cpp_knn(const float *points, const size_t npts, const size_t dim,
			 const float *queries, const size_t nqueries,
			 const size_t K, long *indices)
{

	// create the kdtree
	typedef KDTreeTableAdaptor<float, float> KDTree;
	KDTree mat_index(npts, dim, points, 10);
	mat_index.index->buildIndex();

	std::vector<float> out_dists_sqr(K);
	std::vector<size_t> out_ids(K);

	// iterate over the points
	for (size_t i = 0; i < nqueries; i++)
	{
		nanoflann::KNNResultSet<float> resultSet(K);
		resultSet.init(&out_ids[0], &out_dists_sqr[0]);
		mat_index.index->findNeighbors(resultSet, &queries[i * dim], nanoflann::SearchParams(10));
		for (size_t j = 0; j < K; j++)
		{
			indices[i * K + j] = long(out_ids[j]);
		}
	}
}

void cpp_knn_omp(const float *points, const long long npts, const long long dim,
				 const float *queries, const long long nqueries,
				 const long long K, long *indices)
{

	// create the kdtree
	typedef KDTreeTableAdaptor<float, float> KDTree;
	KDTree mat_index(npts, dim, points, 10);
	mat_index.index->buildIndex();

	// iterate over the points
#pragma omp parallel for
	for (long long i = 0; i < nqueries; i++)
	{
		std::vector<size_t> out_ids(K);
		std::vector<float> out_dists_sqr(K);

		nanoflann::KNNResultSet<float> resultSet(K);
		resultSet.init(&out_ids[0], &out_dists_sqr[0]);
		mat_index.index->findNeighbors(resultSet, &queries[i * dim], nanoflann::SearchParams(10));
		for (long long j = 0; j < K; j++)
		{
			indices[i * K + j] = long(out_ids[j]);
		}
	}
}

// 点云数据，点数，点的维度，要搜索的点，要搜索的点的数，K近邻的数量，临近点的索引，kd树，临近点的距离
/*
@points: 点云数据一维数组表示
@npts: 点的数量 points.size() / 3
@dim: 点的维度 3
@queries: 表示寻找该点的邻近点
@nqueries: 上面点的数量
@K : 邻近点数
@indices: 邻近点的索引
@mat_index: kd树
@out_dists_sqr:邻近点的距离
*/
void cpp_knn_omp_kdtree(const float *points, const long long npts, const long long dim,
						const float *queries, const long long nqueries,
						const long long K, long *indices, KDTreeTableAdaptor<float, float> &mat_index, std::vector<float> &out_dists_sqr)
{

	// iterate over the points

	// #pragma omp parallel for
	for (long long i = 0; i < nqueries; i++)
	{
		std::vector<size_t> out_ids(K);
		out_dists_sqr.resize(K);
		// std::vector<float> out_dists_sqr(K);
		nanoflann::KNNResultSet<float> resultSet(K);
		resultSet.init(&out_ids[0], &out_dists_sqr[0]);
		mat_index.index->findNeighbors(resultSet, &queries[i * dim], nanoflann::SearchParams(10));
		for (long long j = 0; j < K; j++)
		{
			indices[i * K + j] = long(out_ids[j]);
		}
	}
}

void cpp_knn_batch(const float *batch_data, const size_t batch_size, const size_t npts, const size_t dim,
				   const float *queries, const size_t nqueries,
				   const size_t K, long *batch_indices)
{

	for (size_t bid = 0; bid < batch_size; bid++)
	{

		const float *points = &batch_data[bid * npts * dim];
		long *indices = &batch_indices[bid * nqueries * K];

		// create the kdtree
		typedef KDTreeTableAdaptor<float, float> KDTree;
		KDTree mat_index(npts, dim, points, 10);

		mat_index.index->buildIndex();

		std::vector<float> out_dists_sqr(K);
		std::vector<size_t> out_ids(K);

		// iterate over the points
		for (size_t i = 0; i < nqueries; i++)
		{
			nanoflann::KNNResultSet<float> resultSet(K);
			resultSet.init(&out_ids[0], &out_dists_sqr[0]);
			mat_index.index->findNeighbors(resultSet, &queries[bid * nqueries * dim + i * dim], nanoflann::SearchParams(10));
			for (size_t j = 0; j < K; j++)
			{
				indices[i * K + j] = long(out_ids[j]);
			}
		}
	}
}

void cpp_knn_batch_omp(const float *batch_data, const long long batch_size, const long long npts, const long long dim,
					   const float *queries, const long long nqueries,
					   const long long K, long *batch_indices)
{

#pragma omp parallel for
	for (long long bid = 0; bid < batch_size; bid++)
	{

		const float *points = &batch_data[bid * npts * dim];
		long *indices = &batch_indices[bid * nqueries * K];

		// create the kdtree
		typedef KDTreeTableAdaptor<float, float> KDTree;
		KDTree mat_index(npts, dim, points, 10);

		mat_index.index->buildIndex();

		std::vector<float> out_dists_sqr(K);
		std::vector<size_t> out_ids(K);

		// iterate over the points
		for (long long i = 0; i < nqueries; i++)
		{
			nanoflann::KNNResultSet<float> resultSet(K);
			resultSet.init(&out_ids[0], &out_dists_sqr[0]);
			mat_index.index->findNeighbors(resultSet, &queries[bid * nqueries * dim + i * dim], nanoflann::SearchParams(10));
			for (long long j = 0; j < K; j++)
			{
				indices[i * K + j] = long(out_ids[j]);
			}
		}
	}
}

void cpp_knn_batch_distance_pick(const float *batch_data, const size_t batch_size, const size_t npts, const size_t dim,
								 float *batch_queries, const size_t nqueries,
								 const size_t K, long *batch_indices)
{

	mt19937 mt_rand(time(0));

	for (size_t bid = 0; bid < batch_size; bid++)
	{

		const float *points = &batch_data[bid * npts * dim];
		// long* indices = &batch_indices[bid*nqueries*K];
		// float* queries = &batch_queries[bid*nqueries*dim];

		// create the kdtree
		typedef KDTreeTableAdaptor<float, float> KDTree;
		KDTree tree(npts, dim, points, 10);
		tree.index->buildIndex();

		vector<int> used(npts, 0);
		int current_id = 0;
		for (size_t ptid = 0; ptid < nqueries; ptid++)
		{
			// get the possible points
			vector<size_t> possible_ids;
			while (possible_ids.size() == 0)
			{
				for (size_t i = 0; i < npts; i++)
				{
					if (used[i] == current_id)
					{
						possible_ids.push_back(i);
					}
				}
				if (possible_ids.size() == 0)
				{
					current_id = *std::min_element(std::begin(used), std::end(used));
				}
			}

			// get an index
			size_t index = possible_ids[mt_rand() % possible_ids.size()];

			// create the query
			vector<float> query(3);
			for (size_t i = 0; i < dim; i++)
			{
				query[i] = batch_data[bid * npts * dim + index * dim + i];
			}
			// get the indices
			std::vector<float> dists(K);
			std::vector<size_t> ids(K);
			nanoflann::KNNResultSet<float> resultSet(K);
			resultSet.init(&ids[0], &dists[0]);
			tree.index->findNeighbors(resultSet, &query[0], nanoflann::SearchParams(10));

			for (size_t i = 0; i < K; i++)
			{
				used[ids[i]]++;
			}
			used[index] += 100;

			// fill the queries and neighborhoods
			for (size_t i = 0; i < K; i++)
			{
				batch_indices[bid * nqueries * K + ptid * K + i] = ids[i];
			}
			for (size_t i = 0; i < dim; i++)
			{
				batch_queries[bid * nqueries * dim + ptid * dim + i] = query[i];
			}
		}
	}
}

void cpp_knn_batch_distance_pick_omp(const float *batch_data, const long long batch_size, const long long npts, const long long dim,
									 float *batch_queries, const long long nqueries,
									 const long long K, long *batch_indices)
{

	mt19937 mt_rand(time(0));

#pragma omp parallel for
	for (long long bid = 0; bid < batch_size; bid++)
	{

		const float *points = &batch_data[bid * npts * dim];
		// long* indices = &batch_indices[bid*nqueries*K];
		// float* queries = &batch_queries[bid*nqueries*dim];

		// create the kdtree
		typedef KDTreeTableAdaptor<float, float> KDTree;
		KDTree tree(npts, dim, points, 10);
		tree.index->buildIndex();

		vector<int> used(npts, 0);
		int current_id = 0;
		for (long long ptid = 0; ptid < nqueries; ptid++)
		{
			// get the possible points
			vector<size_t> possible_ids;
			while (possible_ids.size() == 0)
			{
				for (long long i = 0; i < npts; i++)
				{
					if (used[i] == current_id)
					{
						possible_ids.push_back(i);
					}
				}
				if (possible_ids.size() == 0)
				{
					current_id = *std::min_element(std::begin(used), std::end(used));
				}
			}

			// get an index
			size_t index = possible_ids[mt_rand() % possible_ids.size()];

			// create the query
			vector<float> query(3);
			for (long long i = 0; i < dim; i++)
			{
				query[i] = batch_data[bid * npts * dim + index * dim + i];
			}
			// get the indices
			std::vector<float> dists(K);
			std::vector<size_t> ids(K);
			nanoflann::KNNResultSet<float> resultSet(K);
			resultSet.init(&ids[0], &dists[0]);
			tree.index->findNeighbors(resultSet, &query[0], nanoflann::SearchParams(10));

			for (long long i = 0; i < K; i++)
			{
				used[ids[i]]++;
			}
			used[index] += 100;

			// fill the queries and neighborhoods
			for (long long i = 0; i < K; i++)
			{
				batch_indices[bid * nqueries * K + ptid * K + i] = ids[i];
			}
			for (long long i = 0; i < dim; i++)
			{
				batch_queries[bid * nqueries * dim + ptid * dim + i] = query[i];
			}
		}
	}
}
