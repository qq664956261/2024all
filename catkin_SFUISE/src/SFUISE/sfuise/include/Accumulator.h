#pragma once

#include <array>
#include <chrono>
#include <unordered_map>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>
#include <eigen3/Eigen/CholmodSupport>

template <class T>
using SparseLLT = Eigen::CholmodSupernodalLLT<T>;

class SparseHashAccumulator
{
public:
    typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VectorX;
    typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatrixX;
    typedef Eigen::Triplet<double> T;
    typedef Eigen::SparseMatrix<double> SparseMatrix;
    // 是在哈希映射中添加或累积矩阵块。该方法与 Eigen 的矩阵类型一起工作，并且通过模板可以有效地处理在编译时已知大小的任何矩阵。
    // 模板参数:
    // ROWS: 要添加的矩阵块的行数。
    // COLS: 要添加的矩阵块的列数。
    // Derived: 派生自 Eigen::MatrixBase 的 Eigen 矩阵的类型。它代表了函数将要操作的数据类型。
    template <int ROWS, int COLS, typename Derived>
    //     函数参数:
    // int si: 矩阵块在更大稀疏矩阵中的起始行索引。
    // int sj: 矩阵块在更大稀疏矩阵中的起始列索引。
    // const Eigen::MatrixBase<Derived>& data: 引用将要被添加到稀疏累加器中的矩阵数据。
    inline void addH(int si, int sj, const Eigen::MatrixBase<Derived> &data)
    {
        //  这是一个 Eigen 宏，用于在编译时检查传入的矩阵大小（Derived）是否确实为 ROWS 行 COLS 列
        EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived, ROWS, COLS);
        KeyT id;
        // 这两行将矩阵块的起始行和列索引分别赋值给键数组的前两个元素。
        id[0] = si;
        id[1] = sj;
        // 这两行将矩阵块的尺寸赋值给键数组的最后两个元素。
        id[2] = ROWS;
        id[3] = COLS;
        // 尝试在哈希映射中找到这个键。
        auto it = hash_map.find(id);
        // 如果在哈希映射中没有找到键，这行代码会检查。如果没找到，就使用 hash_map.emplace(id, data);
        //  添加新的键值对。这实际上是在哈希映射中添加了该矩阵块。
        if (it == hash_map.end())
        {
            hash_map.emplace(id, data);
        }
        // 如果找到了键，意味着具有相同起始索引和大小的矩阵块已经存在于哈希映射中。在这种情况下，函数将新数据添加（+=）到现有的矩阵块中。
        else
        {
            it->second += data;
        }
    }

    template <int ROWS, typename Derived>
    inline void addB(int i, const Eigen::MatrixBase<Derived> &data)
    {
        b.template segment<ROWS>(i) += data;
    }

    inline void setup_solver()
    {
        std::vector<T> triplets;
        //         hash_map.size() * 36: 哈希表 hash_map 每个元素代表稀疏矩阵中的一个块。
        //         乘以 36 意味着我们假设每个块平均可以产生 36 个三元组。
        //         这个数字36是基于矩阵块可能的内部结构的一个估计，
        //         也可能与具体的问题场景和块大小相关。例如，如果包含的每个块是 6x6 的大小，
        //         那么一个块可以贡献最多 36 个非零元素，因为 6 乘以 6 等于 36。
        // b.rows(): b 是一个向量，b.rows() 返回这个向量的大小，代表向量中的元素数量。
        // 在这段代码中，b 向量中的每个元素将在稀疏矩阵的对角线上添加一个三元组，
        // 也就是说添加一个非零对角线元素以保证稀疏矩阵的结构性。因此，b.rows() 代表对角线上三元组的数量。

        triplets.reserve(hash_map.size() * 36 + b.rows());
        // kv.second 表示矩阵数据块，而 kv.first[0] 和 kv.first[1] 表示该数据块在大矩阵中的位置索引。
        for (const auto &kv : hash_map)
        {
            for (int i = 0; i < kv.second.rows(); i++)
            {
                for (int j = 0; j < kv.second.cols(); j++)
                {
                    triplets.emplace_back(kv.first[0] + i, kv.first[1] + j,
                                          kv.second(i, j));
                }
            }
        }
        for (int i = 0; i < b.rows(); i++)
        {
            triplets.emplace_back(i, i, std::numeric_limits<double>::min());
        }
        smm = SparseMatrix(b.rows(), b.rows());
        // 使用三元组列表将数据填充到稀疏矩阵 smm
        smm.setFromTriplets(triplets.begin(), triplets.end());
    }
    //从成员变量 smm 中提取对角线元素
    inline VectorX Hdiagonal() const { return smm.diagonal(); }

    inline VectorX &getB() { return b; }

    inline VectorX solve(const VectorX *diagonal) const
    {
        auto t2 = std::chrono::high_resolution_clock::now();
        SparseMatrix sm = smm;
        if (diagonal)
            sm.diagonal() += *diagonal;//这个操作可能用于加强矩阵的对角线，防止某些数值问题，比如提高矩阵的正定性。
        VectorX res;
        SparseLLT<SparseMatrix> chol(sm);
        res = chol.solve(b);
        auto t3 = std::chrono::high_resolution_clock::now();
        auto elapsed2 = std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2);
        return res;
    }

    inline void reset(int opt_size)
    {
        hash_map.clear();
        b.setZero(opt_size);
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    using KeyT = std::array<int, 4>;
    // 这个自定义散列函数的实现如下：
    // operator()函数是散列函数对象必须提供的操作符。它以一个引用const KeyT& c作为输入，这是要生成散列值的键。
    // 函数体内，定义了一个名为seed的变量，其类型为size_t，用于累积计算最终的散列值，它的初始值被设置为0。
    // 接下来是一个循环，循环遍历键数组c的每个元素（由于KeyT键是一个有4个整数的数组，所以这里是一个固定的4次循环）。
    // 在每次迭代中，seed与数组中当前元素c[i]进行位异或（^）操作，并且同时还与magic number 0x9e3779b9（这是一个给散列函数提供良好属性的常用常量）和当前seed的换位结果进行异或操作。换位操作包括将seed向左移动6位（seed << 6）和向右移动2位（seed >> 2）。
    // 循环结束后，返回最终的散列值seed。
    // 这个散列函数实际上是为了使键值分布更均匀，并减少在哈希表中可能发生的冲突，确保unordered_map可以更有效率地工作。自定义散列函数是必须的，因为标准库不提供对用户自定义类型（如KeyT这样的数组）的散列函数。
    struct KeyHash
    {
        inline size_t operator()(const KeyT &c) const
        {
            size_t seed = 0;
            for (int i = 0; i < 4; i++)
            {
                seed ^= c[i] + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            }
            return seed;
        }
    };
    // KeyT 是键的类型，MatrixX 是值的类型，KeyHash 是一个自定义的散列函数结构，
    // 这个结构体中定义了散列函数运算符 (operator()) 来给出键的散列值。
    // 因为默认的散列函数不知道如何有效地处理数组，所以自定义散列函数在使用数组或其他非标准类型作为键时是必需的。
    std::unordered_map<KeyT, MatrixX, KeyHash> hash_map;
    VectorX b;
    SparseMatrix smm;
};
