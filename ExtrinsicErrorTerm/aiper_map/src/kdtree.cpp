

#include "ExtrinsicErrorTerm/kdtree.h"
#include "ExtrinsicErrorTerm/common/math_utils.h"

#include <glog/logging.h>
#include <set>

namespace sad
{

    bool KdTree::BuildTree(const CloudPtr &cloud)
    {
        if (cloud->empty())
        {
            return false;
        }

        cloud_.clear();
        cloud_.resize(cloud->size());
        for (size_t i = 0; i < cloud->points.size(); ++i)
        {
            cloud_[i] = ToVec3f(cloud->points[i]);
        }

        Clear();
        Reset();

        IndexVec idx(cloud->size());
        for (int i = 0; i < cloud->points.size(); ++i)
        {
            idx[i] = i;
        }

        Insert(idx, root_.get()); // get()它的作用是返回智能指针管理的原始指针，但不传递所有权。
        return true;
    }

    void KdTree::Insert(const IndexVec &points, KdTreeNode *node)
    {
        nodes_.insert({node->id_, node}); // unordered_map insert

        if (points.empty())
        {
            return;
        }

        if (points.size() == 1)
        {
            size_++;
            node->point_idx_ = points[0];
            return;
        }

        IndexVec left, right;
        if (!FindSplitAxisAndThresh(points, node->axis_index_, node->split_thresh_, left, right))
        {
            size_++;
            node->point_idx_ = points[0];
            return;
        }

        const auto create_if_not_empty = [&node, this](KdTreeNode *&new_node, const IndexVec &index)
        {
            if (!index.empty())
            {
                new_node = new KdTreeNode;
                new_node->id_ = tree_node_id_++;
                Insert(index, new_node);
            }
        };

        create_if_not_empty(node->left_, left);
        create_if_not_empty(node->right_, right);
    }

    bool KdTree::GetClosestPoint(const PointType &pt, std::vector<int> &closest_idx, int k)
    {
        if (k > size_)
        {
            LOG(ERROR) << "cannot set k larger than cloud size: " << k << ", " << size_;
            return false;
        }
        k_ = k;
        // std::priority_queue是一种容器适配器，它提供了优先队列的功能。
        // 继承自堆的特性，它可以确保每次取出的元素都是当前队列中优先级最高的
        // （即根据比较方式确定的“最大”或“最小”的元素）。std::priority_queue
        // 在内部通常是使用一个std::vector作为默认的底层容器，并且将其组织成一个堆结构。
        std::priority_queue<NodeAndDistance> knn_result;
        Knn(ToVec3f(pt), root_.get(), knn_result);

        // 排序并返回结果
        closest_idx.resize(knn_result.size());
        for (int i = closest_idx.size() - 1; i >= 0; --i)
        {
            // 倒序插入
            closest_idx[i] = knn_result.top().node_->point_idx_;
            knn_result.pop();
        }
        return true;
    }

    bool KdTree::GetClosestPointMT(const CloudPtr &cloud, std::vector<std::pair<size_t, size_t>> &matches, int k)
    {
        matches.resize(cloud->size() * k);

        // 索引
        std::vector<int> index(cloud->size());
        for (int i = 0; i < cloud->points.size(); ++i)
        {
            index[i] = i;
        }

        std::for_each(index.begin(), index.end(), [this, &cloud, &matches, &k](int idx)
                      {
        std::vector<int> closest_idx;
        GetClosestPoint(cloud->points[idx], closest_idx, k);
        for (int i = 0; i < k; ++i) {
            matches[idx * k + i].second = idx;
            if (i < closest_idx.size()) {
                matches[idx * k + i].first = closest_idx[i];
            } else {
                matches[idx * k + i].first = math::kINVALID_ID;
            }
        } });

        return true;
    }

    void KdTree::Knn(const Vec3f &pt, KdTreeNode *node, std::priority_queue<NodeAndDistance> &knn_result) const
    {
        if (node->IsLeaf())
        {
            // 如果是叶子，检查叶子是否能插入
            ComputeDisForLeaf(pt, node, knn_result);
            return;
        }

        // 看pt落在左还是右，优先搜索pt所在的子树
        // 然后再看另一侧子树是否需要搜索
        KdTreeNode *this_side, *that_side;
        if (pt[node->axis_index_] < node->split_thresh_)
        {
            this_side = node->left_;
            that_side = node->right_;
        }
        else
        {
            this_side = node->right_;
            that_side = node->left_;
        }

        Knn(pt, this_side, knn_result);
        if (NeedExpand(pt, node, knn_result))
        { // 注意这里是跟自己比
            Knn(pt, that_side, knn_result);
        }
    }

    bool KdTree::NeedExpand(const Vec3f &pt, KdTreeNode *node, std::priority_queue<NodeAndDistance> &knn_result) const
    {
        if (knn_result.size() < k_)
        {
            return true;
        }

        if (approximate_)
        {
            float d = pt[node->axis_index_] - node->split_thresh_;//点到分割面的距离
            if ((d * d) < knn_result.top().distance2_ * alpha_)//如果这个距离的平方小于优先队列中当前距离最远的点的距离平方乘以alpha_
            {
                return true;//表示至少在理论上存在在该子节点中找到更近点的可能性，因此需要扩展搜索到这个子节点。
            }
            else
            {
                return false;
            }
        }
        else
        {
            // 检测切面距离，看是否有比现在更小的
            float d = pt[node->axis_index_] - node->split_thresh_;
            if ((d * d) < knn_result.top().distance2_)
            {
                return true;
            }
            else
            {
                return false;
            }
        }
    }

    void KdTree::ComputeDisForLeaf(const Vec3f &pt, KdTreeNode *node,
                                   std::priority_queue<NodeAndDistance> &knn_result) const
    {
        // 比较与结果队列的差异，如果优于最远距离，则插入
        float dis2 = Dis2(pt, cloud_[node->point_idx_]);
        if (knn_result.size() < k_)
        {
            // results 不足k
            knn_result.emplace(node, dis2);
        }
        else
        {
            // results等于k，比较current与max_dis_iter之间的差异
            if (dis2 < knn_result.top().distance2_)
            {
                knn_result.emplace(node, dis2);
                knn_result.pop();
            }
        }
    }

    bool KdTree::FindSplitAxisAndThresh(const IndexVec &point_idx, int &axis, float &th, IndexVec &left, IndexVec &right)
    {
        // 计算三个轴上的散布情况，我们使用math_utils.h里的函数
        Vec3f var;
        Vec3f mean;
        math::ComputeMeanAndCovDiag(point_idx, mean, var, [this](int idx)
                                    { return cloud_[idx]; });
        int max_i, max_j;
        // max_i将被设置为包含最大系数的元素的行位置。
        // max_j将被设置为包含最大系数的元素的列位置。
        var.maxCoeff(&max_i, &max_j);
        axis = max_i;
        th = mean[axis];

        for (const auto &idx : point_idx)
        {
            if (cloud_[idx][axis] < th)
            {
                // 中位数可能向左取整
                left.emplace_back(idx);
            }
            else
            {
                right.emplace_back(idx);
            }
        }

        // 边界情况检查：输入的points等于同一个值，上面的判定是>=号，所以都进了右侧
        // 这种情况不需要继续展开，直接将当前节点设为叶子就行
        // 所有点落在分割面上：如果输入的所有点的坐标在选定的分割轴上的值相同，
        // 这意味着不能基于这个轴的值来分割它们。因为当我们用它们在这个轴上的值与阈值进行比较时，
        // 所有点的值都会等于这个阈值。判定条件：代码中判断是否将点分到左子集或右子集的条件设置的是
        // if (cloud_[idx][axis] < th)，这意味着如果点在分割轴上的值等于阈值时，
        // 它们会被划分到右子集（因为不满足小于条件）。
        // 右子集填充：由于这种比较方式的设定（使用>=），所有点都将被分配到右子集中，左子集将会是空的。
        // 无需继续展开：当出现这种情况时，根据Kd树的构建逻辑，没有必要再继续分割这个节点。
        // 这是因为我们已经不能找到一个有效的分割方式来进一步区分子点集了。如果对一个空的子集继续分割，
        // 只会不必要地增加计算和存储的开销，并且不会对搜索和查询的效率有任何的提升。
        // 因此，当识别到这个边界情况，函数返回false，这个意味着当前节点应该作为一个叶子节点（无法再进一步分割）。
        // 这在实际实现中是一种有效的优化，防止了对于已经是同一值的点集合进行无意义的分割尝试。
        if (point_idx.size() > 1 && (left.empty() || right.empty()))
        {
            return false;
        }

        return true;
    }

    void KdTree::Reset()
    {
        tree_node_id_ = 0;
        root_.reset(new KdTreeNode());
        root_->id_ = tree_node_id_++;
        size_ = 0;
    }

    void KdTree::Clear()
    {
        for (const auto &np : nodes_)
        {
            if (np.second != root_.get())
            {
                delete np.second;
            }
        }

        nodes_.clear();
        root_ = nullptr;
        size_ = 0;
        tree_node_id_ = 0;
    }

    void KdTree::PrintAll()
    {
        for (const auto &np : nodes_)
        {
            auto node = np.second;
            if (node->left_ == nullptr && node->right_ == nullptr)
            {
                LOG(INFO) << "leaf node: " << node->id_ << ", idx: " << node->point_idx_;
            }
            else
            {
                LOG(INFO) << "node: " << node->id_ << ", axis: " << node->axis_index_ << ", th: " << node->split_thresh_;
            }
        }
    }

} // namespace sad
