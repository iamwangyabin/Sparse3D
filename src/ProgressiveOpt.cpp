//
// Created by wang on 19-1-24.
//

#include "ProgressiveOpt.h"
#include <utility>
#include <ProgressiveOpt.h>


bool ProgressiveOpt::Init() {
    namespace fs = boost::filesystem;
    namespace fs = boost::filesystem;
    if (fs::exists(fs::path(traj_file_))) {
        traj_.LoadFromFile(traj_file_);
        if (fs::exists(fs::path(info_file_))) {
            info_.LoadFromFile(info_file_);
        }
    }

    for (auto &i : traj_.data_) {
        Eigen::Matrix4d temp = i.transformation_.inverse();
        i.transformation_ = temp;
    }

    // all pose init to identity matrix
    for (int i = 0; i < frame_num_; ++i) {
        Eigen::Matrix4d mat4d_I = Eigen::Matrix4d::Identity();
        pose_.data_.push_back(FramedTransformation(i, i, mat4d_I));
    }
    pose_visit.resize(frame_num_, false);
    return (traj_.data_.size() > 0);
}








/**
 * Beam Search（集束搜索）是一种启发式图搜索算法
 * 在图的解空间比较大的情况下，
 * 在每一步深度扩展的时候，剪掉一些质量比较差的结点，保留下一些质量较高的结点。
 *
 * */
std::vector<int> ProgressiveOpt::BeamSearch(const double err_threshold, const int beam_width,
                                            std::map<key, std::vector<int>, key_comp> &index_map,
                                            const std::vector<key> &loop, const std::vector<int> &seq) {

    for (int i=0;i<loop.size();++i){
        if (loop[i].frame1==frame_num_-1||loop[i].frame2==frame_num_-1)
            end_flag++;
    }
    Bnum++;
    std::cout << "-----------------Loop" << Bnum << "----------------\n";

    int unfix_edge=0;

    std::vector<inx2seq> fix_part;
    int all_seq_len=1;
    // init one
    for (int i = 0; i < loop.size(); ++i) {
        std::vector<int>& vec= index_map[loop[i]];
        if (seq[vec[0]] == -1)		// not fix
        {
            all_seq_len *= vec.size();
            unfix_edge++;
        }
        else
        {
            for (int j = 0; j < vec.size(); ++j)
            {
                inx2seq is(vec[j], seq[vec[j]]);
                fix_part.push_back(is);
            }
        }
    }
    /**
     * 不知道什么意思
     * vec大于50就原封不动返回……
     * */
    if (unfix_edge > 50)
        return seq;

    std::vector<int> frag_len;
    std::vector<int> inx;
    for (int i = 0; i < loop.size(); ++i) {
        std::vector<int>& vec=index_map[loop[i]];
        if (seq[vec[0]] == -1){
            inx.insert(inx.end(), vec.begin(), vec.end());
            frag_len.push_back(vec.size());
        }
    }

    std::vector<inx2seq> item;
    std::vector<std::vector<inx2seq>> result;
    recur_seq(frag_len, inx, 0, item, result, beam_width);				// recursive compute all possible solution

    for (int i = 0; i < result.size();++i)
    {
        std::vector<inx2seq>& one_loop = result[i];
        one_loop.insert(one_loop.end(), fix_part.begin(), fix_part.end());
        std::sort(one_loop.begin(), one_loop.end(), inx2seq_cmp());				// one solution
    }

    // compute minimum error
    double min_error;
    std::vector<inx2seq> best_loop;
    int loop_len = loop.size();
    LoopError(result, loop_len, min_error, best_loop);


    // update
    if (min_error < err_threshold) {
        std::vector<int> best_l = seq;
        for (int i = 0; i < best_loop.size(); ++i)
            best_l[best_loop[i].index] = best_loop[i].seq;

        for (int i = 0; i < best_l.size(); ++i) {
            if (best_l[i] == 1) {
                int v1 = traj_.data_[i].frame1_;
                int v2 = traj_.data_[i].frame2_;
                pose_visit[v1] = true;
                pose_visit[v2] = true;
            }
        }
        select_loop.push_back(loop);
        std::cout << "Successful!\n";
        std::cout << "loop Error: " << min_error << ";  Threshold:" << err_threshold << "\n";
        return best_l;
    }
    else
    {
        std::cout << "Fail!\n";
		std::cout << "loop Error: " << min_error << ";  Threshold:" << err_threshold << "\n";
        return seq;
    }
}

void ProgressiveOpt::LoopError(
        const std::vector<std::vector<inx2seq>> &all_loop,
        int loop_len,
        /*output parameter*/
        double &min_error,
        std::vector<inx2seq> &best_loop) {

    min_error = 99999.0;
    best_loop.clear();

    for (int i = 0; i < all_loop.size(); ++i) {
        const std::vector<inx2seq> &one_loop = all_loop[i];

        RGBDTrajectory t_arr;
        RGBDInformation2 i_arr;
        Eigen::Matrix4d err = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d final_mat = Eigen::Matrix4d::Identity();
        for (int k = 0; k < one_loop.size(); k++) {
            if (one_loop[k].seq < 1)
                continue;
            FramedTransformation &t = traj_.data_[one_loop[k].index];
            t_arr.data_.push_back(t);

            FrameInformation &inf = info_.data_[one_loop[k].index];
            i_arr.data_.push_back(inf);
        }

        double rotation_deg = -9999.0;
        int low_score_edge = 0;
        int geometric_num = 0;
        for (int i = 0; i < t_arr.data_.size(); ++i) {
            if (i == 1) {
                final_mat = t_arr.data_[i].transformation_;
                if (i_arr.data_[i].flag == 0)
                    geometric_num++;
                if (i_arr.data_[i].flag == 0 && i_arr.data_[i].score_ < low_score_threshold_)
                    low_score_edge++;

                Eigen::Matrix3d mat3;
                mat3 <<
                     t_arr.data_[i].transformation_(0, 0), t_arr.data_[i].transformation_(0,1), t_arr.data_[i].transformation_(0, 2),
                     t_arr.data_[i].transformation_(1, 0), t_arr.data_[i].transformation_(1,1), t_arr.data_[i].transformation_(1, 2),
                     t_arr.data_[i].transformation_(2, 0), t_arr.data_[i].transformation_(2,1), t_arr.data_[i].transformation_(2, 2);
                Eigen::Quaterniond q(mat3);
                /**将acos返回的弧度值转换为角度值
                 * 计算的是旋转的度数
                 * */
                double deg = acos(q.w()) * 180 / 3.1415;
                deg *= 2;
                if (rotation_deg < deg)
                    rotation_deg = deg;
            } else {
                err = err * t_arr.data_[i].transformation_;
                if (i_arr.data_[i].flag == 0)
                    geometric_num++;
                if (i_arr.data_[i].flag == 0 && i_arr.data_[i].score_ < low_score_threshold_)
                    low_score_edge++;

                Eigen::Matrix3d mat3;
                mat3 <<
                     t_arr.data_[i].transformation_(0, 0), t_arr.data_[i].transformation_(0,1), t_arr.data_[i].transformation_(0, 2),
                     t_arr.data_[i].transformation_(1, 0), t_arr.data_[i].transformation_(1,1), t_arr.data_[i].transformation_(1, 2),
                     t_arr.data_[i].transformation_(2, 0), t_arr.data_[i].transformation_(2,1), t_arr.data_[i].transformation_(2, 2);
                Eigen::Quaterniond q(mat3);
                double deg = acos(q.w()) * 180 / 3.1415;
                deg *= 2;
                if (rotation_deg < deg)
                    rotation_deg = deg;
            }
        }
        /***888888888888888888888888888888888888888888888*/
        err=err*final_mat.inverse();
        Eigen::Isometry3d delta(err);
        auto error = g2o::internal::toVectorMQT(delta);
        double translation_err = error(0)*error(0) + error(1)*error(1) + error(2)*error(2);
        double rotation_err = 4 * (error(3)*error(3) + error(4)*error(4) + error(5)*error(5));
        double e = translation_err + rotation_err;

        if (loop_len==3)
        {
            if (geometric_num >=2)
                e += 0.005;
        }
        if (low_score_edge>0)
        {
            e += 999.0;
        }
        if (rotation_deg>80.0)			// if rotation degree too big discard it.
        {
            e += 999.0;
        }
        if (e<min_error)
        {
            min_error = e;
            best_loop = one_loop;
        }
    }
}

void ProgressiveOpt::Opt() {
    std::map<key, std::vector<int>, key_comp> index_map;
    for (int i = 0; i < traj_.data_.size(); ++i) {
        int frame1 = traj_.data_[i].frame1_;
        int frame2 = traj_.data_[i].frame2_;
        index_map[key(frame1, frame2)].push_back(i);
    }
    std::vector<int> seq(traj_.data_.size(), -1);

    if (loopDetect_.Init(traj_file_, frame_num_)) {
        struct state {
            bool in_stack;
            int level;
            int start_vertex;

            state() : in_stack(false), level(-1), start_vertex(0) {}

            state(bool ins, int le, int start) : in_stack(ins), level(le), start_vertex(start) {}
        };
        std::vector<state> stack_vertex(frame_num_);

        // DFS search all loop
        std::stack<int> DFS;
        DFS.push(0);
        stack_vertex[0] = state(true, 0, 0);

        int no_change_num = 0;
        std::vector<bool> last_visit(frame_num_, false);
        while (!DFS.empty()) {
            int item = DFS.top();
            bool add_stack = false;
            for (int i = stack_vertex[item].start_vertex; i < frame_num_; ++i) {
                if (loopDetect_.graph[item][i] == 1 && stack_vertex[i].in_stack == false) {
                    DFS.push(i);

                    int last_level = stack_vertex[item].level;
                    stack_vertex[i] = state(true, last_level + 1, 0);
                    add_stack = true;

                    stack_vertex[item].start_vertex = i + 1;
                    break;
                }
                if (loopDetect_.graph[item][i] == 1 && stack_vertex[i].in_stack == true)        // one loop
                {
                    assert(stack_vertex[item].level != -1 && stack_vertex[i].level != -1);
                    int loop_len = stack_vertex[item].level - stack_vertex[i].level;
                    if (loop_len >= 2) {
                        std::stack<int> temp_stack = DFS;
                        std::vector<int> loop;
                        while (temp_stack.top() != i) {
                            loop.push_back(temp_stack.top());
                            temp_stack.pop();index_map
                        }
                        loop.push_back(i);
                        loop.push_back(item);

                        std::vector<key> one_loop;
                        for (int j = 0; j < loop.size() - 1; ++j) {
                            int v1 = loop[j];
                            int v2 = loop[j + 1];
                            if (v1 > v2) {
                                int temp = v1;
                                v1 = v2;
                                v2 = temp;
                            }
                            key k(v1, v2);
                            one_loop.push_back(k);
                        }

                        double err_threshold;
                        if (one_loop.size() < 10)
                            err_threshold = sqrt(one_loop.size()) * error_threshold_;
                        else
                            err_threshold = one_loop.size() * error_threshold_;

                        seq = BeamSearch(err_threshold, 1024, index_map, one_loop, seq);
                    }
                }
                // end flag
                bool all_visit = true;
                for (int i = 0; i < pose_visit.size(); ++i)
                    if (pose_visit[i] == false)
                        all_visit = false;
                if (all_visit || end_flag >= 10)
                    break;
            }// end for
            if (!add_stack){
                DFS.pop();
                stack_vertex[item].in_stack = false;
                stack_vertex[item].level = -1;
                stack_vertex[item].start_vertex = 0;
            }
            // end flag
            bool all_visit = true;
            bool visit_change = false;
            for (int i = 0; i < pose_visit.size(); ++i)
            {
                if (pose_visit[i] == false)
                    all_visit = false;
                if (pose_visit[i] != last_visit[i])
                    visit_change = true;
            }
            if (all_visit || end_flag >= 10)
                break;

            if (visit_change)
            {
                no_change_num = 0;
                last_visit = pose_visit;
            }
            else
            {
                no_change_num++;
            }
            if (no_change_num>150)
                break;
        }// end whild
    }// end if
    // connect the remaining vertices
    // Strategy 1: use sequential+score. Better result
    UnionFindSet(seq);
    // Strategy 2: only use score. Useful for unordered dataset.
    // UnionFindSetNonSequential(seq);
    if (judgeLinkAllVertex(seq, traj_))
    {
        ComputeInitPose(seq);
        ComputePose(seq);
        pose_.SaveToFile(pose_file_);
    }
}

ProgressiveOpt::ProgressiveOpt(int frame_num, std::string traj_file, std::string info_file, std::string pose_file,
                               std::string fail_file, std::string selected_edge_file, double low_score_threshold,
                               double score1, double score2, double error_threshold) {
    frame_num_ = frame_num;
    traj_file_ = std::move(traj_file);
    info_file_ = std::move(info_file);
    pose_file_ = std::move(pose_file);
    fail_file_ = std::move(fail_file);
    selected_edge_file_ = std::move(selected_edge_file);
    error_threshold_ = error_threshold;
    end_flag = 0;
    Bnum = 0;
    score_threshold1_ = score1;
    score_threshold2_ = score2;
    low_score_threshold_ = low_score_threshold;

}

void ProgressiveOpt::recur_seq(std::vector<int> frag_len, std::vector<int> &inx, int level, std::vector<inx2seq> item,
                               std::vector<std::vector<inx2seq>> &result, const int beam_width) {
    if (result.size() <= beam_width)
    {
        if (level == frag_len.size())
        {
            result.push_back(item);
        }
        else
        {
            for (int i = 0; i < frag_len[level]; ++i)
            {
                std::vector<inx2seq> item_copy = item;
                int inx_index = 0;
                for (int t = 0; t < level; ++t)
                    inx_index += frag_len[t];

                for (int j = 0; j < frag_len[level]; ++j)
                {
                    if (j == i){
                        inx2seq is(inx[inx_index + j], 1);
                        item_copy.push_back(is);
                    }
                    else
                    {
                        inx2seq is(inx[inx_index + j], 0);
                        item_copy.push_back(is);
                    }

                }
                recur_seq(frag_len, inx, level + 1, item_copy, result, beam_width);
            }
        }
    }

}

