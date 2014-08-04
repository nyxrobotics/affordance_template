#ifndef ROBOT_CONFIG_HPP
#define ROBOT_CONFIG_HPP

using namespace std;

namespace rviz_affordance_template_panel
{

	class EndEffectorConfig
    {
    public:
        EndEffectorConfig(const string& name) {name_=name;};
        ~EndEffectorConfig() {}

        string name() const { return name_; }
        void name(const string& name ) { name_=name; }

        int id() const { return id_; }
        void id(int id) { id_=id; }

		vector<float> pose_offset() const { return pose_offset_; }
        void pose_offset(const vector<float> pose_offset ) { pose_offset_=pose_offset; }

    private:
        string name_;
        int id_;
        vector<float> pose_offset_;
    };


    class EndEffectorPoseConfig
    {
    public:
        EndEffectorPoseConfig(const string& name) {name_=name;};
        ~EndEffectorPoseConfig() {}

        string name() const { return name_; }
        void name(const string& name ) { name_=name; }

        string group() const { return group_; }
        void group(const string& group ) { group_=group; }

        int id() const { return id_; }
        void id(int id) { id_=id; }

    private:
        string name_;
        string group_;
        int id_;
    };


    class RobotConfig
    {
    public:
    	typedef boost::shared_ptr<EndEffectorConfig> EndEffectorConfigSharedPtr;
        typedef boost::shared_ptr<EndEffectorPoseConfig> EndEffectorPoseIDConfigSharedPtr;

        RobotConfig(const string& uid) {uid_=uid;};
        ~RobotConfig() {}

        string uid() const { return uid_; }
        void uid(const string& uid ) { uid_=uid; }

        string name() const { return name_; }
        void name(const string& name ) { name_=name; }

        string frame_id() const { return frame_id_; }
        void frame_id(const string& frame_id ) { frame_id_=frame_id; }

        string moveit_config_package() const { return moveit_config_package_; }
        void moveit_config_package(const string& moveit_config_package ) { moveit_config_package_=moveit_config_package; }

		vector<float> root_offset() const { return root_offset_; }
        void root_offset(const vector<float> root_offset ) { root_offset_=root_offset; }

        std::map<std::string, EndEffectorConfigSharedPtr> endeffectorMap;
        std::map<std::string, EndEffectorPoseIDConfigSharedPtr> endeffectorPoseMap;


    private:
        string uid_;
        string name_;
        string frame_id_;
        string moveit_config_package_;
        vector<float> root_offset_;

    };
}

#endif