#include <moveit_utils/planner.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test");
    ros::AsyncSpinner spinner(1); spinner.start();

    std::vector<std::string> group_name{"manipulator2","manipulator"};
    moveit_utils::planner planner(group_name);

    //auto names = planner.getCurrentJointValues("manipulator");
    //for(auto name: names)
    //    std::cerr << name*180/3.14159265 << '\n';

    //planner.setPlannerId("manipulator", "TRRT");
    //planner.getPlannerParams("manipulator");

/*
    std::vector<double> a={1,2,3,0,0,0};
    std::vector<double> b={4,5,6,0,0,0};
    auto Pose = moveit_utils::list2PoseArray(a,b);

    std::vector<double> s1={0.1, 0.1, 0.1};
    std::vector<double> s2={0.2, 0.2, 0.2};
    auto Size = moveit_utils::list2Vec3Array(s1,s2);

    std::vector<double> c1={0, 1, 0, 1};
    std::vector<double> c2={0, 1, 0, 1};
    auto Color = moveit_utils::list2ColorArray(c1,c2);
*/
    auto Pose = moveit_utils::list2PoseArray({0.1,0.0,1.0});
    auto Size = moveit_utils::list2Vec3Array({0.1,0.1,0.1});
    auto Color = moveit_utils::list2ColorArray({0,1,0,1});
    planner.addBoxObject("manipulator", "box1", Pose, Size, Color);

    auto Pose2 = moveit_utils::list2PoseArray({0.2,0.0,1.0});
    auto Size2 = moveit_utils::list2Vec3Array({0.1,0.1,0.1});
    auto Color2 = moveit_utils::list2ColorArray({1,0,0,1});
    planner.addBoxObject("manipulator", "box2", Pose2, Size2, Color2);

    //planner.remObject("box1");

    auto j = planner.getCurrentJointValues("manipulator");
    auto j2 = planner.getCurrentJointValues("manipulator");
    planner.plan("manipulator", {0,1,0,0,0,0});

    return 0;
}