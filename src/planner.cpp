/**
 * @file planner.cpp
 * @brief MoveIt!ドライバ(C++)
 */
#include <moveit_utils/planner.h>

// *******************************************************************************
/**
 * @brief vector配列内で指定した要素のインデックスを取得
 * @param [in] vec 検索先
 * @param [in] val 検索対象
 * @return インデックス
 */
template<typename T>
int find_index(const std::vector<T> vec, const T val)
{
    auto itr = std::find(vec.begin(), vec.end(), val);
    if (itr == vec.end())
        return -1;
    else
        return std::distance(vec.begin(), itr);
}





namespace moveit_utils
{

    // *************************************************************************
    bool planner::isValidGroup(std::string group_name)
    {
        return(move_group.count(group_name)!=0);
    }

    // *************************************************************************
    bool planner::setStartJoint(std::string group_name, std::vector<double> jnt)
    {
        if (!isValidGroup(group_name)) return false;

        if (jnt.size()==0)
        {
            move_group[group_name]->setStartStateToCurrentState();
            return true;
        }

        sensor_msgs::JointState joint;
        joint.name = move_group[group_name]->getJointNames();
        if (joint.name.size()!=jnt.size()) return false;
        joint.position = jnt;
        auto state_p = move_group[group_name]->getCurrentState(1.0);
        state_p->setVariableValues(joint);
        move_group[group_name]->setStartState(*state_p.get());
        return true;
    }

    // *************************************************************************
    bool planner::setGoalJoint(std::string group_name, std::vector<double> jnt)
    {
        if (!isValidGroup(group_name)) return false;
        sensor_msgs::JointState joint;
        joint.name = move_group[group_name]->getJointNames();
        if (joint.name.size()!=jnt.size()) return false;
        joint.position = jnt;
        return move_group[group_name]->setJointValueTarget(joint);
    }

    // *************************************************************************
    bool planner::waitForUpdate(std::string obj_name, bool is_known, bool is_attached, float timeout)
    {
        ros::Time start = ros::Time::now();
        ros::Duration dur = ros::Time::now()-start;
        while(ros::ok() && (dur.sec < timeout) )
        {
            auto attachedObj = planning_scene_interface.getAttachedObjects();
            auto knownObj = planning_scene_interface.getKnownObjectNames();
            bool is_attached_ = (attachedObj.count(obj_name)!=0);
            bool is_known_ = (find_index(knownObj, obj_name)!=-1);
            if( (is_attached_ == is_attached) && (is_known_ == is_known) )
                return true;

            ros::Duration(0.01);
            dur = ros::Time::now()-start;
        }
        return false;
    }


    // *************************************************************************
    planner::planner(std::vector<std::string> group_names)
    {
        for (auto name : group_names)
        {
            try
            {
                move_group[name] = std::make_shared<MoveGroupInterface>(name);
            }
            catch(const std::exception& e)
            {
                std::cerr << "[planner] " << e.what() << '\n';
            }
        }
    }

    // *************************************************************************
    bool planner::plan(std::string group_name, std::vector<double> goal, std::vector<double> start)
    {
        if( !setStartJoint(group_name, start) )
        {
            std::cerr << "[plan] start joint setting error." << std::endl;
            return false;
        }

        if( !setGoalJoint(group_name, goal) )
        {
            std::cerr << "[plan] goal joint setting error." << std::endl;
            return false;
        }

        MoveGroupInterface::Plan my_plan;
        if(move_group[group_name]->plan(my_plan) == MoveItErrorCode::SUCCESS)
        {
            // 成功
            std::cerr << "[plan] success." << std::endl;
            std::cerr << "[plan] planning time = " << my_plan.planning_time_ << std::endl;;
            return true;
        }
        std::cerr << "[plan] planning failed." << std::endl;
        return false;
    }


    // *************************************************************************
    bool planner::addBoxObject(std::string group_name, std::string object_name, std::vector<geometry_msgs::Pose> Pose, std::vector<geometry_msgs::Vector3> Size, std::vector<moveit_msgs::ObjectColor> Color)
    {
        if (!isValidGroup(group_name)) return false;
        if (Pose.size() != Size.size()) return false;

        moveit_msgs::CollisionObject collision_object;
        collision_object.header.frame_id = move_group[group_name]->getPlanningFrame();
        collision_object.id = object_name;
        collision_object.operation = moveit_msgs::CollisionObject::ADD;

        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);

        for(int i=0; i<Pose.size(); i++)
        {
            primitive.dimensions[0] = Size[i].x;
            primitive.dimensions[1] = Size[i].y;
            primitive.dimensions[2] = Size[i].z;
            collision_object.primitives.push_back(primitive);
            collision_object.primitive_poses.push_back(Pose[i]);
        }

        std::vector<moveit_msgs::CollisionObject> collision_objects;
        collision_objects.push_back(collision_object);
        planning_scene_interface.addCollisionObjects(collision_objects, Color);
        return waitForUpdate(object_name, true, false, 4.0);
    }

    // *************************************************************************
    bool planner::remObject(std::string object_name)
    {
        auto knownObj = planning_scene_interface.getKnownObjectNames();
        if (find_index(knownObj, object_name)==-1)
            return true;    // Not defined
        
        std::vector<std::string> objs;
        objs.push_back(object_name);
        planning_scene_interface.removeCollisionObjects(objs);
        return waitForUpdate(object_name, false, false, 4.0);
    }

    // *************************************************************************
    std::vector<std::string> planner::getGroupNames()
    {
        std::vector<std::string> keys;
        for(auto iter = move_group.begin(); iter!=move_group.end(); iter++)
            keys.push_back(iter->first);
        
        if(keys.size()==0)
            std::cerr << "[getGroupNames] No group is defined." << std::endl;
        
        return keys;
    }


    // *************************************************************************
    std::vector<double> planner::getCurrentJointValues(std::string group_name)
    {
        if (isValidGroup(group_name))
            return move_group[group_name]->getCurrentJointValues();
        else
            return std::vector<double>();
    }



    // *************************************************************************
    void planner::setPlannerId(std::string group_name, std::string id)
    {
        if (isValidGroup(group_name))
            move_group[group_name]->setPlannerId(id);
    }

    // *************************************************************************
    std::string planner::getPlannerId(std::string group_name)
    {
        if (isValidGroup(group_name))
            return move_group[group_name]->getPlannerId();
        else
            return std::string();
    }

    // *************************************************************************
    std::map<std::string, std::string> planner::getPlannerParams(std::string group_name)
    {
        if (isValidGroup(group_name))
        {
            auto id = getPlannerId(group_name);
            auto params = move_group[group_name]->getPlannerParams(id, group_name);
            for (auto iter=params.begin(); iter!=params.end(); iter++)
            {
                std::cerr << iter->first <<" : "<< iter->second <<std::endl;
            }
            return params;
        }
        else
            return std::map<std::string, std::string>();
    }


}