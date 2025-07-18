from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    #moveit_config = MoveItConfigsBuilder("moveit_resources_panda").to_dict()
    #moveit_config = MoveItConfigsBuilder("ur5_new").to_dict()      #chat dice che è sbagliato

    #Non so se cambia tra questa e quella a riga 7
    moveit_config = (
        MoveItConfigsBuilder("custom_robot", package_name="ur5_gripper_moveit_config")
        .robot_description(file_path="config/ur.urdf.xacro", mappings={"name": "ur"})
        .robot_description_semantic(file_path="config/ur.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_scene_monitor(
            publish_robot_description= True, publish_robot_description_semantic=True, publish_planning_scene=True
        )
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    ) 

    moveit_config=moveit_config.to_dict() 



    ompl_override = {
        "ompl": {
            # ───────────────────────────────────────────────────────────────────────────
            # Planner configuration overrides for RRTConnect
            "planner_configs": {
                "RRTConnectkConfigDefault": {
                    # 'range' controls the maximum distance (in joint‐space) between
                    # consecutive samples. A larger value lets RRTConnect explore
                    # the space more aggressively (fewer samples), at the cost of
                    # potentially missing narrow passages. 0.5 is chosen to halve
                    # the number of samples while still reaching most goals.
                    "range": 0.5,

                    # 'goal_bias' is the fraction of samples drawn directly toward
                    # the goal state. A lower bias (0.05) forces more uniform random
                    # exploration, which can help find alternative routes quickly
                    # in open spaces.
                    "goal_bias": 0.05,

                    # 'max_planning_time' caps how many seconds OMPL will spend
                    # trying to find a path. By setting this to 1.0, we guarantee
                    # each planning call returns in ≤1s, trading thoroughness
                    # for responsiveness.
                    "max_planning_time": 1.0
                }
            },

            # ───────────────────────────────────────────────────────────────────────────
            # Global OMPL parameters (apply to all planners if not overridden)

            # 'path_tolerance' (m) is the maximum distance allowed between
            # adjacent waypoints during time‐optimal parameterization. Lowering
            # to 0.01 prevents large deviations that could collide when streamed
            # to the controller.
            "path_tolerance": 0.01,

            # 'resample_dt' (s) is the time step used when resampling the
            # entire joint trajectory. A smaller dt (0.05) creates more
            # waypoints for collision checking, smoothing out jerky motions.
            "resample_dt": 0.05,

            # 'min_angle_change' (rad) filters out tiny joint movements that
            # don’t affect the overall path. Raising to 0.01 prunes insignificant
            # wiggles and reduces collision checks.
            "min_angle_change": 0.01,

            # 'default_workspace_bounds' (m) defines the half‐size of the
            # default sampling box when no workspace is specified. Tightening
            # to 2.0m keeps samples near your robot’s actual reach envelope
            # and avoids wasted exploration far outside.
            "default_workspace_bounds": 2.0,

            # ───────────────────────────────────────────────────────────────────────────
            # Start‐state correction parameters

            # 'start_state_max_bounds_error' (rad or m) is how far outside
            # the bounds OMPL will jiggle the start state if it’s invalid.
            # Reducing to 0.01 ensures any automatic perturbation stays very
            # close to the true start, preventing new collisions.
            "start_state_max_bounds_error": 0.01,

            # 'jiggle_fraction' is the fraction of the joint range used to
            # randomly perturb an in‐collision start state. A small fraction
            # (0.01) avoids moving you into radically different, potentially
            # worse poses.
            "jiggle_fraction": 0.01,

            # 'max_sampling_attempts' is how many times OMPL will try to fix
            # a bad start state before failing. 200 gives a moderate budget
            # to find any valid start without hanging.
            "max_sampling_attempts": 200
        }
    }



    # MTC Demo node
    pick_place_demo = Node(
        package="mtc_package",
        executable="mtc_node",
        output="screen",
        parameters=[
            moveit_config,
            ompl_override      # ← insert your OMPL tweaks here
        ],
    )

    return LaunchDescription([pick_place_demo])