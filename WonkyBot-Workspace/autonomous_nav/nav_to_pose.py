
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped

def main():
    rclpy.init()
    nav = BasicNavigator()

    # 1. Wait for Nav2 to be Active
    nav.waitUntilNav2Active()

    # 2. Set your Goal
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = nav.get_clock().now().to_msg()
    
    # Use the coordinates you just found in RViz
    goal_pose.pose.position.x = 1.0  # Meter(s) forward
    goal_pose.pose.position.y = 0.0
    goal_pose.pose.orientation.w = 1.0 # Facing forward

    print("🚀 Autonomous mission started...")
    nav.goToPose(goal_pose)

    # 3. Monitor
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        if feedback:
            # This will show you in real-time how far it is
            print(f"Distance remaining: {feedback.distance_remaining:.2f} m")

    # 4. Result
    result = nav.getResult()
    if result == TaskResult.SUCCEEDED:
        print("🏁 Mission Accomplished!")
    else:
        print("💥 Mission Failed!")

    rclpy.shutdown()

if __name__ == '__main__':
    main()
