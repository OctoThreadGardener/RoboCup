#	data published by the localization node.

std_msgs/Header header 								# for time stamp
geometry_msgs/Pose2D robotPose        				# Pose of the robot according to the particle filter localization	
float32 robotPoseConfidence               			# confidence 0..1 for the robot_pose
bool bBallWasSeen                               	# boolean variable for ball detection
geometry_msgs/Point ballCenterInImage				# coordinates of ball center in the image
float32 ballDistance								# ball distance from robot as seen in the image
float32 ballAngle									# ball angle from center as seen in the image, [-90,90]
geometry_msgs/Point ballCenterOnField				# coordinates of ball center on field relative to robot localization	
bool bObstacleWasSeen                           	# boolean variable for obstacle detection
int32 iObstacleNumber                           	# index of obstacle, if 2 found this is 2
geometry_msgs/Point[] obstacleLeftEndInImage		# coordinates of obstacle left end point in the image
geometry_msgs/Point[] obstacleRightEndInImage		# coordinates of obstacle right end point in the image
float32[] obstacleDistance							# obstacle distance from robot as seen in the image
float32[] obstacleAngle								# obstacle angle from center as seen in the image, [-90,90]
geometry_msgs/Point[] obstacleCenterOnField			# coordinates of obstacle center on field relative to robot localization	
float32[] obstacleRadiusOnField						# estimated radius of obstacle on field
bool bKeeperWasSeen                             	# boolean variable for goalkeeper detection
geometry_msgs/Point keeperLeftStartOnField      	# coordinates of goalkeeper left leg start point on field
geometry_msgs/Point keeperRightEndOnField       	# coordinates of goalkeeper right leg end point on field
