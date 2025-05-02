    def lidar_callback(self, msg: PointCloud2):
        """ Process LiDAR scan and update the occupancy grid """

        # Transformation
        to_frame_rel = 'map'
        from_frame_rel = msg.header.frame_id

        time = rclpy.time.Time().from_msg(msg.header.stamp)

        tf_future = self.tf_buffer.wait_for_transform_async(
            target_frame=to_frame_rel,
            source_frame=from_frame_rel,
            time=time
        )

        rclpy.spin_until_future_complete(self, tf_future, timeout_sec=1)

        try:
            transform = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                time)
        except Exception as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return
        
        # Transform PointCloud2 to the map frame
        cloud_transformed = do_transform_cloud(msg, transform)

        # Get the robot's position in the map frame
        # Transformation
        to_frame_rel = 'map'
        from_frame_rel = 'base_link'

        time = rclpy.time.Time().from_msg(msg.header.stamp)

        tf_future = self.tf_buffer.wait_for_transform_async(
            target_frame=to_frame_rel,
            source_frame=from_frame_rel,
            time=time
        )

        rclpy.spin_until_future_complete(self, tf_future, timeout_sec=1)

        try:
            robot_transform = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                time)
        except Exception as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return
        
        robot_x = robot_transform.transform.translation.x
        robot_y = robot_transform.transform.translation.y
            
        # Convert PointCloud2 to numpy array
        points = np.array(list(pc2.read_points(cloud_transformed, field_names=("x", "y"), skip_nans=True)))

        # Distance thresholding relative to the robot's position
        min_distance = 0.4 # Minimum distance, 0.35 enough to remove the arm from the lidar
        max_distance = 2.5  # Maximum distance (e.g., 10 meters)

        # Filter points based on distance threshold relative to the robot
        filtered_points = []
        for point in points:
            # Calculate distance from the robot (in the map frame)
            distance = np.sqrt((point[0] - robot_x)**2 + (point[1] - robot_y)**2)

            # Apply the distance thresholding (ignore points too close or too far from the robot)
            if min_distance <= distance <= max_distance:
                filtered_points.append(point)

        # Update the occupancy grid with filtered points
        self.update_occupancy_grid(filtered_points)
  