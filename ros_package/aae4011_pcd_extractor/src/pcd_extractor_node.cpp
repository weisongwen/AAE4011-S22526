/**
 * @file pcd_extractor_node.cpp
 * @brief ROS node for extracting point clouds from rosbag and saving to PCD files
 * 
 * This file demonstrates fundamental ROS concepts:
 * - Node initialization and lifecycle
 * - Topic subscription and callbacks
 * - Parameter server usage
 * - Message type conversions (ROS <-> PCL)
 * - File I/O with point clouds
 * - Logging and error handling
 * 
 * Educational Purpose: This is a teaching example for undergraduate students
 * learning ROS and point cloud processing.
 * 
 * @author AAE4011 Teaching Team
 * @date 2026
 */

////////////////////////////////////////////////////////////////////////////////
// INCLUDE HEADERS
////////////////////////////////////////////////////////////////////////////////
// Order of includes (best practice):
// 1. C++ standard library headers
// 2. System/external library headers  
// 3. ROS headers
// 4. Local project headers

// C++ Standard Library
#include <iostream>      // For std::cout, std::cerr (console output)
#include <string>        // For std::string
#include <sstream>       // For std::stringstream (string formatting)
#include <sys/stat.h>    // For mkdir() (directory creation)
#include <sys/types.h>   // For data types used in system calls
#include <libgen.h>      // For dirname() (path manipulation)
#include <cstring>       // For strdup() (string duplication)
#include <cerrno>        // For errno (error handling)

// ROS Core
#include <ros/ros.h>     // Main ROS header - provides ros::init(), ros::NodeHandle, etc.

// ROS Messages
#include <sensor_msgs/PointCloud2.h>  // ROS message type for point clouds

// PCL (Point Cloud Library)
#include <pcl/point_cloud.h>          // PCL's point cloud class
#include <pcl/point_types.h>          // PCL's point type definitions (PointXYZ, PointXYZI, etc.)
#include <pcl/io/pcd_io.h>            // PCL's PCD file I/O functions

// PCL-ROS Conversions
#include <pcl_conversions/pcl_conversions.h>  // Convert between ROS and PCL formats


////////////////////////////////////////////////////////////////////////////////
// HELPER FUNCTIONS
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Recursively create directories (like 'mkdir -p')
 * @param path The directory path to create
 * @return true if successful or directory already exists, false otherwise
 * 
 * This function creates all parent directories as needed, similar to 'mkdir -p'.
 * Example: If path is "a/b/c", it will create "a", then "a/b", then "a/b/c"
 */
bool createDirectoryRecursively(const std::string& path) {
  // Check if directory already exists
  struct stat st;
  if (stat(path.c_str(), &st) == 0) {
    if (S_ISDIR(st.st_mode)) {
      return true;  // Directory already exists
    } else {
      ROS_ERROR("Path exists but is not a directory: %s", path.c_str());
      return false;
    }
  }
  
  // Need to create directory - first create parent directories
  char* path_copy = strdup(path.c_str());
  char* parent_path = dirname(path_copy);
  
  // Recursively create parent directory if it's not empty and not "."
  if (strlen(parent_path) > 0 && strcmp(parent_path, ".") != 0) {
    if (!createDirectoryRecursively(parent_path)) {
      free(path_copy);
      return false;
    }
  }
  free(path_copy);
  
  // Now create this directory
  if (mkdir(path.c_str(), 0777) == 0) {
    ROS_INFO("Created directory: %s", path.c_str());
    return true;
  } else if (errno == EEXIST) {
    // Race condition: directory was created between check and mkdir
    return true;
  } else {
    ROS_ERROR("Failed to create directory: %s (error: %s)", path.c_str(), strerror(errno));
    return false;
  }
}

////////////////////////////////////////////////////////////////////////////////
// CLASS DEFINITION: PCDExtractor
////////////////////////////////////////////////////////////////////////////////
/**
 * @class PCDExtractor
 * @brief A ROS node class that subscribes to point cloud topics and saves to PCD files
 * 
 * Design Pattern: This uses Object-Oriented Programming (OOP) to encapsulate
 * all functionality related to point cloud extraction in a single class.
 * 
 * Benefits of using a class:
 * - Encapsulation: Data and methods grouped together
 * - Clean initialization: Constructor handles setup
 * - Easy to extend: Can add more methods for additional features
 * - Better organization: Clear separation of concerns
 */
class PCDExtractor
{
public:
  /**
   * @brief Constructor - Initializes the node and sets up subscriptions
   * @param nh ROS NodeHandle for communication with ROS master
   * 
   * The constructor is called once when the object is created.
   * It performs all initialization tasks:
   * - Load parameters from parameter server
   * - Create output directory
   * - Subscribe to point cloud topic
   * - Initialize member variables
   */
  PCDExtractor(ros::NodeHandle& nh) : nh_(nh), cloud_count_(0)
  {
    // =========================================================================
    // STEP 1: Load ROS Parameters
    // =========================================================================
    // ROS Parameter Server allows configuration without recompiling code.
    // Parameters can be set via:
    // - Launch files: <param name="..." value="..."/>
    // - Command line: rosrun pkg node _param:=value
    // - rosparam command: rosparam set /node/param value
    
    // Get the output directory path
    // param() tries to get parameter, uses default if not found
    // Syntax: param(parameter_name, variable_to_store, default_value)
    nh_.param<std::string>("output_directory", output_dir_, "/tmp/pcd_output");
    
    // Get the topic name to subscribe to
    nh_.param<std::string>("pointcloud_topic", topic_name_, "/velodyne_points");
    
    // Print loaded parameters to console (useful for debugging)
    ROS_INFO("===== PCD Extractor Node Started =====");
    ROS_INFO("Subscribing to topic: %s", topic_name_.c_str());
    ROS_INFO("Saving PCD files to: %s", output_dir_.c_str());
    
    // =========================================================================
    // STEP 2: Create Output Directory
    // =========================================================================
    // Ensure the output directory exists before we try to write files
    // Use recursive directory creation to handle paths like "../pcd_output"
    // This will create all necessary parent directories
    if (!createDirectoryRecursively(output_dir_)) {
      ROS_ERROR("Failed to create output directory: %s", output_dir_.c_str());
      ROS_ERROR("Please check permissions and path validity!");
      // Continue anyway - errors will be shown when trying to save files
    } else {
      ROS_INFO("Output directory ready: %s", output_dir_.c_str());
    }
    
    // =========================================================================
    // STEP 3: Create Subscriber
    // =========================================================================
    // A subscriber listens to a topic and calls a callback function when
    // new messages arrive.
    //
    // Parameters:
    // 1. topic_name: The topic to subscribe to (e.g., "/velodyne_points")
    // 2. queue_size: How many messages to buffer (10 is typical)
    //    - Too small: Messages might be dropped if callback is slow
    //    - Too large: Uses more memory
    // 3. callback: Function to call when message arrives
    //    - Here we use &PCDExtractor::cloudCallback (member function)
    //    - 'this' binds the callback to this object instance
    //
    // Message type: sensor_msgs::PointCloud2ConstPtr
    // - ConstPtr means const shared pointer (efficient, read-only)
    // - Avoids copying large point cloud data
    
    cloud_sub_ = nh_.subscribe(topic_name_, 10, 
                                &PCDExtractor::cloudCallback, this);
    
    ROS_INFO("Waiting for point cloud messages...");
    ROS_INFO("======================================");
  }
  
  /**
   * @brief Destructor - Cleanup when node shuts down
   * 
   * Called automatically when the object is destroyed (node shutdown).
   * Use this to clean up resources, close files, etc.
   */
  ~PCDExtractor()
  {
    ROS_INFO("PCD Extractor node shutting down.");
    ROS_INFO("Total point clouds saved: %d", cloud_count_);
  }

private:
  // ===========================================================================
  // MEMBER VARIABLES (Class Data)
  // ===========================================================================
  // These variables persist throughout the lifetime of the object
  // The trailing underscore (_) is a naming convention for member variables
  
  ros::NodeHandle nh_;           // ROS node handle for communication
  ros::Subscriber cloud_sub_;    // Subscriber object for point cloud topic
  
  std::string output_dir_;       // Directory path where PCD files are saved
  std::string topic_name_;       // Name of the point cloud topic to subscribe
  
  int cloud_count_;              // Counter for number of point clouds saved
  
  // ===========================================================================
  // CALLBACK FUNCTION: Called when a new point cloud message arrives
  // ===========================================================================
  /**
   * @brief Callback function triggered when a PointCloud2 message is received
   * @param cloud_msg Pointer to the received point cloud message (ROS format)
   * 
   * This is the heart of the node - it processes each incoming point cloud.
   * 
   * Workflow:
   * 1. Receive message (ROS format: sensor_msgs::PointCloud2)
   * 2. Convert to PCL format (pcl::PointCloud<pcl::PointXYZI>)
   * 3. Generate filename with timestamp
   * 4. Save to PCD file
   * 5. Log success/failure
   * 
   * Note: This function must execute quickly! If it's too slow, messages
   * will queue up and eventually be dropped. For heavy processing, consider
   * using a separate thread or queue.
   */
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
  {
    // =========================================================================
    // STEP 1: Log message receipt (optional - can be commented out if too verbose)
    // =========================================================================
    // Use different logging levels:
    // ROS_DEBUG: Detailed info, only shown with debug level logging
    // ROS_INFO:  Important info, shown by default
    // ROS_WARN:  Warnings, shown in yellow
    // ROS_ERROR: Errors, shown in red
    
    ROS_INFO("Received point cloud with %d points (timestamp: %f)", 
             cloud_msg->width * cloud_msg->height,  // Total points
             cloud_msg->header.stamp.toSec());       // Message timestamp
    
    // =========================================================================
    // STEP 2: Convert ROS message to PCL point cloud
    // =========================================================================
    // ROS and PCL use different data structures for point clouds:
    // - ROS: sensor_msgs::PointCloud2 (serialized, transport-friendly)
    // - PCL: pcl::PointCloud<PointT> (C++ class, processing-friendly)
    //
    // We need to convert between them to use PCL's functions.
    
    // Create a PCL point cloud object
    // PointXYZI = Point with X, Y, Z coordinates + Intensity
    // Other common types:
    // - PointXYZ: Just X, Y, Z
    // - PointXYZRGB: X, Y, Z + color
    // - PointXYZRGBA: X, Y, Z + color + alpha
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    
    // Convert from ROS message to PCL point cloud
    // fromROSMsg() handles all the data conversion
    // This is a deep copy - the PCL cloud is independent of the ROS message
    pcl::fromROSMsg(*cloud_msg, *pcl_cloud);
    
    // =========================================================================
    // STEP 3: Generate filename with timestamp
    // =========================================================================
    // Use timestamp to create unique filenames
    // Format: pointcloud_<seconds>.<nanoseconds>.pcd
    // Example: pointcloud_1234567890.123456789.pcd
    //
    // This ensures:
    // - Files are sorted chronologically
    // - No filename collisions
    // - Easy to correlate with rosbag time
    
    std::stringstream filename;
    filename << output_dir_ << "/pointcloud_" 
             << cloud_msg->header.stamp.sec << "."    // Seconds
             << cloud_msg->header.stamp.nsec          // Nanoseconds
             << ".pcd";
    
    // =========================================================================
    // STEP 4: Save point cloud to PCD file
    // =========================================================================
    // PCL provides convenient I/O functions for various formats.
    // savePCDFileBinary() saves in binary format (smaller, faster than ASCII)
    //
    // Other options:
    // - savePCDFileASCII(): Human-readable but larger files
    // - savePCDFile(): Defaults to ASCII
    //
    // Return value: 0 on success, -1 on failure
    
    if (pcl::io::savePCDFileBinary(filename.str(), *pcl_cloud) == 0)
    {
      // Success!
      cloud_count_++;  // Increment counter
      ROS_INFO("Saved point cloud #%d to: %s (%lu points)", 
               cloud_count_, 
               filename.str().c_str(),
               pcl_cloud->points.size());
    }
    else
    {
      // Failed to save - log error
      ROS_ERROR("Failed to save point cloud to: %s", filename.str().c_str());
      ROS_ERROR("Check that directory exists and you have write permissions!");
    }
    
    // =========================================================================
    // OPTIONAL: Additional processing you could add here
    // =========================================================================
    // Students can extend this callback to add features like:
    //
    // 1. Downsampling (reduce point density):
    //    pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
    //    voxel_filter.setInputCloud(pcl_cloud);
    //    voxel_filter.setLeafSize(0.1f, 0.1f, 0.1f);
    //    voxel_filter.filter(*filtered_cloud);
    //
    // 2. Statistical outlier removal (remove noise):
    //    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    //    sor.setInputCloud(pcl_cloud);
    //    sor.setMeanK(50);
    //    sor.setStddevMulThresh(1.0);
    //    sor.filter(*filtered_cloud);
    //
    // 3. Passthrough filter (crop region):
    //    pcl::PassThrough<pcl::PointXYZI> pass;
    //    pass.setInputCloud(pcl_cloud);
    //    pass.setFilterFieldName("z");
    //    pass.setFilterLimits(0.0, 3.0);
    //    pass.filter(*filtered_cloud);
    //
    // 4. Publishing processed cloud back to ROS:
    //    sensor_msgs::PointCloud2 output_msg;
    //    pcl::toROSMsg(*filtered_cloud, output_msg);
    //    cloud_pub_.publish(output_msg);
    //
    // 5. Computing statistics:
    //    Eigen::Vector4f centroid;
    //    pcl::compute3DCentroid(*pcl_cloud, centroid);
    //    ROS_INFO("Cloud centroid: [%.2f, %.2f, %.2f]", 
    //             centroid[0], centroid[1], centroid[2]);
    
    // =========================================================================
    // Note: Callback should return quickly!
    // =========================================================================
    // If processing takes too long, consider:
    // - Using a queue and separate processing thread
    // - Publishing to another topic for downstream processing
    // - Skipping some frames (only process every Nth cloud)
  }
  
}; // End of PCDExtractor class


////////////////////////////////////////////////////////////////////////////////
// MAIN FUNCTION: Entry point of the program
////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Main function - initializes ROS and starts the node
 * @param argc Number of command-line arguments
 * @param argv Array of command-line argument strings
 * @return 0 on successful exit
 * 
 * The main() function is where program execution begins.
 * For a ROS node, it typically:
 * 1. Initialize ROS
 * 2. Create node handle
 * 3. Create node object (which sets up subscribers, publishers, etc.)
 * 4. Spin (process callbacks in a loop)
 * 5. Cleanup and exit
 */
int main(int argc, char** argv)
{
  // ===========================================================================
  // STEP 1: Initialize ROS
  // ===========================================================================
  // ros::init() must be called before using any ROS functionality
  // 
  // Parameters:
  // - argc, argv: Command-line arguments (can contain ROS remappings)
  // - node_name: Name of this node in the ROS graph
  //   * Must be unique if multiple instances run
  //   * Used in logs, topic names, parameter namespace
  //   * Can be remapped: rosrun pkg node __name:=new_name
  //
  // Options (3rd parameter, if needed):
  // - ros::init_options::AnonymousName: Append random number to make unique
  
  ros::init(argc, argv, "pcd_extractor_node");
  
  // ===========================================================================
  // STEP 2: Create NodeHandle
  // ===========================================================================
  // NodeHandle is your interface to ROS communication
  // - Create publishers, subscribers, services
  // - Access parameter server
  // - Manage node lifecycle
  //
  // Different NodeHandle types:
  // - ros::NodeHandle nh: Public namespace (/)
  // - ros::NodeHandle nh("~"): Private namespace (/node_name/)
  // - ros::NodeHandle nh("namespace"): Custom namespace
  
  ros::NodeHandle nh("~");  // Private namespace for parameters
  
  // ===========================================================================
  // STEP 3: Create PCDExtractor object
  // ===========================================================================
  // Constructor is called here, which:
  // - Loads parameters
  // - Creates output directory  
  // - Sets up subscriber
  // - Prints startup messages
  //
  // The object persists until the end of main()
  
  PCDExtractor extractor(nh);
  
  // ===========================================================================
  // STEP 4: Spin (Process callbacks)
  // ===========================================================================
  // ros::spin() enters an infinite loop that:
  // 1. Checks for incoming messages
  // 2. Calls appropriate callback functions
  // 3. Handles ROS events
  // 4. Continues until Ctrl+C or rosnode kill
  //
  // Alternative: ros::spinOnce() + custom loop
  // while(ros::ok()) {
  //   ros::spinOnce();
  //   // Do other work here
  //   loop_rate.sleep();
  // }
  
  ROS_INFO("Spinning... Press Ctrl+C to stop.");
  ros::spin();
  
  // ===========================================================================
  // STEP 5: Cleanup and exit
  // ===========================================================================
  // When spin() returns (node shutdown):
  // - Destructor is called automatically
  // - ROS communications are closed
  // - Node exits gracefully
  
  ROS_INFO("Node terminated.");
  return 0;
}


////////////////////////////////////////////////////////////////////////////////
// SUMMARY FOR STUDENTS
////////////////////////////////////////////////////////////////////////////////
/*

KEY CONCEPTS COVERED:

1. ROS Node Lifecycle:
   - Initialization (ros::init)
   - Setup (NodeHandle, subscribers)
   - Execution (ros::spin)
   - Cleanup (destructor)

2. Topic Subscription:
   - Creating a subscriber
   - Callback functions
   - Message types (sensor_msgs::PointCloud2)

3. ROS Parameters:
   - Reading from parameter server
   - Setting defaults
   - Using private namespace

4. Message Conversion:
   - ROS messages (sensor_msgs::PointCloud2)
   - PCL point clouds (pcl::PointCloud<PointT>)
   - Conversion functions (fromROSMsg, toROSMsg)

5. File I/O:
   - Creating directories
   - Generating filenames
   - Saving PCD files

6. Logging:
   - ROS_INFO, ROS_WARN, ROS_ERROR
   - Different verbosity levels

7. Object-Oriented Programming:
   - Class design
   - Constructor/destructor
   - Member variables and functions
   - Encapsulation

COMMON PITFALLS:

✗ Forgetting to call ros::init()
✗ Not calling ros::spin() (callbacks never execute)
✗ Callback function takes too long (messages get dropped)
✗ Wrong message type in subscriber
✗ Forgetting to source workspace (rosrun doesn't find node)
✗ Directory permissions (can't write PCD files)
✗ Topic name mismatch (no messages received)

TESTING CHECKLIST:

✓ Does the node start without errors?
✓ Is it subscribing to the correct topic? (rostopic info)
✓ Are messages being received? (check logs)
✓ Are PCD files being created? (ls output directory)
✓ Can you visualize the PCD files? (pcl_viewer)
✓ Does the node shutdown cleanly? (Ctrl+C)

EXERCISES FOR EXTENSION:

1. Add a parameter to save only every Nth cloud
2. Add downsampling using VoxelGrid filter
3. Publish filtered clouds to a new topic
4. Add a service to start/stop saving
5. Compute and log statistics (points, bounds, centroid)
6. Support different point types (PointXYZ, PointXYZRGB)
7. Add timestamp in human-readable format to filename
8. Implement compression (save as .pcd.gz)

DEBUGGING TIPS:

# Check if node is running:
$ rosnode list

# Check topics:
$ rostopic list
$ rostopic info /velodyne_points
$ rostopic hz /velodyne_points

# Check parameters:
$ rosparam list
$ rosparam get /pcd_extractor_node/output_directory

# Monitor logs:
$ rosnode info pcd_extractor_node

# Test with live data:
$ rosbag play --pause your_bag.bag
$ (press space to play)

*/
