/*********************************************************************************************************************
Main Function for point cloud registration with Normal DIstribution transform
Last modified: August 24, 2020

Reference:
Magnusson, M. (2009). The Three-Dimensional Normal-Distributions Transform — 
an Efﬁcient Representation for Registration, Surface Analysis, and Loop Detection. PhD thesis, Orebro University.

Responsible for implementation: 
Documentation: https://pointclouds.org/documentation/classpcl_1_1_normal_distributions_transform.html#details
**********************************************************************************************************************/
#include "validationlib.h"


int
main (int argc, char** argv)
{
    clock_t tempo;
    tempo = clock();

    // Loading first scan of room.
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *target_cloud) == -1)
    {
        PCL_ERROR ("Couldn't read file room_scan1.pcd \n");
        return (-1);
    }
    std::cout << "Loaded " << target_cloud->size () << " data points from room_scan1.pcd" << std::endl;

    // Loading second scan of room from new perspective.
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[2], *input_cloud) == -1)
    {
        PCL_ERROR ("Couldn't read file room_scan2.pcd \n");
        return (-1);
    }


    // Filtering input scan to roughly 10% of original size to increase speed of registration.
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
    approximate_voxel_filter.setLeafSize (0.2, 0.2, 0.2);
    approximate_voxel_filter.setInputCloud (input_cloud);
    approximate_voxel_filter.filter (*filtered_cloud);

    // Initializing Normal Distributions Transform (NDT).
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;

    // Setting scale dependent NDT parameters
    // Setting minimum transformation difference for termination condition.
    //ndt.setTransformationEpsilon (0.01);
    // Setting maximum step size for More-Thuente line search.
    float step = atof(argv[3]);
    float resolution = atof(argv[4]);
    int iteration = atoi(argv[5]);

    ndt.setStepSize(step); // ORIGINAL POLY
    ndt.setResolution(resolution); // ORIGINAL POLY
    ndt.setMaximumIterations(iteration); // ORIGINAL POLY

    // Setting point cloud to be aligned.
    ndt.setInputSource (filtered_cloud);
    // Setting point cloud to be aligned to.
    ndt.setInputTarget (target_cloud);

    // Set initial alignment estimate found using robot odometry.
//    Eigen::AngleAxisf init_rotation (0.6931, Eigen::Vector3f::UnitZ ());
//    Eigen::Translation3f init_translation (1.79387, 0.720047, 0);
//    Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix ();

    //    Eigen::AngleAxisf init_rotation (0.0174, Eigen::Vector3f::UnitZ ());
//    Eigen::Translation3f init_translation (1.79387, 0.720047, 0);
//    Eigen::Matrix3f init_guess = (init_translation * init_rotation).matrix ();


    // Calculating required rigid transform to align the input cloud to the target cloud.
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    ndt.align (*output_cloud);
    // Calculation of final RMSE
    double rms = computeCloudRMS(target_cloud, output_cloud, std::numeric_limits<double>::max ());
    std::cout << "RMS: " << rms << std::endl;
   
    // Transforming unfiltered, input cloud using found transform.
    pcl::transformPointCloud (*input_cloud, *output_cloud, ndt.getFinalTransformation ());
    Eigen::Matrix4f rotation_matrix;
    rotation_matrix=ndt.getFinalTransformation ();
  
      //Calculation of rotation after alignment    
        double elem1 = rotation_matrix(0, 0); // r_11
        double elem2 = rotation_matrix(1, 1); // r_22
        double elem3 = rotation_matrix(2, 2); // r_33
        double angle123 = (elem1 + elem2 + elem3 - 1) / 2.0; // Axis–angle representation==(sum of the main diagonal - 1)/2
        double result = acos (angle123) * 180.0 / PI;

    std::cout << "ROTATION: " << result << std::endl;
    printf("TIME:%f \n",(clock() - tempo) / (double)CLOCKS_PER_SEC);
    cout << "-------------------------------------------------------" << endl;
  pcl::io::savePCDFileASCII (argv[6], *output_cloud);
    // Initializing point cloud visualizer
    boost::shared_ptr<pcl::visualization::PCLVisualizer>
            viewer_final (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer_final->setBackgroundColor (255, 255, 255);

    // Coloring and visualizing target cloud (blue).
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
            target_color (target_cloud, 0, 0, 255);
    viewer_final->addPointCloud<pcl::PointXYZ> (target_cloud, target_color, "target cloud");
    viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                    1, "target cloud");

    // Coloring and visualizing transformed input cloud (green).
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
            output_color (output_cloud, 0, 255, 0);
    viewer_final->addPointCloud<pcl::PointXYZ> (output_cloud, output_color, "output cloud");
    viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                    1, "output cloud");

    // Starting visualizer
    // viewer_final->addCoordinateSystem (1.0, "global");
    viewer_final->initCameraParameters ();

    // Wait until visualizer window is closed.
    while (!viewer_final->wasStopped ())
    {
        viewer_final->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }

    return (0);
}
