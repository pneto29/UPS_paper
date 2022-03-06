/*********************************************************************************************************************
Main Function for point cloud registration with Four Point COngruent Sets
Last modified: July 19, 2021

Reference:
Aiger, Dror, Niloy J. Mitra, and Daniel Cohen-Or. "4-points congruent sets for robust pairwise surface registration." ACM SIGGRAPH 2008 papers. 2008. 1-10.

Responsible for implementation:
Documentation: https://pointclouds.org/documentation/classpcl_1_1registration_1_1_f_p_c_s_initial_alignment.html
**********************************************************************************************************************/

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>  
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/console/time.h>   // 
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/math/special_functions/round.hpp>
#include <pcl/registration/ia_fpcs.h>

#include "validationlib.h"
using namespace std;
int
main(int argc, char** argv)
{
    pcl::console::TicToc time;
    //
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *target_cloud) == -1)
    {
        PCL_ERROR("Failed to read target cloud\n");
        return (-1);
    }

    //
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[2], *input_cloud) == -1)
    {
        PCL_ERROR("Failed to read source cloud  \n");
        return (-1);
    }

    time.tic();
    //inicialization
    pcl::registration::FPCSInitialAlignment<pcl::PointXYZ, pcl::PointXYZ> sfpcs;
    sfpcs.setInputSource(input_cloud);  //
    sfpcs.setInputTarget(target_cloud);  //
    sfpcs.setApproxOverlap(0.4);// Defina a sobreposição aproximada entre a origem e o destino.
    //  sfpcs.setLambda(0.5);
    //  sfpcs.setDelta(0.2);//Incremento de fator constante para ponderar parâmetros de cálculo internos.
    //sfpcs.setMaxComputationTime(1000);//Defina o tempo máximo de cálculo (em segundos).
    sfpcs.setNumberOfSamples(1000); //Defina o número de amostras de nuvem de ponto de origem a serem usadas durante o registro
    pcl::PointCloud<pcl::PointXYZ>::Ptr fpcs(new pcl::PointCloud<pcl::PointXYZ>);
    sfpcs.align(*fpcs);
    cout << "sfpcs time registration： " << time.toc() << " ms" << endl;
    cout << "Rotation matrix：\n" << sfpcs.getFinalTransformation() << endl;


    // Use the created transformation to transform the source point cloud
    pcl::transformPointCloud(*input_cloud, *fpcs, sfpcs.getFinalTransformation());

    double rms = computeCloudRMS(target_cloud, fpcs, std::numeric_limits<double>::max ());
    std::cout << "RMS: " << rms << std::endl;

    // Save the converted source point cloud as the final transformed output
    //  pcl::io::savePCDFileASCII ("room_scan2_transformed.pcd", *output_cloud);

    //
    boost::shared_ptr<pcl::visualization::PCLVisualizer>viewer(new pcl::visualization::PCLVisualizer("Mostrar nuvem de pontos"));
    viewer->setBackgroundColor(0, 0, 0);  //

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>target_color(target_cloud, 255, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(target_cloud, target_color, "target cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target cloud");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>input_color(input_cloud, 0, 0, 255);
    viewer->addPointCloud<pcl::PointXYZ>(input_cloud, input_color, "input cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "input cloud");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
            output_color(fpcs, 0, 255, 0);
    viewer->addPointCloud<pcl::PointXYZ>(fpcs, output_color, "output cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "output cloud");


    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        // boost::this_thread::sleep(boost::posix_time::microseconds(1000));
    }

    return (0);
}
