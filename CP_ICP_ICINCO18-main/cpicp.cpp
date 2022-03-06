#include <algorithm>
#include <string> 
#include <vector>
#include <pcl/registration/registration.h>
//#include <pcl/correspondence.h>
#include <pcl/point_cloud.h>
//#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/visualization/common/common.h>
#include <pcl/common/common.h>
#include <pcl/search/kdtree.h>
#include <math.h>       /* acos */
#define PI 3.14159265
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/console/print.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
using namespace std;
using std::vector;

typedef pcl::search::KdTree<pcl::PointXYZ> SearchMethod;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

bool comparePoint(pcl::PointXYZ p1, pcl::PointXYZ p2);

bool comparePoint(pcl::PointXYZ p1, pcl::PointXYZ p2) {

    if ( p1.z <  p2.z) { //
        return true;
    }
    else{
        return false;
    }
}

vector<double> min(vector<double> vec){
    double m = vec[0];
    int n = vec.size();
    double k = 0;
    for(int i=0; i<n; i++){
        if(vec[i]<m){
            m = vec[i];
            k = i;
        }
    }

    vector <double> vf;
    vf.push_back(k);
    vf.push_back(m);
    return vf;
}


double computeCloudRMS(pcl::PointCloud<pcl::PointXYZ>::ConstPtr target, pcl::PointCloud<pcl::PointXYZ>::ConstPtr source, double max_range){

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(target);

    double fitness_score = 0.0;

    std::vector<int> nn_indices (1);
    std::vector<float> nn_dists (1);

    // For each point in the source dataset
    int nr = 0;
    for (size_t i = 0; i < source->points.size (); ++i){
        //Avoid NaN points as they crash nn searches
        if(!pcl_isfinite((*source)[i].x)){
            continue;
        }

        // Find its nearest neighbor in the target
        tree->nearestKSearch (source->points[i], 1, nn_indices, nn_dists);

        // Deal with occlusions (incomplete targets)
        if (nn_dists[0] <= max_range*max_range){
            //            // Add to the fitness score
            fitness_score += nn_dists[0];
            nr++;
        }
    }

    if (nr > 0){
        return sqrt(fitness_score / nr);
    }else{
        return (std::numeric_limits<double>::max ());
    }
}


std::vector<PointCloud::Ptr> cloudPartitioning(PointCloud::Ptr cloud, int n){	
    //float s = cloud->height;

    float s = cloud->width;
    float num = floor(s/n);
    //Sorting
    std::sort(cloud->points.begin(),cloud->points.end(),comparePoint);

    //Creating subclouds
    vector<PointCloud::Ptr> sn;
    for(int j=0;j<n;j++){
        PointCloud::Ptr sn1 (new PointCloud);
        //nuvens de faces/*
        //sn1->width = 1;
        //sn1->height = num;

        //sn1->resize(sn1->width*sn1->height);

        sn1->width = num;
        sn1->height = 1;
        //sn1->resize(sn1->width*sn1->height);

        for(int i=j*num ; i< (j+1)*num ; i++){
            sn1->points.push_back(cloud->points[i]);
        }
        sn.push_back(sn1);
    }
    //    std::cout << "Size of sn: " << sn.size() << endl;
    return sn;
}

int
main (int argc, char** argv)
{

    PointCloud::Ptr cloud_in (new PointCloud);
    PointCloud::Ptr cloud_out (new PointCloud);

    PointCloud::Ptr cloud_icp (new PointCloud);
    PointCloud::Ptr cloud_regist (new PointCloud);


    if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *cloud_out) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file model \n");
        return (-1);
    }


    if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[2], *cloud_in) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file shape \n");
        return (-1);
    }

  

    // Applying cloud partitioning
    int num = atoi (argv[3]);

    vector <PointCloud::Ptr> sn_in;
    vector <PointCloud::Ptr> sn_out;


    float rmse_stop_criterium = atof(argv[4]);
    int iteration_icp = atoi(argv[5]);
  //  float distance = atof(argv[6]);

    int flag_break=0;

    for(int n= 2; n<num+1; n++){

        clock_t tempo;
        tempo = clock();
        sn_in = cloudPartitioning(cloud_in,n);
        sn_out = cloudPartitioning(cloud_out,n);

        vector <PointCloud::Ptr> sn_final;
        vector <double> rms;
        Eigen::Matrix4f rotation_matrix;
 //for(int i=1; i<n; i++){
  //for(int i=n-1; i>0; i=i-1){
        for(int j=1; j<n; j++){

            pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
            icp.setInputSource(sn_in[j]);
            icp.setInputTarget(sn_out[j]);
            icp.setMaximumIterations(iteration_icp);
          //  icp.setTransformationEpsilon (1e-5);
          //  icp.setMaxCorrespondenceDistance (distance);
           // icp.setEuclideanFitnessEpsilon (1);
           // icp.setRANSACOutlierRejectionThreshold (1.5);
            icp.align(*cloud_icp);

           pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final (new pcl::PointCloud<pcl::PointXYZ>);
           Eigen::Matrix4f transform_icp = icp.getFinalTransformation();
           pcl::transformPointCloud (*cloud_in, *cloud_final, transform_icp);
            sn_final.push_back(cloud_icp);
            double this_rms = computeCloudRMS(cloud_out,cloud_final,std::numeric_limits<double>::max ());
            rms.push_back(this_rms);
            //cout << "RMSE " << this_rms <<": " << endl;
            /////////////////////////////////////////////////////////////////////////////////////////
          //  Eigen::Matrix4f rotation_matrix = Eigen::Matrix4f::Identity();
            if(this_rms< rmse_stop_criterium){
                rotation_matrix = icp.getFinalTransformation();
      		 cout << rotation_matrix << endl;
             //   pcl::transformPointCloud (*cloud_in,*cloud_regist, rotation_matrix);
                flag_break=1; break;
            }


        }
        vector <double> rms_min = min(rms);

        double elem1, elem2, elem3, angle123,axis_angle;
        elem1 = rotation_matrix(0,0);

        elem2 = rotation_matrix(1,1);
        elem3= rotation_matrix(2,2);
        angle123= (elem1+elem2+elem3-1)/2;
        axis_angle = acos (angle123) * 180.0 / PI;

        double trans1, trans2, trans3, error_trans, trans12, trans22, trans32;
        trans1 = rotation_matrix(0,3);
        trans2 = rotation_matrix(1,3);
        trans3 = rotation_matrix(2,3);
        trans12= trans1*trans1;
        trans22= trans2*trans2;
        trans32= trans3*trans3;
        error_trans=sqrt(trans12+trans22+trans32);


      // cout << "Iteracao  " << n << ": " << endl;
       cout << "Rotation (degree) " << axis_angle<<":  " << endl;
       cout << "Translacao (meters) " << error_trans<<":  " << endl;

       cout << "RMSE " << rms_min[1] <<": " << endl;
       cout << rotation_matrix << endl;
        printf("Time:%f \n",(clock() - tempo) / (double)CLOCKS_PER_SEC);
        cout << "-------------------------------------------------------" << endl;

        if(flag_break==1){

            break;
        }
    }  // Laço n
//  pcl::io::savePCDFileASCII (argv[6], *cloud_regist);

    //---------------------------
    //Visualização
    //---------------------------

    int sizePoints = 2;

    //    // Adicionando ambas as nuvens de pontos

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1 (new pcl::visualization::PCLVisualizer ("REGISTRO"));

    // Adicionando ambas as nuvens de pontos
    viewer1->setBackgroundColor(255,255,255);

    viewer1->addPointCloud(cloud_out, "cloud_out");
    viewer1->addPointCloud(cloud_in, "cloud_in");

    //Configurando cloud_out
    viewer1->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0,0,255,"cloud_in");
    viewer1->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, sizePoints, "cloud_in");
    viewer1->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0, 255, 0,"cloud_out");
    viewer1->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, sizePoints, "cloud_out");
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2 (new pcl::visualization::PCLVisualizer ("REGISTRO"));

    // Adicionando ambas as nuvens de pontos
    viewer2->setBackgroundColor(255,255,255);

    viewer2->addPointCloud(cloud_out, "cloud_out");
    viewer2->addPointCloud(cloud_icp, "cloud_regist");

    //Configurando cloud_out
    viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0,0,255,"cloud_regist");
    viewer2->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, sizePoints, "cloud_regist");
    viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0, 255, 0,"cloud_out");
    viewer2->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, sizePoints, "cloud_out");
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    while(!viewer2->wasStopped())
    {
        viewer2->spinOnce();
     //   boost::this_thread::sleep (boost::posix_time::microseconds(100000));
    }
    system("pause");
}
