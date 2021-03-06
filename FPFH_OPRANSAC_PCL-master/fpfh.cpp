#include "validationlib.h"

class FeatureCloud
{
	public:
		typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
		typedef pcl::PointCloud<pcl::Normal> SurfaceNormals;
		typedef pcl::PointCloud<pcl::FPFHSignature33> LocalFeatures;
		typedef pcl::search::KdTree<pcl::PointXYZ> SearchMethod;

		FeatureCloud () :
			search_method_xyz_(new SearchMethod),
            normal_radius_(0.03f),
            feature_radius_(0.02f)
		{}

		~FeatureCloud () {}
		
		void setInputCloud(PointCloud::Ptr xyz) {
			xyz_ = xyz;
			processInput();
		}
		
		void loadInputCloud(const std::string &pcd_file) {
			xyz_ = PointCloud::Ptr(new PointCloud);
			pcl::io::loadPCDFile(pcd_file, *xyz_);
			processInput();
		}
		
		PointCloud::Ptr getPointCloud() const {
			return xyz_;
		}
		
		SurfaceNormals::Ptr getSurfaceNormals() const {
		  return normals_;
		}
		
		LocalFeatures::Ptr getLocalFeatures() const {
			return features_;
		}

	protected:
		void processInput() {
			computeSurfaceNormals();
			computeLocalFeatures();
		}
		
		void computeSurfaceNormals() {
			normals_ = SurfaceNormals::Ptr(new SurfaceNormals);
		  	
		  	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;
		  	norm_est.setInputCloud(xyz_);
		  	norm_est.setSearchMethod(search_method_xyz_);
		  	norm_est.setRadiusSearch(normal_radius_);
		  	norm_est.compute(*normals_);
		}
		
		void computeLocalFeatures() {
		  	features_ = LocalFeatures::Ptr(new LocalFeatures);
			
		  	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
		  	fpfh_est.setInputCloud(xyz_);
		  	fpfh_est.setInputNormals(normals_);
		  	fpfh_est.setSearchMethod(search_method_xyz_);
		  	fpfh_est.setRadiusSearch(feature_radius_);
		  	fpfh_est.compute(*features_);
		}

	private:
		PointCloud::Ptr xyz_;
		SurfaceNormals::Ptr normals_;
		LocalFeatures::Ptr features_;
		SearchMethod::Ptr search_method_xyz_;
		float normal_radius_;
		float feature_radius_;
};

class TemplateAlignment
{
	public:
		struct Result {
		  	float fitness_score;
		  	Eigen::Matrix4f final_transformation;
		  	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		};

		TemplateAlignment () :
            min_sample_distance_(0.05f),
            max_correspondence_distance_(0.01f*0.01f),
            nr_iterations_(500)
		{
		  	sac_ia_.setMinSampleDistance(min_sample_distance_);
		  	sac_ia_.setMaxCorrespondenceDistance(max_correspondence_distance_);
		  	sac_ia_.setMaximumIterations(nr_iterations_);
		}

		~TemplateAlignment () {}
		
		void setTargetCloud(FeatureCloud &target_cloud) {
		  	target_ = target_cloud;
		  	sac_ia_.setInputTarget(target_cloud.getPointCloud());
		  	sac_ia_.setTargetFeatures(target_cloud.getLocalFeatures());
		}
		
		void addTemplateCloud(FeatureCloud &template_cloud) {
		  	templates_.push_back(template_cloud);
		}
		
		void align(FeatureCloud &template_cloud, TemplateAlignment::Result &result) {
		  	sac_ia_.setInputSource(template_cloud.getPointCloud());
		  	sac_ia_.setSourceFeatures(template_cloud.getLocalFeatures());

		  	pcl::PointCloud<pcl::PointXYZ> registration_output;
		  	sac_ia_.align (registration_output);

		  	result.fitness_score = (float)sac_ia_.getFitnessScore(max_correspondence_distance_);
		  	result.final_transformation = sac_ia_.getFinalTransformation();
		}
		
		void alignAll(std::vector<TemplateAlignment::Result, Eigen::aligned_allocator<Result> > &results) {
		  	results.resize (templates_.size());
		  	
		  	for (size_t i = 0; i < templates_.size(); ++i)
		    	align(templates_[i], results[i]);
		}
		
		int findBestAlignment(TemplateAlignment::Result &result) {
		  	std::vector<Result, Eigen::aligned_allocator<Result> > results;
		  	alignAll(results);

		  	float lowest_score = std::numeric_limits<float>::infinity();
		  	int best_template = 0;
		  	for (size_t i = 0; i < results.size(); ++i) {
				const Result &r = results[i];
				
				if (r.fitness_score < lowest_score) {
				  lowest_score = r.fitness_score;
				  best_template = (int)i;
				}
		  	}
		  	
		  	result = results[best_template];
		  	return best_template;
		}

	private:
		std::vector<FeatureCloud> templates_;
		FeatureCloud target_;
		pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia_;
		float min_sample_distance_;
		float max_correspondence_distance_;
		int nr_iterations_;
};

int main(int argc, char **argv) {
	if (argc < 3) {
		printf("Missing parameters\n");
		printf("Usage: fpfh <templates_file> <target>\n"); // ?????
		
		return -1;
	}
	
	clock_t tempo;
    tempo = clock();
	
  	// Load the object templates specified in the object_templates.txt file
  	std::vector<FeatureCloud> object_templates;
  	std::ifstream input_stream(argv[1]);
  	object_templates.resize(0);
  	std::string pcd_filename;
  	
  	while (input_stream.good()) {
    	std::getline(input_stream, pcd_filename);
    	
    	if (pcd_filename.empty() || pcd_filename.at(0) == '#')
      		continue;
      	
    	FeatureCloud template_cloud;
    	template_cloud.loadInputCloud(pcd_filename);
    	object_templates.push_back(template_cloud);
	}
	
  	input_stream.close();
	
  	// Load the target cloud PCD file
  	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  	pcl::io::loadPCDFile(argv[2], *cloud);
	
 	// Preprocess the cloud by...
  	// ...removing distant points
//  	const float depth_limit = 1.0;
//  	pcl::PassThrough<pcl::PointXYZ> pass;
//  	pass.setInputCloud(cloud);
//  	pass.setFilterFieldName("z");
//  	pass.setFilterLimits(0, depth_limit);
//  	pass.filter(*cloud);
	
  	// ...and downsampling the point cloud
    const float voxel_grid_size = 0.05f;
    pcl::VoxelGrid<pcl::PointXYZ> vox_grid;
    vox_grid.setInputCloud(cloud);
    vox_grid.setLeafSize(voxel_grid_size, voxel_grid_size, voxel_grid_size);
    pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZ>);
    vox_grid.filter(*tempCloud);
    cloud = tempCloud;

  	// Assign to the target FeatureCloud
  	FeatureCloud target_cloud;
  	target_cloud.setInputCloud(cloud);
	
  	// Set the TemplateAlignment inputs
  	TemplateAlignment template_align;
  	for (size_t i = 0; i < object_templates.size(); ++i)
    	template_align.addTemplateCloud(object_templates[i]);
  	
  	template_align.setTargetCloud(target_cloud);

  	// Find the best template alignment
  	TemplateAlignment::Result best_alignment;
  	int best_index = template_align.findBestAlignment(best_alignment);
  	const FeatureCloud &best_template = object_templates[best_index];
  	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new PointCloud);
  	pcl::transformPointCloud(*best_template.getPointCloud(), *transformed_cloud, best_alignment.final_transformation);
    Eigen::Matrix3f rotation = best_alignment.final_transformation.block<3,3>(0, 0);



    // Save the aligned template for visualization
    //pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
  //  pcl::transformPointCloud (*best_template.getPointCloud (), transformed_cloud, best_alignment.final_transformation);

  	// ICP BEGIN -----------------------------------------------------------
  	PointCloud::Ptr icp_cloud(new PointCloud);
  	
  	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(transformed_cloud);
    icp.setInputTarget(cloud);
    icp.setMaximumIterations(30);
    icp.align(*icp_cloud);
    std::cout << "TIME: " << (clock() - tempo) / (double)CLOCKS_PER_SEC << std::endl;

    pcl::io::savePCDFileBinary ("output.pcd", *icp_cloud);

  //  pcl::io::savePCDFileASCII ("test_pcd.pcd", cloud);

  	// ICP END -------------------------------------------------------------
 //  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud2 (new pcl::PointCloud<pcl::PointXYZ> ());
     // You can either apply transform_1 or transform_2; they are the same
 //    pcl::transformPointCloud (*cloud, *transformed_cloud2, best_alignment.final_transformation);
  	//Eigen::Matrix4f rotation_matrix = best_alignment.final_transformation;

  	Eigen::Matrix4f rotation_matrix = icp.getFinalTransformation();
  	double rms = computeCloudRMSE(cloud, icp_cloud, std::numeric_limits<double>::max());
  	double elem1 = rotation_matrix(0, 0);
    double elem2 = rotation_matrix(1, 1);
    double elem3 = rotation_matrix(2, 2);
    double angle123 = (elem1 + elem2 + elem3 - 1) / 2.0;
    double rot_angle = (acos(angle123) * 180.0) / PI;
    

    double fpfh1 = rotation(0, 0);
    double fpfh2 = rotation(1, 1);
    double fpfh3 = rotation(2, 2);
    double fpfh123 = (fpfh1 + fpfh2 + fpfh3 - 1) / 2.0;
    double rot_angle2 = (acos(fpfh123) * 180.0) / PI;

    std::cout << rotation_matrix << std::endl;
    std::cout << "RMSE: " << rms << std::endl;
    std::cout << "ANGL_ TOTAL: " << rot_angle2 + rot_angle << std::endl;
    std::cout << "FPFH_ROT: " << rot_angle << std::endl;
    std::cout << "ICP_ROT: " << rot_angle2 << std::endl;

  	return (0);
}

