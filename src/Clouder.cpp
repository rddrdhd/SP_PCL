//
// Created by rddrdhd on 8.5.22.
//
#include "Clouder.h"

Clouder::Clouder() {
    pcl::PointCloud<PointType>::Ptr cloud (new pcl::PointCloud<PointType>);
    this->cloud_ = cloud;
}

Clouder::Clouder(const char* filepath) {
    pcl::PointCloud<PointType>::Ptr cloud (new pcl::PointCloud<PointType>);
    pcl::io::loadPCDFile (filepath, *cloud);
    this->cloud_ = cloud;
}

/*
 * Computes normals - for keypoints. If there are no keypoints yet, computes normals for the whole cloud.
 * */
void Clouder::computeNormals(int k_neigh, float r_neigh) {
    pcl::console::TicToc tt; tt.tic();
    pcl::NormalEstimation<PointType, NormalType> ne;
    pcl::PointCloud<NormalType>::Ptr cloud_point_normals (new pcl::PointCloud<NormalType>);
    pcl::search::KdTree<PointType>::Ptr tree_n(new pcl::search::KdTree<PointType>());

    ne.setInputCloud(this->cloud_);


    //ne.setSearchMethod(tree_n);
    if (k_neigh != 0){
        this->k_normal_neighbours_ = k_neigh;
        ne.setKSearch(k_neigh);
    } else if ( r_neigh != 0.0){
        ne.setRadiusSearch(r_neigh);
    } else {
        ne.setKSearch(this->k_normal_neighbours_);
    }
    //ne.setViewPoint(0,0,70); // for valve
    ne.compute(*cloud_point_normals);

    // Copy the xyz info from cloud_xyz and add it to cloud_point_normals as the xyz field in PointNormals estimation is zero
    for(size_t i = 0; i < cloud_point_normals->points.size(); ++i)
    {
        cloud_point_normals->points[i].x = this->cloud_->points[i].x;
        cloud_point_normals->points[i].y = this->cloud_->points[i].y;
        cloud_point_normals->points[i].z = this->cloud_->points[i].z;
    }
    this->point_normals_ = cloud_point_normals;
    double t = tt.toc();
    pcl::console::print_value( "Computing normals takes %.3f\n[0]:\t", t );
    cout<<cloud_point_normals->points[0]<<endl;
}

pcl::PointCloud<PointType>::Ptr Clouder::getKeypointsXYZ() {
    if(this->keypointsWithScale_.empty()
            and this->keypointsXYZ_.empty()
            and downsampled_cloud_ ) {
       return downsampled_cloud_;
    } else if(!this->keypointsWithScale_.empty()) {
        // Copying the pointwithscale to pointxyz so as visualize the cloud
        pcl::PointCloud<PointType>::Ptr cloud_temp (new pcl::PointCloud<PointType>);
        copyPointCloud(this->keypointsWithScale_, *cloud_temp);
        return cloud_temp;
    } else if(!this->keypointsXYZ_.empty()) {
        pcl::PointCloud<PointType>::Ptr cloud_temp (new pcl::PointCloud<PointType>);
        copyPointCloud(this->keypointsXYZ_, *cloud_temp);
        return cloud_temp;
    } else {
        return nullptr;
    }
}

void Clouder::showNormals() {
    // Visualization of normals along with the original cloud
    pcl::visualization::PCLVisualizer viewer("Normals");
    viewer.setBackgroundColor( 0.0, 0.0, 0.5 );
    viewer.addPointCloud(cloud_, "cloud");
    viewer.addPointCloudNormals<NormalType, NormalType> ( point_normals_,point_normals_, 1, 0.015, "normals");

    pcl::visualization::PointCloudColorHandlerCustom<PointType> cloud_color_handler (cloud_, 255, 200, 0);

    viewer.addPointCloud(cloud_, cloud_color_handler, "cloud");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");


    while(!viewer.wasStopped ())
    {
        viewer.spinOnce ();
    }
}

void Clouder::generateSIFTKeypoints( int octaves, int scales, float min_scale, float min_contrast) {
    pcl::console::TicToc tt;tt.tic();

    // Estimate the sift interest points using normals values from xyz as the Intensity variants
    pcl::SIFTKeypoint<NormalType, pcl::PointWithScale> sift;

    pcl::search::KdTree<NormalType>::Ptr tree(new pcl::search::KdTree<NormalType> ());
    sift.setSearchMethod(tree);
    sift.setMinimumContrast((min_contrast != 0) ? min_contrast : this->min_contrast_);
    sift.setScales((min_scale!=0) ? min_scale : this->min_scale_,
                   (octaves!=0) ? octaves : this->n_octaves_,
                   (scales!=0) ? scales : this->n_scales_per_octave_);

    sift.setInputCloud(this->point_normals_);
    sift.compute(this->keypointsWithScale_);

    double t = tt.toc();
    if(this->keypointsWithScale_.empty()){
        pcl::console::print_value( "Scale-invariant feature transform takes %.3f\n\tBut did not found any keypoints!\n", t);

    } else {
        pcl::console::print_value( "Scale-invariant feature transform takes %.3f\n\tFrom %d to %d points\n", t, this->size(), this->keypointsWithScale_.size());
    }
}

void Clouder::generateHarrisKeypoints(float radius, float radius_search, int method_number ) {
    pcl::console::TicToc tt;tt.tic();
    pcl::HarrisKeypoint3D<PointType,pcl::PointXYZI> harris;
    pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType> ());

    harris.setNonMaxSupression(true);
    harris.setRadius(radius);
    harris.setRadiusSearch(radius_search);
    pcl::HarrisKeypoint3D<PointType,pcl::PointXYZI>::ResponseMethod method;
    std::string method_name;
    switch( method_number ) {
        case 1:
            method = pcl::HarrisKeypoint3D<PointType,pcl::PointXYZI>::HARRIS;
            method_name = "HARRIS";
            break;
        case 2:
            method = pcl::HarrisKeypoint3D<PointType,pcl::PointXYZI>::TOMASI;
            method_name = "TOMASI";
            break;
        case 3:
            method = pcl::HarrisKeypoint3D<PointType,pcl::PointXYZI>::NOBLE;
            method_name = "NOBLE";
            break;
        case 4:
            method = pcl::HarrisKeypoint3D<PointType,pcl::PointXYZI>::LOWE;
            method_name = "LOWE";
            break;
        case 5:
            method = pcl::HarrisKeypoint3D<PointType,pcl::PointXYZI>::CURVATURE;
            method_name = "CURVATURE";
            break;
        default:
            method_name = "INVALID";
            break;
    }
    if(method_name == "INVALID"){
        printf("Invalid Harris method_number (options are 1-5)");
        return;
    }
    harris.setMethod(method);
    harris.setSearchMethod(tree);
    harris.setInputCloud(this->cloud_);
    pcl::PointCloud<pcl::PointXYZI> keypoints;
    harris.compute(keypoints);

    copyPointCloud(keypoints, this->keypointsXYZ_);

    double t = tt.toc();
    if(this->keypointsXYZ_.empty()){
        pcl::console::print_value( "Harris keypoints (%s) takes %.3f\n\tBut did not found any keypoints!\n",method_name.c_str(), t);
    } else {
        pcl::console::print_value( "Harris keypoints (%s) takes %.3f\n\tFrom %d to %d points\n",method_name.c_str(), t, this->size(), this->keypointsXYZ_.size());
    }
}

void Clouder::showKeypoints(){
    // Visualization of keypoints along with the original cloud
    auto keypointsXYZ = getKeypointsXYZ();
    if(keypointsXYZ == nullptr){printf("No keypoints to show\n");return;}
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    pcl::visualization::PointCloudColorHandlerCustom<PointType> keypoints_color_handler (keypointsXYZ, 10, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<PointType> cloud_color_handler (cloud_, 255, 200, 0);


    viewer.setBackgroundColor( 0.0, 0.0, 0.0);

    viewer.addPointCloud(keypointsXYZ, keypoints_color_handler, "keypoints");
    viewer.addPointCloud(cloud_, cloud_color_handler, "cloud");
    //viewer.addPointCloudNormals<PointType, NormalType> ( cloud_, point_normals_, 1, 0.005, "normals");

    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "keypoints");
    //viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
    std::vector<pcl::visualization::Camera> cam;

    while(!viewer.wasStopped ())
    {
        viewer.spinOnce ();
    }

}

void Clouder::generateDownsampledCloud(float downsampling_r){
    pcl::console::TicToc tt;tt.tic();
    pcl::PointCloud<PointType>::Ptr result (new pcl::PointCloud<PointType> ());

    pcl::UniformSampling<PointType> uniform_sampling;
    uniform_sampling.setInputCloud (cloud_);
    if(downsampling_r != 0){
        uniform_sampling.setRadiusSearch (downsampling_r);
    } else {
        uniform_sampling.setRadiusSearch (this->downsampling_radius_);
    }
    uniform_sampling.filter (*result);
    this->downsampled_cloud_ = result;
    double t = tt.toc();
    pcl::console::print_value( "Downsampling the cloud takes %.3f\n\tFrom %d to %d points\n", t, this->size(), this->downsampled_cloud_->size() );
}

void Clouder::computeFPFHDescriptors(int k_neighbours) {
    pcl::console::TicToc tt; tt.tic();

    pcl::FPFHEstimationOMP<PointType, NormalType, FPFHType> fpfh;

    auto keypoints = getKeypointsXYZ();
    if(keypoints == nullptr or keypoints->empty()){
        printf("No keypoints to describe!");
        return;
    }
    fpfh.setInputCloud (keypoints);
    fpfh.setInputNormals (this->point_normals_);
    fpfh.setSearchSurface(this->cloud_);

    pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
    fpfh.setSearchMethod (tree);

    pcl::PointCloud<FPFHType>::Ptr features (new pcl::PointCloud<FPFHType> ());

    // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
    int k = (k_neighbours!=0) ? k_neighbours : this->descriptor_k_neighbours_;
    if (k < this->k_normal_neighbours_) {printf("Use bigger k in FPFH than for normals, %d < %d!\n", k, this->k_normal_neighbours_);return;}
    fpfh.setKSearch(k);

    fpfh.compute (*features);
    this->FPFH_descriptors_ = features;

    double t = tt.toc();
    pcl::console::print_value( "Fast Persistent Feature Histogram takes %.3f\n[0]:\t", t );
    cout<<features->points[0]<<endl;
}

void Clouder::computePFHDescriptors(){ // TODO generating all-zero desriptors
    pcl::console::TicToc tt;tt.tic();
    pcl::PFHEstimation<PointType, NormalType, PFHType> pfh;

    auto keypoints = getKeypointsXYZ();
    if(keypoints == nullptr){
        printf("No keypoints!");
        return;
    }
    pfh.setInputCloud (keypoints);
    pfh.setInputNormals (this->point_normals_);
    pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType> ());
    pfh.setSearchMethod (tree);

    pcl::PointCloud<PFHType>::Ptr features (new pcl::PointCloud<PFHType> ());

    pfh.setRadiusSearch (descriptor_radius_);
    //pfh.setKSearch(descriptor_k_neighbours_);

    // WIP: returning nans :(
    pfh.compute(*features);
    this->PFH_descriptors_ = features;

    double t = tt.toc();
    pcl::console::print_value( "Persistent Feature Histogram takes %.3f\n[0]:\t", t );
    cout << features->points[0] << endl;
}

void Clouder::computeSHOTDescriptors(){
    pcl::console::TicToc tt;tt.tic();
    auto keypoints = getKeypointsXYZ();
   /* pcl::PointCloud<pcl::ReferenceFrame>::Ptr frames (new pcl::PointCloud<pcl::ReferenceFrame> ());
    pcl::SHOTLocalReferenceFrameEstimation<PointType, pcl::ReferenceFrame> lrf_estimator;
    lrf_estimator.setRadiusSearch (descriptor_radius_);
    lrf_estimator.setInputCloud (keypoints);
    lrf_estimator.setSearchSurface(cloud_);
    lrf_estimator.compute (*frames);
*/

    pcl::SHOTEstimationOMP<PointType, NormalType, SHOTType> shot;
    pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType> ());
    shot.setSearchMethod(tree);

    shot.setRadiusSearch (this->descriptor_radius_);
    shot.setInputNormals (this->point_normals_);
    shot.setInputCloud (keypoints);
    shot.setSearchSurface(this->cloud_);

    //shot.setLRFRadius(rf_rad_);
    //shot.setInputReferenceFrames(frames);

    pcl::PointCloud<SHOTType>::Ptr features (new pcl::PointCloud<SHOTType> ());

    shot.compute(*features);
    this->SHOT_descriptors_ = features;

    double t = tt.toc();
    pcl::console::print_value( "Signature of Histograms of Orientation takes %.3f\n", t );
    cout << features->points[0] << endl;
}

void Clouder::findKDTreeCorrespondencesFromSHOT(Clouder model){
    std::cout << "Finding Model-Scene Correspondences with KdTree" << std::endl << std::endl;
    pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());

    pcl::KdTreeFLANN<SHOTType> match_search;
    auto model_descriptors = model.SHOT_descriptors_;
    auto scene_descriptors = this->SHOT_descriptors_;
    match_search.setInputCloud (model_descriptors);

    //  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
    for (std::size_t i = 0; i < scene_descriptors->size (); ++i)
    {
        std::vector<int> neigh_indices (1);
        std::vector<float> neigh_sqr_dists (1);
        if (!std::isfinite (scene_descriptors->at (i).descriptor[0])) //skipping NaNs
        {
            continue;
        }
        int found_neighs = match_search.nearestKSearch (scene_descriptors->at (i), 1, neigh_indices, neigh_sqr_dists);
        if(found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
        {
            pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
            model_scene_corrs->push_back (corr);
        }
        this->correspondences_ = model_scene_corrs;
    }
    std::cout << "Correspondences found: " << model_scene_corrs->size () << std::endl;
}
/*
pcl::PointCloud<PointType>::Ptr Clouder::getDenseKeypointsXYZ() {

    pcl::PointCloud<PointType>::Ptr keypoints_filtered;
    pcl::RadiusOutlierRemoval<PointType> outrem;
    // build the filter
    auto kp = this->getKeypointsXYZ();
    outrem.setInputCloud(kp);
    outrem.setRadiusSearch(1.2);
    outrem.setMinNeighborsInRadius (2);
    outrem.setKeepOrganized(true);
    // apply filter
    outrem.filter (keypoints_filtered);
    return keypoints_filtered;
}*/
