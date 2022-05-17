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
    // Estimate the normals of the cloud_xyz

    pcl::NormalEstimation<PointType, NormalType> ne;
    pcl::PointCloud<NormalType>::Ptr cloud_point_normals (new pcl::PointCloud<NormalType>);
    pcl::search::KdTree<PointType>::Ptr tree_n(new pcl::search::KdTree<PointType>());
    auto keypoints = downsampled_cloud_;//getKeypointsXYZ();
    if( keypoints){
        ne.setInputCloud( keypoints);
    } else {
        ne.setInputCloud(this->cloud_);
    }
    ne.setSearchMethod(tree_n);
    if (k_neigh != 0){
        ne.setKSearch(k_neigh);
    } else if ( r_neigh != 0.0){
        ne.setRadiusSearch(r_neigh);
    } else {
        ne.setKSearch(this->k_neighbours_);
    }
    ne.setViewPoint(0,0,70);
    //ne.setRadiusSearch(0.01f);
    ne.compute(*cloud_point_normals);

    // Copy the xyz info from cloud_xyz and add it to cloud_point_normals as the xyz field in PointNormals estimation is zero
    for(size_t i = 0; i < cloud_point_normals->points.size(); ++i)
    {
        cloud_point_normals->points[i].x = this->cloud_->points[i].x;
        cloud_point_normals->points[i].y = this->cloud_->points[i].y;
        cloud_point_normals->points[i].z = this->cloud_->points[i].z;

    }
    if( keypoints){
        this->key_point_normals_ = cloud_point_normals;
    } else {
        this->point_normals_ = cloud_point_normals;
    }

    double t = tt.toc();
    pcl::console::print_value( "Computing normals takes %.3f\n[0]:\t", t );
    cout<<cloud_point_normals->points[0]<<endl;
}

pcl::PointCloud<PointType>::Ptr Clouder::getKeypointsXYZ(){
    if(this->keypoints_.empty()){
       return nullptr;
    } else {
        // Copying the pointwithscale to pointxyz so as visualize the cloud
        pcl::PointCloud<PointType>::Ptr cloud_temp (new pcl::PointCloud<PointType>);
        copyPointCloud(this->keypoints_, *cloud_temp);
        return cloud_temp;
    }
};

void Clouder::showNormals() {
    // Visualization of normals along with the original cloud
    pcl::visualization::PCLVisualizer viewer("Normals");
    viewer.setBackgroundColor( 0.0, 0.0, 0.5 );

    auto keypoints = getKeypointsXYZ();
    if (keypoints){
        viewer.addPointCloudNormals<NormalType, NormalType> ( key_point_normals_,key_point_normals_, 1, 5, "normals");

    } else {
        viewer.addPointCloudNormals<NormalType, NormalType> ( point_normals_,point_normals_, 1, 5, "normals");

    }

    viewer.initCameraParameters();
    viewer.setCameraPosition(-447.737, -98.0556, 21.3569,    0,0,70,   0,0,1);
    viewer.setCameraFieldOfView(0.523599);
    viewer.setCameraClipDistances(0.05, 600);

    while(!viewer.wasStopped ())
    {
        viewer.spinOnce ();
    }
}

void Clouder::generateSIFTKeypoints() {
    pcl::console::TicToc tt;tt.tic();

    // Estimate the sift interest points using normals values from xyz as the Intensity variants
    pcl::SIFTKeypoint<NormalType, pcl::PointWithScale> sift;

    pcl::search::KdTree<NormalType>::Ptr tree(new pcl::search::KdTree<NormalType> ());
    sift.setSearchMethod(tree);
    sift.setMinimumContrast(this->min_contrast_);
    sift.setScales(this->min_scale_, this->n_octaves_, this->n_scales_per_octave_);

    sift.setInputCloud(this->point_normals_);
    sift.compute(this->keypoints_);

    double t = tt.toc();
    pcl::console::print_value( "Scale-invariant feature transform takes %.3f\n\tFrom %d to %d points\n", t, this->size(), this->getKeypointsXYZ()->size());
}

void Clouder::showKeypoints(){
    // Visualization of keypoints along with the original cloud
    auto keypointsXYZ = downsampled_cloud_;//getKeypointsXYZ();

    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    pcl::visualization::PointCloudColorHandlerCustom<PointType> keypoints_color_handler (keypointsXYZ, 10, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<NormalType> cloud_color_handler (point_normals_, 255, 200, 0);

    viewer.initCameraParameters();
    viewer.setCameraPosition(-447.737, -98.0556, 21.3569,    0,0,70,   0,0,1);
    viewer.setCameraFieldOfView(0.523599);
    viewer.setCameraClipDistances(0.05, 600);

/*
 *  - pos: (-538.548, -85.0334, 19.5809)
 - view: (-0.0877812, 0.0478142, 0.994992)
 - focal: (14.7891, 30.568, 62.8427)
 */

    viewer.setBackgroundColor( 0.0, 0.0, 0.0);
    viewer.addPointCloud(point_normals_, cloud_color_handler, "cloud");
    viewer.addPointCloud(keypointsXYZ, keypoints_color_handler, "keypoints");
    viewer.addPointCloudNormals<PointType, NormalType> ( keypointsXYZ,key_point_normals_, 1, 15, "normals");

    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "keypoints");
    std::vector<pcl::visualization::Camera> cam;

    while(!viewer.wasStopped ())
    {
        viewer.spinOnce ();
      }
    viewer.getCameras(cam);
    cout << "Cam: " << endl
         << " - pos: (" << cam[0].pos[0] << ", "    << cam[0].pos[1] << ", "    << cam[0].pos[2] << ")" << endl
         << " - view: ("    << cam[0].view[0] << ", "   << cam[0].view[1] << ", "   << cam[0].view[2] << ")"    << endl
         << " - focal: ("   << cam[0].focal[0] << ", "  << cam[0].focal[1] << ", "  << cam[0].focal[2] << ")"   << endl;

}

void Clouder::generateDownsampledCloud(){
    pcl::console::TicToc tt;tt.tic();
    pcl::PointCloud<PointType>::Ptr result (new pcl::PointCloud<PointType> ());

    pcl::UniformSampling<PointType> uniform_sampling;
    uniform_sampling.setInputCloud (cloud_);
    uniform_sampling.setRadiusSearch (downsampling_radius);
    uniform_sampling.filter (*result);
    this->downsampled_cloud_ = result;
    double t = tt.toc();
    pcl::console::print_value( "Downsampling the cloud takes %.3f\n\tFrom %d to %d points\n", t, this->size(), this->downsampled_cloud_->size() );
}

void Clouder::computeFPFHDescriptors() {
    pcl::console::TicToc tt; tt.tic();

    pcl::FPFHEstimationOMP<PointType, NormalType, FPFHType> fpfh;

    //auto keypointsXYZ = getKeypointsXYZ();
    //fpfh.setInputCloud (keypointsXYZ); //?
    auto keypoints = getKeypointsXYZ();
    fpfh.setInputCloud (keypoints);
    fpfh.setInputNormals (this->key_point_normals_);

    pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);

    fpfh.setSearchMethod (tree);

    pcl::PointCloud<FPFHType>::Ptr features (new pcl::PointCloud<FPFHType> ());

    // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
    fpfh.setKSearch(descriptor_k_neighbours_);

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

    // WIP: Local reference frames error
    pcl::PointCloud<pcl::ReferenceFrame>::Ptr frames (new pcl::PointCloud<pcl::ReferenceFrame> ());
    pcl::SHOTLocalReferenceFrameEstimation<PointType, pcl::ReferenceFrame> lrf_estimator;
    lrf_estimator.setRadiusSearch (descriptor_radius_);
    lrf_estimator.setInputCloud (getKeypointsXYZ());
    lrf_estimator.setSearchSurface(cloud_);
    lrf_estimator.compute (*frames);


    pcl::SHOTEstimationOMP<PointType, NormalType, SHOTType> shot;
    shot.setInputCloud (getKeypointsXYZ());
    shot.setInputNormals (point_normals_);
    shot.setSearchSurface(cloud_);
    shot.setRadiusSearch (descriptor_radius_);
    shot.setLRFRadius(rf_rad_);
    shot.setInputReferenceFrames(frames);
    // Output datasets
    pcl::PointCloud<SHOTType>::Ptr features (new pcl::PointCloud<SHOTType> ());
    // Compute the features
    shot.compute(*features);

    this->SHOT_descriptors_ = features;

    double t = tt.toc();
    pcl::console::print_value( "Signature of Histograms of Orientation takes %.3f\n", t );
    cout << features->points[0] << endl;
}
