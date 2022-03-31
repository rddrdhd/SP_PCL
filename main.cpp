#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/imgcodecs.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include "src/PGMbReader.h"

#define CAM_PPX 542.554688 //The ppx and ppy fields describe the pixel coordinates of the principal point (center of projection)
#define CAM_PPY 394.199219 // The ppx and ppy fields describe the pixel coordinates of the principal point (center of projection)
#define CAM_FX 737.078125 // The fx and fy fields describe the focal length of the image, as a multiple of pixel width and height
#define CAM_FY 738.515625 // The fx and fy fields describe the focal length of the image, as a multiple of pixel width and height
#define CAM_D_SCALE 5.0
#define CAM_W 1280
#define CAM_H 768
#define PGM_W 1024
#define PGM_H 768
#define PGM_SCALE 0.8
#define PGM_MAX_D 65535

typedef pcl::PointXYZRGBA PointType;
typedef pcl::Normal NormalType;
typedef pcl::ReferenceFrame RFType;
typedef pcl::SHOT352 DescriptorType;


std::string model_filename_;
std::string scene_filename_;

//Algorithm params
bool show_keypoints_ (false);
bool show_correspondences_ (false);
bool use_cloud_resolution_ (false);
bool use_hough_ (true);
float model_ss_ (0.01f);
float scene_ss_ (0.03f);
float rf_rad_ (0.015f);
float descr_rad_ (0.02f);
float cg_size_ (0.01f);
float cg_thresh_ (5.0f);
using namespace cv;
using namespace std;
void
showHelp (char *filename)
{
    std::cout << std::endl;
    std::cout << "***************************************************************************" << std::endl;
    std::cout << "*                                                                         *" << std::endl;
    std::cout << "*             Correspondence Grouping Tutorial - Usage Guide              *" << std::endl;
    std::cout << "*                                                                         *" << std::endl;
    std::cout << "***************************************************************************" << std::endl << std::endl;
    std::cout << "Usage: " << filename << " model_filename.pcd scene_filename.pcd [Options]" << std::endl << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "     -h:                     Show this help." << std::endl;
    std::cout << "     -k:                     Show used keypoints." << std::endl;
    std::cout << "     -c:                     Show used correspondences." << std::endl;
    std::cout << "     -r:                     Compute the model cloud resolution and multiply" << std::endl;
    std::cout << "                             each radius given by that value." << std::endl;
    std::cout << "     --algorithm (Hough|GC): Clustering algorithm used (default Hough)." << std::endl;
    std::cout << "     --model_ss val:         Model uniform sampling radius (default 0.01)" << std::endl;
    std::cout << "     --scene_ss val:         Scene uniform sampling radius (default 0.03)" << std::endl;
    std::cout << "     --rf_rad val:           Reference frame radius (default 0.015)" << std::endl;
    std::cout << "     --descr_rad val:        Descriptor radius (default 0.02)" << std::endl;
    std::cout << "     --cg_size val:          Cluster size (default 0.01)" << std::endl;
    std::cout << "     --cg_thresh val:        Clustering threshold (default 5)" << std::endl << std::endl;
}

void
parseCommandLine (int argc, char *argv[])
{
    //Show help
    if (pcl::console::find_switch (argc, argv, "-h"))
    {
        showHelp (argv[0]);
        exit (0);
    }

    //Model & scene filenames
    std::vector<int> filenames;
    filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
    if (filenames.size () != 2)
    {
        std::cout << "Filenames missing.\n";
        showHelp (argv[0]);
        exit (-1);
    }

    model_filename_ = argv[filenames[0]];
    scene_filename_ = argv[filenames[1]];

    //Program behavior
    if (pcl::console::find_switch (argc, argv, "-k"))
    {
        show_keypoints_ = true;
    }
    if (pcl::console::find_switch (argc, argv, "-c"))
    {
        show_correspondences_ = true;
    }
    if (pcl::console::find_switch (argc, argv, "-r"))
    {
        use_cloud_resolution_ = true;
    }

    std::string used_algorithm;
    if (pcl::console::parse_argument (argc, argv, "--algorithm", used_algorithm) != -1)
    {
        if (used_algorithm.compare ("Hough") == 0)
        {
            use_hough_ = true;
        }else if (used_algorithm.compare ("GC") == 0)
        {
            use_hough_ = false;
        }
        else
        {
            std::cout << "Wrong algorithm name.\n";
            showHelp (argv[0]);
            exit (-1);
        }
    }

    //General parameters
    pcl::console::parse_argument (argc, argv, "--model_ss", model_ss_);
    pcl::console::parse_argument (argc, argv, "--scene_ss", scene_ss_);
    pcl::console::parse_argument (argc, argv, "--rf_rad", rf_rad_);
    pcl::console::parse_argument (argc, argv, "--descr_rad", descr_rad_);
    pcl::console::parse_argument (argc, argv, "--cg_size", cg_size_);
    pcl::console::parse_argument (argc, argv, "--cg_thresh", cg_thresh_);
}

double
computeCloudResolution (const pcl::PointCloud<PointType>::ConstPtr &cloud)
{
    double res = 0.0;
    int n_points = 0;
    int nres;
    std::vector<int> indices (2);
    std::vector<float> sqr_distances (2);
    pcl::search::KdTree<PointType> tree;
    tree.setInputCloud (cloud);

    for (std::size_t i = 0; i < cloud->size (); ++i)
    {
        if (! std::isfinite ((*cloud)[i].x))
        {
            continue;
        }
        //Considering the second neighbor since the first is the point itself.
        nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
        if (nres == 2)
        {
            res += sqrt (sqr_distances[1]);
            ++n_points;
        }
    }
    if (n_points != 0)
    {
        res /= n_points;
    }
    return res;
}


std::string getBinary(char c, char c2) {
    std::string b;
    for (int i = 7; i >= 0; --i) {
        b.push_back(((c2 & (1 << i))? '1' : '0'));
    }
    for (int i = 7; i >= 0; --i) {
        b.push_back(((c & (1 << i))? '1' : '0'));
    }
    return b;
}
int getPixelDistance(const std::string& b) {
    unsigned long long value = std::stoull(b, nullptr, 2);
    return int(value / CAM_D_SCALE);
}

pcl::PointXYZ getPointXYZ(float depth, int xv, int yv){ //https://github.com/IntelRealSense/realsense-ros/issues/1342
    float xw, yw, zw;
    //float u = xv - (CAM_W/2);
    //float v = yv - (CAM_H/2);
    zw = depth;
    xw= (float(xv) - CAM_PPX) * zw / CAM_FX;
    yw = (float(yv) - CAM_PPY) * zw / CAM_FY;

    return {xw, yw, zw};
}

void savePCLPointCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud, std::string file_name){
    cout << "saving..."<<endl;
    pcl::io::savePCDFileASCII (file_name, cloud);
    std::cerr << "Saved " << cloud.size () << " data points to "<<file_name << std::endl;

    //for (const auto& point: cloud)
    //    std::cerr << "    " << point.x << " " << point.y << " " << point.z << std::endl;
}
void PGMtoPCD(const char* pgm_filepath, const char* pcd_filepath){
    printf("TODO"); // TODO use OpenCV for loading PGM
}
void savePointCloudFromPGM(const char* pgm_filepath, const char* pcd_filepath){
    auto* pgm = static_cast<PGMImage *>(malloc(sizeof(PGMImage)));
    const char* ipfile;
    ipfile = pgm_filepath;
    printf("file : %s\n", ipfile);

    // Process the image and print its details
    if (PGMbReader::openPGM(pgm, ipfile)){
        PGMbReader::printImageDetails(pgm, ipfile);
        pcl::PointCloud<pcl::PointXYZ> cloud(pgm->width, pgm->height);

        int points_count = int(pgm->width * pgm->height);
        cloud.resize(points_count);
        //printf("cam:%dx%d == pgm:%dx%d\n", CAM_W, CAM_H, pgm->width, pgm->height);
        std::vector<pcl::PointXYZ> points;
        int row, col;
        for(row = 0; row < (CAM_H); row++){ //768
            for(col = 0; col < (CAM_W); col++){ //1280
                auto bin_string = getBinary(pgm->data[row][col * 2], pgm->data[row][col * 2 + 1] ); // jas 16b = 2x8b
                auto mm_depth = getPixelDistance(bin_string); // 0-13107 mm
                auto coords = getPointXYZ(float(mm_depth), row, col);
                points.push_back(coords);
            }
        }
        //printf("%dx%d", row, col);
        //printf("points:%d (%dx%d)\n", points_count, pgm->width, pgm->height);
        for (int point_id = 0; point_id < points_count; ++point_id)
        {
            cloud[point_id].x = points[point_id].x;
            cloud[point_id].y = points[point_id].y;
            cloud[point_id].z = points[point_id].z;
        }

        savePCLPointCloud(cloud, pcd_filepath);
    }

}

int compareCloud(int argc, char *argv[]){
    parseCommandLine (argc, argv);

    pcl::PointCloud<PointType>::Ptr model (new pcl::PointCloud<PointType> ());
    pcl::PointCloud<PointType>::Ptr model_keypoints (new pcl::PointCloud<PointType> ());
    pcl::PointCloud<PointType>::Ptr scene (new pcl::PointCloud<PointType> ());
    pcl::PointCloud<PointType>::Ptr scene_keypoints (new pcl::PointCloud<PointType> ());
    pcl::PointCloud<NormalType>::Ptr model_normals (new pcl::PointCloud<NormalType> ());
    pcl::PointCloud<NormalType>::Ptr scene_normals (new pcl::PointCloud<NormalType> ());
    pcl::PointCloud<DescriptorType>::Ptr model_descriptors (new pcl::PointCloud<DescriptorType> ());
    pcl::PointCloud<DescriptorType>::Ptr scene_descriptors (new pcl::PointCloud<DescriptorType> ());

    //
    //  Load clouds
    //
    if (pcl::io::loadPCDFile (model_filename_, *model) < 0)
    {
        std::cout << "Error loading model cloud." << std::endl;
        showHelp (argv[0]);
        return (-1);
    }
    if (pcl::io::loadPCDFile (scene_filename_, *scene) < 0)
    {
        std::cout << "Error loading scene cloud." << std::endl;
        showHelp (argv[0]);
        return (-1);
    }

    //
    //  Set up resolution invariance
    //
    if (use_cloud_resolution_)
    {
        float resolution = static_cast<float> (computeCloudResolution (model));
        if (resolution != 0.0f)
        {
            model_ss_   *= resolution;
            scene_ss_   *= resolution;
            rf_rad_     *= resolution;
            descr_rad_  *= resolution;
            cg_size_    *= resolution;
        }

        std::cout << "Model resolution:       " << resolution << std::endl;
        std::cout << "Model sampling size:    " << model_ss_ << std::endl;
        std::cout << "Scene sampling size:    " << scene_ss_ << std::endl;
        std::cout << "LRF support radius:     " << rf_rad_ << std::endl;
        std::cout << "SHOT descriptor radius: " << descr_rad_ << std::endl;
        std::cout << "Clustering bin size:    " << cg_size_ << std::endl << std::endl;
    }

    //
    //  Compute Normals
    //
    pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
    norm_est.setKSearch (10);
    norm_est.setInputCloud (model);
    norm_est.compute (*model_normals);

    norm_est.setInputCloud (scene);
    norm_est.compute (*scene_normals);

    //
    //  Downsample Clouds to Extract keypoints
    //

    pcl::UniformSampling<PointType> uniform_sampling;
    uniform_sampling.setInputCloud (model);
    uniform_sampling.setRadiusSearch (model_ss_);
    uniform_sampling.filter (*model_keypoints);
    std::cout << "Model total points: " << model->size () << "; Selected Keypoints: " << model_keypoints->size () << std::endl;

    uniform_sampling.setInputCloud (scene);
    uniform_sampling.setRadiusSearch (scene_ss_);
    uniform_sampling.filter (*scene_keypoints);
    std::cout << "Scene total points: " << scene->size () << "; Selected Keypoints: " << scene_keypoints->size () << std::endl;


    //
    //  Compute Descriptor for keypoints
    //
    pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
    descr_est.setRadiusSearch (descr_rad_);

    descr_est.setInputCloud (model_keypoints);
    descr_est.setInputNormals (model_normals);
    descr_est.setSearchSurface (model);
    descr_est.compute (*model_descriptors);

    descr_est.setInputCloud (scene_keypoints);
    descr_est.setInputNormals (scene_normals);
    descr_est.setSearchSurface (scene);
    descr_est.compute (*scene_descriptors);

    //
    //  Find Model-Scene Correspondences with KdTree
    //
    pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());

    pcl::KdTreeFLANN<DescriptorType> match_search;
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
    }
    std::cout << "Correspondences found: " << model_scene_corrs->size () << std::endl;

    //
    //  Actual Clustering
    //
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
    std::vector<pcl::Correspondences> clustered_corrs;

    //  Using Hough3D
    if (use_hough_)
    {
        //
        //  Compute (Keypoints) Reference Frames only for Hough
        //
        pcl::PointCloud<RFType>::Ptr model_rf (new pcl::PointCloud<RFType> ());
        pcl::PointCloud<RFType>::Ptr scene_rf (new pcl::PointCloud<RFType> ());

        pcl::BOARDLocalReferenceFrameEstimation<PointType, NormalType, RFType> rf_est;
        rf_est.setFindHoles (true);
        rf_est.setRadiusSearch (rf_rad_);

        rf_est.setInputCloud (model_keypoints);
        rf_est.setInputNormals (model_normals);
        rf_est.setSearchSurface (model);
        rf_est.compute (*model_rf);

        rf_est.setInputCloud (scene_keypoints);
        rf_est.setInputNormals (scene_normals);
        rf_est.setSearchSurface (scene);
        rf_est.compute (*scene_rf);

        //  Clustering
        pcl::Hough3DGrouping<PointType, PointType, RFType, RFType> clusterer;
        clusterer.setHoughBinSize (cg_size_);
        clusterer.setHoughThreshold (cg_thresh_);
        clusterer.setUseInterpolation (true);
        clusterer.setUseDistanceWeight (false);

        clusterer.setInputCloud (model_keypoints);
        clusterer.setInputRf (model_rf);
        clusterer.setSceneCloud (scene_keypoints);
        clusterer.setSceneRf (scene_rf);
        clusterer.setModelSceneCorrespondences (model_scene_corrs);

        //clusterer.cluster (clustered_corrs);
        clusterer.recognize (rototranslations, clustered_corrs);
    }
    else // Using GeometricConsistency
    {
        pcl::GeometricConsistencyGrouping<PointType, PointType> gc_clusterer;
        gc_clusterer.setGCSize (cg_size_);
        gc_clusterer.setGCThreshold (cg_thresh_);

        gc_clusterer.setInputCloud (model_keypoints);
        gc_clusterer.setSceneCloud (scene_keypoints);
        gc_clusterer.setModelSceneCorrespondences (model_scene_corrs);

        //gc_clusterer.cluster (clustered_corrs);
        gc_clusterer.recognize (rototranslations, clustered_corrs);
    }

    //
    //  Output results
    //
    std::cout << "Model instances found: " << rototranslations.size () << std::endl;
    for (std::size_t i = 0; i < rototranslations.size (); ++i)
    {
        std::cout << "\n    Instance " << i + 1 << ":" << std::endl;
        std::cout << "        Correspondences belonging to this instance: " << clustered_corrs[i].size () << std::endl;

        // Print the rotation matrix and translation vector
        Eigen::Matrix3f rotation = rototranslations[i].block<3,3>(0, 0);
        Eigen::Vector3f translation = rototranslations[i].block<3,1>(0, 3);

        printf ("\n");
        printf ("            | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
        printf ("        R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
        printf ("            | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
        printf ("\n");
        printf ("        t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));
    }

    //
    //  Visualization
    //
    pcl::visualization::PCLVisualizer viewer ("Correspondence Grouping");
    viewer.addPointCloud (scene, "scene_cloud");

    pcl::PointCloud<PointType>::Ptr off_scene_model (new pcl::PointCloud<PointType> ());
    pcl::PointCloud<PointType>::Ptr off_scene_model_keypoints (new pcl::PointCloud<PointType> ());

    if (show_correspondences_ || show_keypoints_)
    {
        //  We are translating the model so that it doesn't end in the middle of the scene representation
        pcl::transformPointCloud (*model, *off_scene_model, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));
        pcl::transformPointCloud (*model_keypoints, *off_scene_model_keypoints, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));

        pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_color_handler (off_scene_model, 255, 255, 128);
        viewer.addPointCloud (off_scene_model, off_scene_model_color_handler, "off_scene_model");
    }

    if (show_keypoints_)
    {
        pcl::visualization::PointCloudColorHandlerCustom<PointType> scene_keypoints_color_handler (scene_keypoints, 0, 0, 255);
        viewer.addPointCloud (scene_keypoints, scene_keypoints_color_handler, "scene_keypoints");
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_keypoints");

        pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_keypoints_color_handler (off_scene_model_keypoints, 0, 0, 255);
        viewer.addPointCloud (off_scene_model_keypoints, off_scene_model_keypoints_color_handler, "off_scene_model_keypoints");
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "off_scene_model_keypoints");
    }

    for (std::size_t i = 0; i < rototranslations.size (); ++i)
    {
        pcl::PointCloud<PointType>::Ptr rotated_model (new pcl::PointCloud<PointType> ());
        pcl::transformPointCloud (*model, *rotated_model, rototranslations[i]);

        std::stringstream ss_cloud;
        ss_cloud << "instance" << i;

        pcl::visualization::PointCloudColorHandlerCustom<PointType> rotated_model_color_handler (rotated_model, 255, 0, 0);
        viewer.addPointCloud (rotated_model, rotated_model_color_handler, ss_cloud.str ());

        if (show_correspondences_)
        {
            for (std::size_t j = 0; j < clustered_corrs[i].size (); ++j)
            {
                std::stringstream ss_line;
                ss_line << "correspondence_line" << i << "_" << j;
                PointType& model_point = off_scene_model_keypoints->at (clustered_corrs[i][j].index_query);
                PointType& scene_point = scene_keypoints->at (clustered_corrs[i][j].index_match);

                //  We are drawing a line for each pair of clustered correspondences found between the model and the scene
                viewer.addLine<PointType, PointType> (model_point, scene_point, 0, 255, 0, ss_line.str ());
            }
        }
    }

    while (!viewer.wasStopped ())
    {
        viewer.spinOnce ();
    }
    return(0);
}

int main(int argc, char *argv[]){
    const char* filename = "/media/rddrdhd/Data/School/SP/project_c/pgm_files/20201017_102106_950_depth.pgm";
    const char* pcd_filepath = "../my_cloud.pcd";
    Mat img = imread(filename, -1); //1024x768, depth=2
    cout << img.rows << "x" << img.cols << "."<<img.depth()<<endl;
    pcl::PointCloud<pcl::PointXYZ> cloud(img.cols, img.rows);

    int points_count = int(img.cols * img.rows);
    cloud.resize(points_count);
    //printf("cam:%dx%d == pgm:%dx%d\n", CAM_W, CAM_H, pgm->width, pgm->height);
    std::vector<pcl::PointXYZ> points;
    int pc = 0;
    int x, y, depth;
    for(y = 0; y<(img.cols/2);y++){
        for(x = 0; x<img.rows;x++){
            pc++;
            depth = img.at<ushort>(y,x);
            pcl::PointXYZ point = getPointXYZ(depth,x,y);
            if(y==0 and x<10){

                cout << "\ni "<<x<<"\tx "<<point.x<<"\ty "<<point.y<<"\tz "<<point.z <<"\td "<< depth;

            }
            points.push_back(point);
        }
    }
    for (int point_id = 0; point_id < (points_count/2); ++point_id)
    {
        cloud[point_id].x = points[point_id].x;
        cloud[point_id].y = points[point_id].y;
        cloud[point_id].z = points[point_id].z;
    }

    savePCLPointCloud(cloud, pcd_filepath);
    //img.convertTo(img, CV_16U, 255.0 / 655536.0);
       // Mat idk;
        //img.convertTo(idk,CV_16U);
    imshow("Display window", img);
    waitKey(0);


    //
    //savePointCloudFromPGM("../pgm_files/20201017_102106_950_depth.pgm","../my_cloud.pcd");
    //compareCloud(argc, argv);
    printf("opencv version: %d.%d.%d\n",CV_VERSION_MAJOR,CV_VERSION_MINOR,CV_VERSION_REVISION);
    return 0;
}