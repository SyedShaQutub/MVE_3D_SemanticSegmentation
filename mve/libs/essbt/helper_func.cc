
#include <cstdlib>
#include <iostream>
#include <string>

#include "essbt/helper_func.h"
#include "mve/mesh.h"
#include "mve/mesh_io.h"

// #include <opencv2/highgui/highgui.hpp>
// #include <opencv2/imgproc.hpp>
// #include <opencv2/core/core.hpp>

using namespace cv;
ESSBT_NAMESPACE_BEGIN
bool mesh_sanity_check(essbt::AppSettings conf){
    mve::TriangleMesh::Ptr mesh;
    std::cout<<"\n\n insface_ide mesh_sanity_check \n\n";
    try
    {
        std::cout << "Loading mesh: " << conf.in_mesh << std::endl;
        mesh = mve::geom::load_mesh(conf.in_mesh);
    }
    catch (std::exception& e)
    {
        std::cerr << "Error loading mesh: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    /* Sanity checks. */
    if (mesh->get_vertices().empty())
    {
        std::cerr << "Error: Mesh is empty!" << std::endl;
        return EXIT_FAILURE;
    }

    if (mesh->get_faces().empty())
    {
        std::cerr << "Error: Components/faces cleanup "
            "requested, but mesh has no faces." << std::endl;
        return EXIT_FAILURE;
    }

    return true;
}

bool mesh_sanity_check_1(essbt::AppSettings conf){
    std::cout<<"\n\n insface_ide mesh_sanity_check_1 \n\n";
    return true;
}

void normVec_face(mve::TriangleMesh::Ptr mesh, float* vec, std::size_t face_id){
    mve::TriangleMesh::FaceList faces; 
    mve::MeshBase::VertexList vertices;
    mve::TriangleMesh::NormalList fnorm1; 
    faces = mesh->get_faces();
    vertices = mesh->get_vertices();
    fnorm1 = mesh->get_face_normals();

    
    //Normal vector calculation
    std::size_t ia = faces[face_id + 0];
    std::size_t ib = faces[face_id + 1];
    std::size_t ic = faces[face_id + 2];
    math::Vec3f const& a = vertices[ia];
    math::Vec3f const& b = vertices[ib];
    math::Vec3f const& c = vertices[ic];     

    
    math::Vec3f ab = b - a;
    //math::Vec3f bc = c - b;
    math::Vec3f ca = a - c;
    // Face normal
    math::Vec3f fn = ab.cross(-ca);
    float fnl = fn.norm();
    if (fnl != 0.0f)
        fn /= fnl;
    else
        std::cout << "Warning: Zero-length normals detected: "<< std::endl;
    for (int i = 0; i < 3; ++i)
        vec[i] = fn[i]; 
  
    // std::cout<<"\n\n get_face_normals FROM THE FACE NORMAL :"<<fnorm1.at(face_id);
    // std::cout<<"\n face normal : "<<vec;

}

// voface_id fill_viewing_direction(mve::CameraInfo cam, float* viewdir_vec){
//     //math::Vec3f viewdir;
    
//     for (int k = 0; k < 3; ++k)
//         //viewdir_vec[k] = cam.rot[6 + k];
//         std::cout<<"\n view dir "<<cam.rot[6 + k]<<"\n";
// }

essbt::facecam_ facecam_Superpos(mve::TriangleMesh::Ptr& mesh, mve::Bundle::ConstPtr const& bundle, mve::Scene::ViewList const& views, std::size_t face_id  ){

    //std::cout<<"\n face id is: "<<face_id;
    mve::CameraInfo cam;
    math::Vec3f viewdir;
    math::Vec3f campos;
    math::Vec3f face_normal;
    std::vector<unsigned int> camface_ids_;

    essbt::normVec_face(mesh, *face_normal, face_id);
    mve::TriangleMesh::FaceList faces = mesh->get_faces();
    mve::MeshBase::VertexList vertices = mesh->get_vertices();
    mve::Bundle::Cameras const& bundle_cams = bundle->get_cameras();
    //mve::Scene::ViewList const& views = scene->get_views();

    std::size_t ia = faces[face_id + 0];
    std::size_t ib = faces[face_id + 1];
    std::size_t ic = faces[face_id + 2];

    math::Vec3f const& a = vertices[ia];
    math::Vec3f const& b = vertices[ib];
    math::Vec3f const& c = vertices[ic];

    std::vector<math::Vec3f> Vertices;
    Vertices.push_back(a);
    Vertices.push_back(b);
    Vertices.push_back(c);


    std::vector<std::vector<math::Vec2f>> pixel_cordinates;
    std::vector<ushort> pix_vals;
    //TODO
    //hardcoded max_label
    ushort max_label = 35;
    bool local_valid_camera;
    for (std::size_t j = 0; j< bundle_cams.size(); j++){
    //for (std::size_t j = 0; j< 1; j++){

        cam = bundle_cams.at(j);
        // reject invalface_id camera 
        if(cam.flen == 0.0){
            // math::Vec2f zero_point(0.0f,0.0f);
            // std::vector <math::Vec2f> _image_points({zero_point,zero_point,zero_point});
            //std::cout<<" \n Invalid cam  \n";
            continue;
        }
        
        cam.fill_viewing_direction(*viewdir);
        float facecam_val=0;
        facecam_val = face_normal.dot(viewdir);

        if(facecam_val > 0){
            continue;
        }

        //traverse over all the valface_id cam face_ids taken from facecam structure 
        math::Matrix4f wtc;
        math::Matrix3f calib;

        mve::View::Ptr view = views.at(j);
        std::string filename = view->get_name();
        std::string direc = view->get_directory();

        filename = filename + ".jpg";
        std::string orig_image = "original";
        std::string semantic_image = "semantic";
        mve::ByteImage::Ptr image = view->get_byte_image(orig_image);
        mve::ByteImage::Ptr seman_image = view->get_byte_image(semantic_image);

        direc = direc + "/semantic.jpg";

        int width, height;
        width = image->width();
        height = image->height();

        //chec if point is in Frustum

        cam.fill_world_to_cam(*wtc);
        cam.fill_calibration(*calib, width, height);

        std::vector <math::Vec2f> image_points;
        // points from traingular face projecting on the image plane  
        
        for (std::size_t i = 0; i<3; ++i ){
        
            math::Vec3f cp = wtc.mult(Vertices.at(i), 1.0f);
            // check whether point lies in front of camera (pointInFrustum function)
            if (cp[2] <= 0.0f){
                local_valid_camera = true;
                break;
            }
            math::Vec3f sp;

            //sp = calib * wtc.mult(a, 1.0f);
            sp = calib * cp;
            float x = sp[0] / sp[2] - 0.5f;
            float y = sp[1] / sp[2] - 0.5f;

            
            local_valid_camera = !(x >= 0 && x <= width - 1 && y >= 0 && y <= height);

            image_points.push_back({x,y});
        }
        if(local_valid_camera){
            // math::Vec2f zero_point(0.0f,0.0f);
            // std::vector <math::Vec2f> _image_points({zero_point,zero_point,zero_point});
            // pixel_cordinates.push_back(_image_points);
            continue;
        }
        /* read pixel values from the polygon generated from crop operation-opencv; append all the values to one vector and then find the max of all the labels */
        ushort pix_val = essbt::seman_labels(direc, image_points); /* majority label from the projected traingle is consface_idered */
        local_valid_camera = false;
        camface_ids_.push_back(j);
        pixel_cordinates.push_back(image_points);
        pix_vals.push_back(pix_val);
    }
    /*
        camface_id and pixel cordinates correspond to each other at same location
     */
    if(!(camface_ids_.size() == pixel_cordinates.size())){
        std::cout<<"\n camface_ids "<<camface_ids_.size()<<"\n";
        std::cout<<"\n pixel_cordinates "<<pixel_cordinates.size()<<"\n";
        std::cout<<"\n mismatch in computation; check the Alogrithm \n";
        exit(0);
    }

    ushort major_label = essbt::majority_element_moore_voting_algorithm(pix_vals);
    //ushort major_label = 0;
    //TODO : retain mojority elements from each camera and return them as a vector
    // if(!local_valid_camera){
    //     essbt::facecam_ facecam_superpos_(face_id, camface_ids_, pixel_cordinates, major_label);
    // }
    // else if(local_valid_camera){
    //     math::Vec2f zero_point(0.0f,0.0f);
    //     std::vector <math::Vec2f> invalid_image_points({zero_point,zero_point,zero_point});
    //     std::vector<std::vector<math::Vec2f>> invalid_pixel_cordinates({invalid_image_points});
    //     std::cout<<" \n No valid pixel cordinates matched  \n";
    //     essbt::facecam_ facecam_superpos_(face_id, camface_ids_, invalid_pixel_cordinates, max_label);
    // }
    //essbt::facecam_ Facecam_superpos_ = facecam_superpos_;
    essbt::facecam_ facecam_superpos_(face_id, camface_ids_, pixel_cordinates, major_label);
    return facecam_superpos_;
}

essbt::facecam_ facecam_Superpos_new(mve::TriangleMesh::Ptr& mesh, mve::Bundle::ConstPtr const& bundle, mve::Scene::ViewList const& views, std::vector<cv::Mat> const&  image_list, std::size_t face_id  ){

    //std::cout<<"\n face id is: "<<face_id;
    mve::CameraInfo cam;
    math::Vec3f viewdir;
    math::Vec3f campos;
    math::Vec3f face_normal;
    std::vector<unsigned int> camface_ids_;

    essbt::normVec_face(mesh, *face_normal, face_id);
    mve::TriangleMesh::FaceList faces = mesh->get_faces();
    mve::MeshBase::VertexList vertices = mesh->get_vertices();
    mve::Bundle::Cameras const& bundle_cams = bundle->get_cameras();
    //mve::Scene::ViewList const& views = scene->get_views();

    std::size_t ia = faces[face_id + 0];
    std::size_t ib = faces[face_id + 1];
    std::size_t ic = faces[face_id + 2];

    math::Vec3f const& a = vertices[ia];
    math::Vec3f const& b = vertices[ib];
    math::Vec3f const& c = vertices[ic];

    std::vector<math::Vec3f> Vertices;
    Vertices.push_back(a);
    Vertices.push_back(b);
    Vertices.push_back(c);


    std::vector<std::vector<math::Vec2f>> pixel_cordinates;
    std::vector<ushort> pix_vals;
    //TODO @ssq
    //hardcoded max_label
    ushort max_label = 35;
    bool local_valid_camera = false;
    bool global_valid_camera = false;
    math::Matrix4f wtc;
    math::Matrix3f calib;    
    for (std::size_t j = 0; j< bundle_cams.size(); j++){
    //for (std::size_t j = 0; j< 1; j++){

        cam = bundle_cams.at(j);
        // reject invalface_id camera 
        if(cam.flen == 0.0){
            // math::Vec2f zero_point(0.0f,0.0f);
            // std::vector <math::Vec2f> _image_points({zero_point,zero_point,zero_point});
            //std::cout<<" \n Invalid cam  \n";
            local_valid_camera = false;
            continue;
        }
        
        cam.fill_viewing_direction(*viewdir);
        float facecam_val=0;
        facecam_val = face_normal.dot(-viewdir); // facecam_val < 0; facenormal and viewdir are facing each other

        if(facecam_val < 0){
            local_valid_camera = false;
            continue;
        }

        //traverse over all the valface_id cam face_ids taken from facecam structure 
        mve::View::Ptr view = views.at(j);
        std::string direc = view->get_directory();
        std::string orig_image = "original";
        mve::ByteImage::Ptr image = view->get_byte_image(orig_image);
        int width, height;
        width = image->width();
        height = image->height();

        //chec if point is in Frustum
        cam.fill_world_to_cam(*wtc);
        cam.fill_calibration(*calib, width, height);

        std::vector <math::Vec2f> image_points;
        // points from traingular face projecting on the image plane  
        
        for (std::size_t i = 0; i<3; ++i ){
        
            math::Vec3f cp = wtc.mult(Vertices.at(i), 1.0f);
            // check whether point lies in front of camera (pointInFrustum function)
            if (cp[2] <= 0.0f){
                local_valid_camera = false;
                break;
            }
            math::Vec3f sp;

            //sp = calib * wtc.mult(a, 1.0f);
            sp = calib * cp;
            float x = sp[0] / sp[2] - 0.5f;
            float y = sp[1] / sp[2] - 0.5f;

            
            local_valid_camera = (x >= 0 && x <= width - 1 && y >= 0 && y <= height);

            image_points.push_back({x,y});
        }
        if(local_valid_camera == false){
            // math::Vec2f zero_point(0.0f,0.0f);
            // std::vector <math::Vec2f> _image_points({zero_point,zero_point,zero_point});
            // pixel_cordinates.push_back(_image_points);
            continue;
        }
        /* read pixel values from the polygon generated from crop operation-opencv; append all the values to one vector and then find the max of all the labels */
        global_valid_camera = true;
        //local_valid_camera  = false;
        Mat const& read_image = image_list.at(j);
        ushort pix_val = essbt::seman_labels_new(direc, image_points, read_image); /* majority label from the projected traingle is consface_idered */
        camface_ids_.push_back(j);
        pixel_cordinates.push_back(image_points);
        pix_vals.push_back(pix_val);
    }
    /* camface_id and pixel cordinates correspond to each other at same location */

    if(global_valid_camera == false){
        std::cout<<"\n invalid faceid ::";
        return essbt::facecam_(face_id, camface_ids_, pixel_cordinates, INVALID_LABEL);
        //return facecam_superpos_;
    }
    if(!(camface_ids_.size() == pixel_cordinates.size())){
        std::cout<<"\n camface_ids "<<camface_ids_.size()<<"\n";
        std::cout<<"\n pixel_cordinates "<<pixel_cordinates.size()<<"\n";
        std::cout<<"\n mismatch in computation; check the Alogrithm \n";
        exit(0);
    }
    ushort major_label = essbt::majority_element_moore_voting_algorithm(pix_vals);
    return essbt::facecam_(face_id, camface_ids_, pixel_cordinates, major_label);
    //return facecam_superpos_;
}


ushort seman_labels( std::string const& image_path, std::vector <math::Vec2f> const image_points ){

        Mat image;
        image = imread(image_path, CV_LOAD_IMAGE_GRAYSCALE);   // Read the greyscale image file
        cv::Point corners[3];

        corners[0] = Point( image_points.at(0)(0), image_points.at(0)(1) );
        corners[1] = Point( image_points.at(1)(0), image_points.at(1)(1) );
        corners[2] = Point( image_points.at(2)(0), image_points.at(2)(1) );

        // corners[0] = Point( 0, 0 );
        // corners[1] = Point( 5, 0);
        // corners[2] = Point( 5, 5 );


        const Point* corner_list[] = { corners };

        int num_points = 3;
        int num_polygons = 1;
        int line_type = LINE_4;

        cv::Mat mask(image.rows, image.cols, CV_8UC1, cv::Scalar(0));

        cv::fillPoly( mask, corner_list, &num_points, num_polygons, cv::Scalar(255),  line_type);

        // cv::Mat result;
        // cv::Mat result_white(image.rows, image.cols, CV_8UC1, cv::Scalar(255));
        // cv::bitwise_and(image, mask, result_white, result);

        std::vector<cv::Point> locations;   // output, locations of non-zero pixels
        cv::findNonZero(mask, locations);
        std::vector<ushort> pix_vals;
        // access pixel coordinates
        for (std::size_t i = 0; i<locations.size(); ++i ){
            Point pnt = locations[i];
            char pix_val = image.at<char>(pnt.y,pnt.x);
            pix_vals.push_back((int)pix_val);                       
        }
    // namedWindow( "Display window", WINDOW_NORMAL );// Create a window for display.
    // imshow( "Display window", mask ); 
    // waitKey(0);
    std::sort(pix_vals.begin(), pix_vals.end());
    return essbt::majority_element_moore_voting_algorithm(pix_vals);


}

ushort seman_labels_new( std::string const& image_path, std::vector <math::Vec2f> const image_points, cv::Mat const& image){

        //Mat image = image_list.at(j);
        //image_list = imread(image_path, CV_LOAD_IMAGE_GRAYSCALE);   // Read the greyscale image file
        cv::Point corners[3];

        corners[0] = Point( image_points.at(0)(0), image_points.at(0)(1) );
        corners[1] = Point( image_points.at(1)(0), image_points.at(1)(1) );
        corners[2] = Point( image_points.at(2)(0), image_points.at(2)(1) );

        // corners[0] = Point( 0, 0 );
        // corners[1] = Point( 5, 0);
        // corners[2] = Point( 5, 5 );


        const Point* corner_list[] = { corners };

        int num_points = 3;
        int num_polygons = 1;
        int line_type = LINE_4;

        cv::Mat mask(image.rows, image.cols, CV_8UC1, cv::Scalar(0));

        cv::fillPoly( mask, corner_list, &num_points, num_polygons, cv::Scalar(255),  line_type);

        // cv::Mat result;
        // cv::Mat result_white(image.rows, image.cols, CV_8UC1, cv::Scalar(255));
        // cv::bitwise_and(image, mask, result_white, result);

        std::vector<cv::Point> locations;   // output, locations of non-zero pixels
        cv::findNonZero(mask, locations);
        std::vector<ushort> pix_vals;
        // access pixel coordinates
        for (std::size_t i = 0; i<locations.size(); ++i ){
            Point pnt = locations[i];
            char pix_val = image.at<char>(pnt.y,pnt.x);
            pix_vals.push_back((int)pix_val);                       
        }
    // namedWindow( "Display window", WINDOW_NORMAL );// Create a window for display.
    // imshow( "Display window", mask ); 
    // waitKey(0);
    std::sort(pix_vals.begin(), pix_vals.end());
    return essbt::majority_element_moore_voting_algorithm(pix_vals);

}


ushort majority_element_moore_voting_algorithm(std::vector<ushort> const pixel_vals) {

	ushort major=pixel_vals[0];
	int count = 1;
	for(uint i=1; i<pixel_vals.size();i++)
	{
		if(count==0)
		{
			count++;
			major=pixel_vals[i];
    	}
		else if(major==pixel_vals[i])
		{
			count++;
		}
		else
		{ 
			count--;
		}
	}
	return major;
}

std::vector<cv::Mat> image_read(mve::Bundle::ConstPtr const& bundle, mve::Scene::ViewList const& views){
    
    std::vector <Mat> image_list;
    //std::vector<Mat> *image_list;
    mve::Bundle::Cameras const& bundle_cams = bundle->get_cameras();
    Mat image;
    for (std::size_t j = 0; j< bundle_cams.size(); j++){

        mve::View::Ptr view = views.at(j);
        std::string filename = view->get_name();
        std::string direc = view->get_directory();
        direc = direc + "/semantic.jpg";
        image = imread(direc, CV_LOAD_IMAGE_GRAYSCALE);
        image_list.push_back(image);
    }
    std::cout<<"\nInside Image read\n";
    return image_list;
}

void show_image(std::vector<Mat> image_list, uint index ){

    Mat image = image_list.at(index);
    namedWindow( "Display window", WINDOW_AUTOSIZE );    
    imshow("Display window", image);
    waitKey(0);
}

ESSBT_NAMESPACE_END
